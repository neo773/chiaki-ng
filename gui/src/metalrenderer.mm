// Native Metal video renderer for macOS.
//
// Frames: VideoToolbox frames are IOSurface-backed CVPixelBuffers, wrapped as per-plane
// MTLTextures through a CVMetalTextureCache (no copy). Software frames are uploaded into a small
// ring of shared-storage textures.
//
// One full-screen pass per present: Catmull-Rom luma + bilinear chroma, YUV->RGB with the stream's
// matrix and range, letterboxing, and the Qt Quick UI blended on top only when it has content.
//
// GPU safety: nothing here waits on the GPU without a bound. At most two command buffers are in
// flight (semaphore waited with a timeout), nextDrawable has the layer's 1 s timeout, and every
// command buffer's completion handler owns the frames and textures it reads, so the renderer can
// be destroyed without waiting for the GPU.

#include "metalrenderer.h"

#import <AppKit/AppKit.h>
#import <CoreVideo/CoreVideo.h>
#import <Metal/Metal.h>
#import <QuartzCore/QuartzCore.h>
#include <os/signpost.h>

#include <chiaki/latencystats.h>
#include <chiaki/time.h>

#include <QDebug>
#include <QLoggingCategory>

#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstring>
#include <memory>
#include <mutex>

extern "C" {
#include <libavutil/frame.h>
#include <libavutil/mastering_display_metadata.h>
#include <libavutil/pixdesc.h>
#include <libavutil/pixfmt.h>
}

#if !__has_feature(objc_arc)
#error "metalrenderer.mm must be compiled with -fobjc-arc"
#endif

Q_DECLARE_LOGGING_CATEGORY(chiakiGui);

namespace {

constexpr long kMaxCommandBuffersInFlight = 2;
// Bound for waiting on a free command buffer slot; the render thread retries instead of blocking
constexpr int64_t kInFlightWaitNs = 50 * NSEC_PER_MSEC;
constexpr int kUploadRingSize = 3;
// BT.2408 reference white, used to place the SDR UI inside an HDR (PQ) frame
constexpr float kSdrWhiteNits = 203.0f;

const char *const kShaderSource = R"METAL(
#include <metal_stdlib>
using namespace metal;

struct Params {
    float4 dst_rect;    // video destination in drawable pixels: x, y, w, h
    float4 src_rect;    // video source in luma texels: x, y, w, h
    float4 luma_size;   // w, h, 1/w, 1/h
    float4 chroma_size; // w, h, 1/w, 1/h
    float4 chroma_map;  // xy: siting offset in chroma texels, zw: luma->chroma texel scale
    float4 m0;          // rgb = (dot(m0, yuv1), dot(m1, yuv1), dot(m2, yuv1))
    float4 m1;
    float4 m2;
    float4 viewport;    // w, h, 1/w, 1/h of the drawable
    uint flags;
    float sdr_white;    // reference white for the UI in PQ output, as a fraction of 10000 nits
    float bit_scale;    // turns sampled values into code / (2^depth - 1)
    float pad;
};

constant uint kFlagVideo = 1u;
constant uint kFlagTriplanar = 2u;
constant uint kFlagBicubic = 4u;
constant uint kFlagOverlay = 8u;
constant uint kFlagPQ = 16u;

struct VOut {
    float4 pos [[position]];
};

vertex VOut chiaki_fullscreen_vertex(uint vid [[vertex_id]])
{
    float2 p = float2(float((vid << 1) & 2u), float(vid & 2u));
    VOut o;
    o.pos = float4(p * 2.0 - 1.0, 0.0, 1.0);
    return o;
}

// Catmull-Rom in 9 bilinear taps (coord in texels)
static float sample_luma(texture2d<float> t, sampler s, float2 coord, float4 size, bool bicubic)
{
    if (!bicubic)
        return t.sample(s, coord * size.zw).r;
    float2 tp1 = floor(coord - 0.5) + 0.5;
    float2 f = coord - tp1;
    float2 w0 = f * (-0.5 + f * (1.0 - 0.5 * f));
    float2 w1 = 1.0 + f * f * (-2.5 + 1.5 * f);
    float2 w2 = f * (0.5 + f * (2.0 - 1.5 * f));
    float2 w3 = f * f * (-0.5 + 0.5 * f);
    float2 w12 = w1 + w2;
    float2 t0 = (tp1 - 1.0) * size.zw;
    float2 t3 = (tp1 + 2.0) * size.zw;
    float2 t12 = (tp1 + w2 / w12) * size.zw;
    float r = 0.0;
    r += t.sample(s, float2(t0.x, t0.y)).r * w0.x * w0.y;
    r += t.sample(s, float2(t12.x, t0.y)).r * w12.x * w0.y;
    r += t.sample(s, float2(t3.x, t0.y)).r * w3.x * w0.y;
    r += t.sample(s, float2(t0.x, t12.y)).r * w0.x * w12.y;
    r += t.sample(s, float2(t12.x, t12.y)).r * w12.x * w12.y;
    r += t.sample(s, float2(t3.x, t12.y)).r * w3.x * w12.y;
    r += t.sample(s, float2(t0.x, t3.y)).r * w0.x * w3.y;
    r += t.sample(s, float2(t12.x, t3.y)).r * w12.x * w3.y;
    r += t.sample(s, float2(t3.x, t3.y)).r * w3.x * w3.y;
    return r;
}

constant float kPqM1 = 0.1593017578125;
constant float kPqM2 = 78.84375;
constant float kPqC1 = 0.8359375;
constant float kPqC2 = 18.8515625;
constant float kPqC3 = 18.6875;

static float3 pq_to_linear(float3 e)
{
    float3 p = pow(max(e, 0.0), 1.0 / kPqM2);
    return pow(max(p - kPqC1, 0.0) / (kPqC2 - kPqC3 * p), 1.0 / kPqM1);
}

static float3 linear_to_pq(float3 l)
{
    float3 p = pow(max(l, 0.0), kPqM1);
    return pow((kPqC1 + kPqC2 * p) / (1.0 + kPqC3 * p), kPqM2);
}

static float3 srgb_to_linear(float3 c)
{
    return select(pow((c + 0.055) / 1.055, 2.4), c / 12.92, c <= 0.04045);
}

static float3 bt709_to_bt2020(float3 c)
{
    return float3(dot(float3(0.6274, 0.3293, 0.0433), c),
                  dot(float3(0.0691, 0.9195, 0.0114), c),
                  dot(float3(0.0164, 0.0880, 0.8956), c));
}

fragment float4 chiaki_composite_fragment(VOut in [[stage_in]],
                                          constant Params &p [[buffer(0)]],
                                          texture2d<float> tex_y [[texture(0)]],
                                          texture2d<float> tex_u [[texture(1)]],
                                          texture2d<float> tex_v [[texture(2)]],
                                          texture2d<float> tex_ui [[texture(3)]])
{
    constexpr sampler lin(coord::normalized, address::clamp_to_edge, filter::linear);
    constexpr sampler nearest(coord::normalized, address::clamp_to_edge, filter::nearest);

    float2 px = in.pos.xy;
    float3 rgb = float3(0.0);
    if ((p.flags & kFlagVideo) != 0u) {
        float2 rel = (px - p.dst_rect.xy) / p.dst_rect.zw;
        if (all(rel >= 0.0) && all(rel <= 1.0)) {
            float2 lc = p.src_rect.xy + rel * p.src_rect.zw;
            float y = sample_luma(tex_y, lin, lc, p.luma_size, (p.flags & kFlagBicubic) != 0u);
            float2 cuv = (lc * p.chroma_map.zw + p.chroma_map.xy) * p.chroma_size.zw;
            float2 c;
            if ((p.flags & kFlagTriplanar) != 0u)
                c = float2(tex_u.sample(lin, cuv).r, tex_v.sample(lin, cuv).r);
            else
                c = tex_u.sample(lin, cuv).rg;
            float4 yuv = float4(float3(y, c) * p.bit_scale, 1.0);
            rgb = saturate(float3(dot(p.m0, yuv), dot(p.m1, yuv), dot(p.m2, yuv)));
        }
    }
    if ((p.flags & kFlagOverlay) != 0u) {
        // Qt Quick renders premultiplied sRGB
        float4 ui = tex_ui.sample(nearest, px * p.viewport.zw);
        if (ui.a > 0.0) {
            if ((p.flags & kFlagPQ) != 0u) {
                float3 ui_lin = bt709_to_bt2020(srgb_to_linear(ui.rgb / max(ui.a, 1e-4))) * p.sdr_white;
                rgb = linear_to_pq(pq_to_linear(rgb) * (1.0 - ui.a) + ui_lin * ui.a);
            } else {
                rgb = rgb * (1.0 - ui.a) + ui.rgb;
            }
        }
    }
    return float4(rgb, 1.0);
}

// Sets *flag when any pixel of the UI texture is not fully transparent
kernel void chiaki_overlay_alpha_check(texture2d<float, access::read> ui [[texture(0)]],
                                       device atomic_uint *flag [[buffer(0)]],
                                       uint2 gid [[thread_position_in_grid]])
{
    bool has = false;
    if (gid.x < ui.get_width() && gid.y < ui.get_height())
        has = ui.read(gid).a > 0.0;
    if (simd_any(has) && simd_is_first())
        atomic_store_explicit(flag, 1u, memory_order_relaxed);
}
)METAL";

enum : uint32_t {
    kFlagVideo = 1u,
    kFlagTriplanar = 2u,
    kFlagBicubic = 4u,
    kFlagOverlay = 8u,
    kFlagPQ = 16u,
};

// Must match Params in the shader
struct ShaderParams {
    float dst_rect[4];
    float src_rect[4];
    float luma_size[4];
    float chroma_size[4];
    float chroma_map[4];
    float m0[4];
    float m1[4];
    float m2[4];
    float viewport[4];
    uint32_t flags;
    float sdr_white;
    float bit_scale;
    float pad;
};
static_assert(sizeof(ShaderParams) == 160, "ShaderParams must match the Metal struct layout");

enum class LayerMode {
    SDR,
    PQ,
    HLG,
};

struct GpuStats {
    std::atomic<uint64_t> gpu_us_sum{0};
    std::atomic<uint64_t> gpu_us_max{0};
    std::atomic<uint32_t> frames{0};
    std::atomic<uint32_t> errors{0};
    // Result of the last UI alpha check: 0 empty, 1 has content (or not checked yet)
    std::atomic<int> overlay_has_content{0};
    std::atomic<uint32_t> overlay_generation{0};

    // [latency] stats, filled by drawable presented handlers (any thread)
    std::mutex latency_mutex;
    ChiakiLatencyStats handoff{}; // decoded -> handed to submitFrame
    ChiakiLatencyStats render{}; // submitFrame -> command buffer commit
    ChiakiLatencyStats display{}; // commit -> on glass (presentedTime)
    ChiakiLatencyStats total{}; // first network packet -> on glass
    std::atomic<uint32_t> dropped_before_display{0}; // superseded by a newer frame before render
    std::atomic<uint32_t> not_presented{0}; // rendered, but the drawable was never shown
};

// Per-frame timestamps (chiaki_time_now_monotonic_us(), 0 if unknown) for the [latency] log
struct FrameTiming {
    uint64_t first_packet_us = 0;
    uint64_t decoded_us = 0;
    uint64_t handoff_us = 0;
    os_signpost_id_t signpost = OS_SIGNPOST_ID_NULL;
};

struct MappedFrame {
    AVFrame *frame = nullptr;
    __strong id<MTLTexture> planes[3] = {nil, nil, nil};
    CVMetalTextureRef cv[2] = {nullptr, nullptr};
    bool triplanar = false;
    bool full_range = false;
    int depth = 8;
    float bit_scale = 1.0f;

    void reset()
    {
        av_frame_free(&frame);
        for (auto &tex : cv) {
            if (tex)
                CFRelease(tex);
            tex = nullptr;
        }
        for (auto &plane : planes)
            plane = nil;
        triplanar = false;
        full_range = false;
        depth = 8;
        bit_scale = 1.0f;
    }

    void takeFrom(MappedFrame &other)
    {
        reset();
        frame = other.frame;
        other.frame = nullptr;
        for (int i = 0; i < 3; i++) {
            planes[i] = other.planes[i];
            other.planes[i] = nil;
        }
        for (int i = 0; i < 2; i++) {
            cv[i] = other.cv[i];
            other.cv[i] = nullptr;
        }
        triplanar = other.triplanar;
        full_range = other.full_range;
        depth = other.depth;
        bit_scale = other.bit_scale;
    }
};

struct UploadSet {
    __strong id<MTLTexture> tex[3] = {nil, nil, nil};
};

void putBE16(uint8_t *p, uint32_t v)
{
    v = std::min<uint32_t>(v, 0xffff);
    p[0] = static_cast<uint8_t>(v >> 8);
    p[1] = static_cast<uint8_t>(v);
}

void putBE32(uint8_t *p, uint64_t v)
{
    v = std::min<uint64_t>(v, 0xffffffffull);
    p[0] = static_cast<uint8_t>(v >> 24);
    p[1] = static_cast<uint8_t>(v >> 16);
    p[2] = static_cast<uint8_t>(v >> 8);
    p[3] = static_cast<uint8_t>(v);
}

uint32_t rationalToUnits(AVRational r, double unit)
{
    if (r.den == 0)
        return 0;
    const double v = av_q2d(r) / unit + 0.5;
    return v > 0.0 ? static_cast<uint32_t>(v) : 0;
}

// rgb = M * (y, cb, cr, 1) for samples already scaled to code / (2^depth - 1)
void computeYuvMatrix(const AVFrame *frame, bool full_range, int depth, float m[3][4])
{
    double kr = 0.2126, kb = 0.0722; // BT.709
    const AVColorTransferCharacteristic trc = frame->color_trc;
    switch (frame->colorspace) {
    case AVCOL_SPC_BT709:
        break;
    case AVCOL_SPC_BT2020_NCL:
    case AVCOL_SPC_BT2020_CL:
        kr = 0.2627;
        kb = 0.0593;
        break;
    case AVCOL_SPC_SMPTE170M:
    case AVCOL_SPC_BT470BG:
    case AVCOL_SPC_FCC:
        kr = 0.299;
        kb = 0.114;
        break;
    case AVCOL_SPC_SMPTE240M:
        kr = 0.212;
        kb = 0.087;
        break;
    default:
        if (trc == AVCOL_TRC_SMPTE2084 || trc == AVCOL_TRC_ARIB_STD_B67 || frame->color_primaries == AVCOL_PRI_BT2020) {
            kr = 0.2627;
            kb = 0.0593;
        } else if (frame->height > 0 && frame->height < 720) {
            kr = 0.299;
            kb = 0.114;
        }
        break;
    }
    const double kg = 1.0 - kr - kb;

    const double max_code = static_cast<double>((1u << depth) - 1u);
    const double s = static_cast<double>(1u << (depth - 8));
    double y_black, y_range, c_center, c_range;
    if (full_range) {
        y_black = 0.0;
        y_range = 1.0;
        c_center = (128.0 * s) / max_code;
        c_range = 1.0;
    } else {
        y_black = (16.0 * s) / max_code;
        y_range = (219.0 * s) / max_code;
        c_center = (128.0 * s) / max_code;
        c_range = (224.0 * s) / max_code;
    }

    const double rv = 2.0 * (1.0 - kr);
    const double bu = 2.0 * (1.0 - kb);
    const double gu = -2.0 * kb * (1.0 - kb) / kg;
    const double gv = -2.0 * kr * (1.0 - kr) / kg;
    const double ys = 1.0 / y_range;
    const double y0 = -y_black / y_range;

    const double rows[3][3] = {
        { ys, 0.0, rv / c_range },
        { ys, gu / c_range, gv / c_range },
        { ys, bu / c_range, 0.0 },
    };
    for (int i = 0; i < 3; i++) {
        m[i][0] = static_cast<float>(rows[i][0]);
        m[i][1] = static_cast<float>(rows[i][1]);
        m[i][2] = static_cast<float>(rows[i][2]);
        m[i][3] = static_cast<float>(y0 - (rows[i][1] + rows[i][2]) * c_center);
    }
}

LayerMode layerModeForFrame(const AVFrame *frame)
{
    if (!frame)
        return LayerMode::SDR;
    if (frame->color_trc == AVCOL_TRC_SMPTE2084)
        return LayerMode::PQ;
    if (frame->color_trc == AVCOL_TRC_ARIB_STD_B67)
        return LayerMode::HLG;
    return LayerMode::SDR;
}

} // namespace

struct MetalRenderer::Private {
    __strong id<MTLDevice> device = nil;
    __strong id<MTLCommandQueue> queue = nil;
    __strong id<MTLLibrary> library = nil;
    __strong id<MTLFunction> vertex_fn = nil;
    __strong id<MTLFunction> fragment_fn = nil;
    __strong id<MTLComputePipelineState> overlay_check = nil;
    __strong NSMutableDictionary<NSNumber *, id<MTLRenderPipelineState>> *pipelines = nil;
    __strong dispatch_semaphore_t in_flight = nil;
    __strong id<MTLTexture> dummy_plane = nil;
    __strong id<MTLTexture> dummy_rgba = nil;
    CVMetalTextureCacheRef texture_cache = nullptr;

    // Layer: set once on the GUI thread, then read on the render thread
    __strong CAMetalLayer *layer = nil;
    std::atomic<bool> attached{false};
    LayerMode layer_mode = LayerMode::SDR;
    uint8_t hdr_display_info[24] = {};
    uint8_t hdr_content_info[4] = {};
    bool hdr_display_info_valid = false;
    bool hdr_content_info_valid = false;
    bool hdr_metadata_dirty = false;
    float hdr_max_nits = 1000.0f;

    // UI overlay (render thread)
    __strong id<MTLTexture> overlay = nil;
    __strong NSMutableArray<id<MTLTexture>> *retired_overlays = nil;
    QSize overlay_size;
    bool overlay_check_pending = false;

    // Frame hand-off (any thread)
    mutable std::mutex frame_mutex;
    AVFrame *pending = nullptr;
    FrameTiming pending_timing;
    std::atomic<bool> clear_requested{false};

    // "render" intervals (submitFrame -> commit) for Instruments' Points of Interest
    os_log_t signpost_log = os_log_create("chiaki-ng", OS_LOG_CATEGORY_POINTS_OF_INTEREST);

    // Render thread
    MappedFrame current;
    FrameTiming current_timing;
    // current was mapped but not rendered yet
    bool current_fresh = false;
    double last_latency_log_s = 0.0;
    UploadSet upload_ring[kUploadRingSize];
    int upload_index = 0;
    bool logged_unsupported_format = false;
    bool logged_zero_copy_failure = false;

    std::shared_ptr<GpuStats> stats = std::make_shared<GpuStats>();
    double last_stats_log_s = 0.0;
    double last_backpressure_log_s = 0.0;
    uint32_t backpressure_events = 0;
    uint32_t last_logged_errors = 0;

    id<MTLRenderPipelineState> pipelineFor(MTLPixelFormat format, QString *error = nullptr)
    {
        NSNumber *key = @(static_cast<NSUInteger>(format));
        id<MTLRenderPipelineState> pso = pipelines[key];
        if (pso)
            return pso;
        MTLRenderPipelineDescriptor *desc = [MTLRenderPipelineDescriptor new];
        desc.label = @"chiaki composite";
        desc.vertexFunction = vertex_fn;
        desc.fragmentFunction = fragment_fn;
        desc.colorAttachments[0].pixelFormat = format;
        NSError *ns_error = nil;
        pso = [device newRenderPipelineStateWithDescriptor:desc error:&ns_error];
        if (!pso) {
            const QString msg = QStringLiteral("Failed to create Metal pipeline for pixel format %1: %2")
                .arg(static_cast<qulonglong>(format))
                .arg(QString::fromNSString(ns_error.localizedDescription));
            if (error)
                *error = msg;
            qCWarning(chiakiGui).noquote() << msg;
            return nil;
        }
        pipelines[key] = pso;
        return pso;
    }

    id<MTLTexture> makePlaneTexture(MTLPixelFormat format, NSUInteger w, NSUInteger h)
    {
        MTLTextureDescriptor *td = [MTLTextureDescriptor texture2DDescriptorWithPixelFormat:format
                                                                                      width:w
                                                                                     height:h
                                                                                  mipmapped:NO];
        td.usage = MTLTextureUsageShaderRead;
        td.storageMode = MTLStorageModeShared;
        return [device newTextureWithDescriptor:td];
    }

    bool mapVideoToolbox(AVFrame *frame, MappedFrame &out)
    {
        CVPixelBufferRef pb = reinterpret_cast<CVPixelBufferRef>(frame->data[3]);
        if (!pb || !CVPixelBufferGetIOSurface(pb) || CVPixelBufferGetPlaneCount(pb) != 2)
            return false;

        bool wide = false;
        bool full = false;
        switch (CVPixelBufferGetPixelFormatType(pb)) {
        case kCVPixelFormatType_420YpCbCr8BiPlanarVideoRange:
        case kCVPixelFormatType_422YpCbCr8BiPlanarVideoRange:
        case kCVPixelFormatType_444YpCbCr8BiPlanarVideoRange:
            break;
        case kCVPixelFormatType_420YpCbCr8BiPlanarFullRange:
        case kCVPixelFormatType_422YpCbCr8BiPlanarFullRange:
        case kCVPixelFormatType_444YpCbCr8BiPlanarFullRange:
            full = true;
            break;
        case kCVPixelFormatType_420YpCbCr10BiPlanarVideoRange:
        case kCVPixelFormatType_422YpCbCr10BiPlanarVideoRange:
        case kCVPixelFormatType_444YpCbCr10BiPlanarVideoRange:
            wide = true;
            break;
        case kCVPixelFormatType_420YpCbCr10BiPlanarFullRange:
        case kCVPixelFormatType_422YpCbCr10BiPlanarFullRange:
        case kCVPixelFormatType_444YpCbCr10BiPlanarFullRange:
            wide = true;
            full = true;
            break;
        default:
            return false;
        }

        for (size_t i = 0; i < 2; i++) {
            const MTLPixelFormat format = i == 0
                ? (wide ? MTLPixelFormatR16Unorm : MTLPixelFormatR8Unorm)
                : (wide ? MTLPixelFormatRG16Unorm : MTLPixelFormatRG8Unorm);
            CVMetalTextureRef tex = nullptr;
            const CVReturn ret = CVMetalTextureCacheCreateTextureFromImage(kCFAllocatorDefault,
                                                                           texture_cache,
                                                                           pb,
                                                                           nullptr,
                                                                           format,
                                                                           CVPixelBufferGetWidthOfPlane(pb, i),
                                                                           CVPixelBufferGetHeightOfPlane(pb, i),
                                                                           i,
                                                                           &tex);
            if (ret != kCVReturnSuccess || !tex) {
                if (tex)
                    CFRelease(tex);
                out.reset();
                return false;
            }
            out.cv[i] = tex;
            out.planes[i] = CVMetalTextureGetTexture(tex);
            if (!out.planes[i]) {
                out.reset();
                return false;
            }
        }
        out.planes[2] = nil;
        out.triplanar = false;
        out.full_range = full || frame->color_range == AVCOL_RANGE_JPEG;
        out.depth = wide ? 10 : 8;
        // 10 bits in the MSBs of 16
        out.bit_scale = wide ? static_cast<float>(65535.0 / 65472.0) : 1.0f;
        out.frame = frame;
        return true;
    }

    bool uploadSoftware(AVFrame *frame, MappedFrame &out)
    {
        const AVPixelFormat format = static_cast<AVPixelFormat>(frame->format);
        const AVPixFmtDescriptor *desc = av_pix_fmt_desc_get(format);
        if (!desc || desc->nb_components < 3 ||
            (desc->flags & (AV_PIX_FMT_FLAG_RGB | AV_PIX_FMT_FLAG_BE | AV_PIX_FMT_FLAG_HWACCEL |
                            AV_PIX_FMT_FLAG_PAL | AV_PIX_FMT_FLAG_BITSTREAM)))
            return false;
        const int plane_count = av_pix_fmt_count_planes(format);
        const int depth = desc->comp[0].depth;
        if ((plane_count != 2 && plane_count != 3) || depth < 8 || depth > 16)
            return false;
        // NV21-style V-before-U is not handled
        if (plane_count == 2 && desc->comp[1].offset > desc->comp[2].offset)
            return false;
        if (frame->width <= 0 || frame->height <= 0)
            return false;

        const bool wide = depth > 8;
        const bool triplanar = plane_count == 3;
        const NSUInteger widths[3] = {
            static_cast<NSUInteger>(frame->width),
            static_cast<NSUInteger>(AV_CEIL_RSHIFT(frame->width, desc->log2_chroma_w)),
            static_cast<NSUInteger>(AV_CEIL_RSHIFT(frame->width, desc->log2_chroma_w)),
        };
        const NSUInteger heights[3] = {
            static_cast<NSUInteger>(frame->height),
            static_cast<NSUInteger>(AV_CEIL_RSHIFT(frame->height, desc->log2_chroma_h)),
            static_cast<NSUInteger>(AV_CEIL_RSHIFT(frame->height, desc->log2_chroma_h)),
        };
        const MTLPixelFormat single = wide ? MTLPixelFormatR16Unorm : MTLPixelFormatR8Unorm;
        const MTLPixelFormat dual = wide ? MTLPixelFormatRG16Unorm : MTLPixelFormatRG8Unorm;
        const NSUInteger bytes_per_sample = wide ? 2 : 1;

        // With at most two command buffers in flight and the slot wait done first, the set after
        // the one on screen is never read by the GPU
        const int index = (upload_index + 1) % kUploadRingSize;
        UploadSet &set = upload_ring[index];
        for (int i = 0; i < plane_count; i++) {
            const MTLPixelFormat pf = (i == 0 || triplanar) ? single : dual;
            const NSUInteger components = (i == 0 || triplanar) ? 1 : 2;
            if (frame->linesize[i] <= 0 || !frame->data[i] ||
                static_cast<NSUInteger>(frame->linesize[i]) < widths[i] * components * bytes_per_sample)
                return false;
            id<MTLTexture> tex = set.tex[i];
            if (!tex || tex.pixelFormat != pf || tex.width != widths[i] || tex.height != heights[i]) {
                tex = makePlaneTexture(pf, widths[i], heights[i]);
                if (!tex)
                    return false;
                tex.label = @"chiaki video plane";
                set.tex[i] = tex;
            }
            [tex replaceRegion:MTLRegionMake2D(0, 0, widths[i], heights[i])
                   mipmapLevel:0
                     withBytes:frame->data[i]
                   bytesPerRow:static_cast<NSUInteger>(frame->linesize[i])];
            out.planes[i] = tex;
        }
        if (!triplanar)
            out.planes[2] = nil;
        upload_index = index;

        out.triplanar = triplanar;
        out.full_range = frame->color_range == AVCOL_RANGE_JPEG ||
            format == AV_PIX_FMT_YUVJ420P || format == AV_PIX_FMT_YUVJ422P || format == AV_PIX_FMT_YUVJ444P;
        out.depth = depth;
        if (wide) {
            const double max_value = static_cast<double>(((1u << depth) - 1u) << desc->comp[0].shift);
            out.bit_scale = static_cast<float>(65535.0 / max_value);
        } else {
            out.bit_scale = 1.0f;
        }
        out.frame = frame;
        return true;
    }

    // Takes ownership of frame either way
    bool mapFrame(AVFrame *frame, const std::function<void()> &on_zero_copy_failure)
    {
        MappedFrame mapped;
        bool ok = false;
        if (frame->format == AV_PIX_FMT_VIDEOTOOLBOX) {
            ok = mapVideoToolbox(frame, mapped);
            if (!ok) {
                if (!logged_zero_copy_failure) {
                    logged_zero_copy_failure = true;
                    CVPixelBufferRef pb = reinterpret_cast<CVPixelBufferRef>(frame->data[3]);
                    const OSType type = pb ? CVPixelBufferGetPixelFormatType(pb) : 0;
                    qCWarning(chiakiGui).nospace()
                        << "Metal renderer can't sample this VideoToolbox frame directly (pixel format 0x"
                        << Qt::hex << type << Qt::dec << ", IOSurface " << (pb && CVPixelBufferGetIOSurface(pb) ? "yes" : "no")
                        << "); switching to copied frames";
                }
                if (on_zero_copy_failure)
                    on_zero_copy_failure();
            }
        } else {
            ok = uploadSoftware(frame, mapped);
            if (!ok && !logged_unsupported_format) {
                logged_unsupported_format = true;
                const char *name = av_get_pix_fmt_name(static_cast<AVPixelFormat>(frame->format));
                qCWarning(chiakiGui) << "Metal renderer doesn't support frame format" << (name ? name : "unknown");
            }
        }
        if (!ok) {
            av_frame_free(&frame);
            return false;
        }
        current.takeFrom(mapped);
        if (texture_cache)
            CVMetalTextureCacheFlush(texture_cache, 0);
        updateHdrMetadata(current.frame);
        return true;
    }

    void updateHdrMetadata(const AVFrame *frame)
    {
        if (layerModeForFrame(frame) != LayerMode::PQ)
            return;
        if (const AVFrameSideData *sd = av_frame_get_side_data(frame, AV_FRAME_DATA_MASTERING_DISPLAY_METADATA)) {
            const auto *md = reinterpret_cast<const AVMasteringDisplayMetadata *>(sd->data);
            if (md->has_primaries && md->has_luminance) {
                uint8_t info[24] = {};
                // SEI order is G, B, R; FFmpeg stores R, G, B
                const int order[3] = { 1, 2, 0 };
                for (int c = 0; c < 3; c++) {
                    putBE16(info + c * 4, rationalToUnits(md->display_primaries[order[c]][0], 0.00002));
                    putBE16(info + c * 4 + 2, rationalToUnits(md->display_primaries[order[c]][1], 0.00002));
                }
                putBE16(info + 12, rationalToUnits(md->white_point[0], 0.00002));
                putBE16(info + 14, rationalToUnits(md->white_point[1], 0.00002));
                putBE32(info + 16, rationalToUnits(md->max_luminance, 0.0001));
                putBE32(info + 20, rationalToUnits(md->min_luminance, 0.0001));
                if (!hdr_display_info_valid || std::memcmp(info, hdr_display_info, sizeof(info)) != 0) {
                    std::memcpy(hdr_display_info, info, sizeof(info));
                    hdr_display_info_valid = true;
                    hdr_metadata_dirty = true;
                    const double max_nits = av_q2d(md->max_luminance);
                    if (max_nits > 0.0)
                        hdr_max_nits = static_cast<float>(max_nits);
                }
            }
        }
        if (const AVFrameSideData *sd = av_frame_get_side_data(frame, AV_FRAME_DATA_CONTENT_LIGHT_LEVEL)) {
            const auto *cll = reinterpret_cast<const AVContentLightMetadata *>(sd->data);
            uint8_t info[4] = {};
            putBE16(info, cll->MaxCLL);
            putBE16(info + 2, cll->MaxFALL);
            if (!hdr_content_info_valid || std::memcmp(info, hdr_content_info, sizeof(info)) != 0) {
                std::memcpy(hdr_content_info, info, sizeof(info));
                hdr_content_info_valid = true;
                hdr_metadata_dirty = true;
            }
        }
    }

    // Render thread; changes are wrapped in a transaction so they don't animate
    void applyLayerMode(LayerMode mode)
    {
        if (mode == layer_mode && !(mode == LayerMode::PQ && hdr_metadata_dirty))
            return;
        if (mode != LayerMode::PQ && layer_mode == LayerMode::PQ) {
            hdr_display_info_valid = false;
            hdr_content_info_valid = false;
            hdr_max_nits = 1000.0f;
        }
        hdr_metadata_dirty = false;

        [CATransaction begin];
        [CATransaction setDisableActions:YES];
        CGColorSpaceRef cs = nullptr;
        switch (mode) {
        case LayerMode::PQ:
            layer.pixelFormat = MTLPixelFormatBGR10A2Unorm;
            cs = CGColorSpaceCreateWithName(kCGColorSpaceITUR_2100_PQ);
            break;
        case LayerMode::HLG:
            layer.pixelFormat = MTLPixelFormatBGR10A2Unorm;
            cs = CGColorSpaceCreateWithName(kCGColorSpaceITUR_2100_HLG);
            break;
        case LayerMode::SDR:
            layer.pixelFormat = MTLPixelFormatBGRA8Unorm;
            cs = CGColorSpaceCreateWithName(kCGColorSpaceSRGB);
            break;
        }
        layer.colorspace = cs;
        if (cs)
            CGColorSpaceRelease(cs);
        const bool hdr = mode != LayerMode::SDR;
        layer.wantsExtendedDynamicRangeContent = hdr;
        if (@available(macOS 26.0, *)) {
            layer.preferredDynamicRange = hdr ? CADynamicRangeHigh : CADynamicRangeStandard;
            layer.contentsHeadroom = hdr ? std::max(1.0, static_cast<double>(hdr_max_nits) / kSdrWhiteNits) : 0.0;
        }
        if (mode == LayerMode::PQ) {
            NSData *display = hdr_display_info_valid ? [NSData dataWithBytes:hdr_display_info length:sizeof(hdr_display_info)] : nil;
            NSData *content = hdr_content_info_valid ? [NSData dataWithBytes:hdr_content_info length:sizeof(hdr_content_info)] : nil;
            // Normalized (unorm) drawables: the scale is 10000 nits per 1.0
            layer.EDRMetadata = [CAEDRMetadata HDR10MetadataWithDisplayInfo:display contentInfo:content opticalOutputScale:10000.0f];
        } else if (mode == LayerMode::HLG) {
            layer.EDRMetadata = [CAEDRMetadata HLGMetadata];
        } else {
            layer.EDRMetadata = nil;
        }
        [CATransaction commit];

        if (mode != layer_mode) {
            qCInfo(chiakiGui) << "Metal renderer output:"
                              << (mode == LayerMode::PQ ? "HDR10 (BT.2100 PQ, EDR)" : mode == LayerMode::HLG ? "HLG (BT.2100 HLG, EDR)" : "SDR (sRGB)");
        }
        layer_mode = mode;
    }

    void noteBackpressure(const char *what)
    {
        backpressure_events++;
        const double now = CACurrentMediaTime();
        if (now - last_backpressure_log_s >= 2.0) {
            qCDebug(chiakiGui).nospace() << "[metal] " << what << ", render retried (" << backpressure_events << " times since last log)";
            last_backpressure_log_s = now;
            backpressure_events = 0;
        }
    }

    void logStats(QSize drawable)
    {
        const double now = CACurrentMediaTime();
        if (last_stats_log_s == 0.0) {
            last_stats_log_s = now;
            return;
        }
        if (now - last_stats_log_s < 2.0)
            return;
        last_stats_log_s = now;
        const uint32_t frames = stats->frames.exchange(0);
        const uint64_t sum = stats->gpu_us_sum.exchange(0);
        const uint64_t max = stats->gpu_us_max.exchange(0);
        const uint32_t errors = stats->errors.load();
        if (errors != last_logged_errors) {
            qCWarning(chiakiGui) << "Metal command buffers failed:" << (errors - last_logged_errors);
            last_logged_errors = errors;
        }
        if (frames > 0 && chiakiGui().isDebugEnabled()) {
            qCDebug(chiakiGui).nospace()
                << "[metal] gpu_us avg=" << (sum / frames) << " max=" << max
                << " presents=" << frames << " drawable=" << drawable.width() << "x" << drawable.height()
                << " overlay=" << stats->overlay_has_content.load();
        }
    }

    void endRenderSignpost(const FrameTiming &timing)
    {
        if (timing.signpost != OS_SIGNPOST_ID_NULL)
            os_signpost_interval_end(signpost_log, timing.signpost, "render");
    }

    void logLatency()
    {
        const double now = CACurrentMediaTime();
        if (last_latency_log_s == 0.0)
            last_latency_log_s = now;
        if (now - last_latency_log_s < CHIAKI_LATENCY_STATS_INTERVAL_US / 1e6)
            return;
        last_latency_log_s = now;
        ChiakiLatencySummary s[4];
        {
            std::lock_guard<std::mutex> locker(stats->latency_mutex);
            s[0] = chiaki_latency_stats_summarize(&stats->handoff);
            s[1] = chiaki_latency_stats_summarize(&stats->render);
            s[2] = chiaki_latency_stats_summarize(&stats->display);
            s[3] = chiaki_latency_stats_summarize(&stats->total);
        }
        const uint32_t dropped = stats->dropped_before_display.exchange(0);
        const uint32_t not_presented = stats->not_presented.exchange(0);
        if (s[2].frames == 0 && dropped == 0 && not_presented == 0)
            return;
        qCInfo(chiakiGui).noquote() << QString::asprintf(
            "[latency] handoff_ms p50=%.2f p99=%.2f max=%.2f | render_ms p50=%.2f p99=%.2f max=%.2f"
            " | display_ms p50=%.2f p99=%.2f max=%.2f | total_ms p50=%.2f p99=%.2f max=%.2f"
            " frames=%u dropped_before_display=%u not_presented=%u",
            s[0].p50_ms, s[0].p99_ms, s[0].max_ms, s[1].p50_ms, s[1].p99_ms, s[1].max_ms,
            s[2].p50_ms, s[2].p99_ms, s[2].max_ms, s[3].p50_ms, s[3].p99_ms, s[3].max_ms,
            s[2].frames, dropped, not_presented);
    }
};

MetalRenderer::MetalRenderer()
    : d(new Private)
{
}

MetalRenderer::~MetalRenderer()
{
    @autoreleasepool {
        {
            std::lock_guard<std::mutex> locker(d->frame_mutex);
            av_frame_free(&d->pending);
        }
        d->current.reset();
        if (d->texture_cache) {
            CFRelease(d->texture_cache);
            d->texture_cache = nullptr;
        }
        delete d;
        d = nullptr;
    }
}

MetalRenderer *MetalRenderer::create(QString *error)
{
    @autoreleasepool {
        auto fail = [error](const QString &msg) -> MetalRenderer * {
            if (error)
                *error = msg;
            return nullptr;
        };

        std::unique_ptr<MetalRenderer> renderer(new MetalRenderer);
        Private *d = renderer->d;

        d->device = MTLCreateSystemDefaultDevice();
        if (!d->device)
            return fail(QStringLiteral("No Metal device"));
        d->queue = [d->device newCommandQueue];
        if (!d->queue)
            return fail(QStringLiteral("Failed to create Metal command queue"));
        d->queue.label = @"chiaki";

        NSError *ns_error = nil;
        d->library = [d->device newLibraryWithSource:[NSString stringWithUTF8String:kShaderSource]
                                             options:[MTLCompileOptions new]
                                               error:&ns_error];
        if (!d->library)
            return fail(QStringLiteral("Failed to compile Metal shaders: %1").arg(QString::fromNSString(ns_error.localizedDescription)));
        d->vertex_fn = [d->library newFunctionWithName:@"chiaki_fullscreen_vertex"];
        d->fragment_fn = [d->library newFunctionWithName:@"chiaki_composite_fragment"];
        id<MTLFunction> check_fn = [d->library newFunctionWithName:@"chiaki_overlay_alpha_check"];
        if (!d->vertex_fn || !d->fragment_fn || !check_fn)
            return fail(QStringLiteral("Metal shader functions missing"));

        d->pipelines = [NSMutableDictionary new];
        QString pipeline_error;
        if (!d->pipelineFor(MTLPixelFormatBGRA8Unorm, &pipeline_error) ||
            !d->pipelineFor(MTLPixelFormatBGR10A2Unorm, &pipeline_error))
            return fail(pipeline_error);
        d->overlay_check = [d->device newComputePipelineStateWithFunction:check_fn error:&ns_error];
        if (!d->overlay_check)
            return fail(QStringLiteral("Failed to create Metal compute pipeline: %1").arg(QString::fromNSString(ns_error.localizedDescription)));

        if (CVMetalTextureCacheCreate(kCFAllocatorDefault, nullptr, d->device, nullptr, &d->texture_cache) != kCVReturnSuccess || !d->texture_cache)
            return fail(QStringLiteral("Failed to create CVMetalTextureCache"));

        // The fragment function declares all four textures, so unused slots get 1x1 stand-ins
        d->dummy_plane = d->makePlaneTexture(MTLPixelFormatRG8Unorm, 1, 1);
        d->dummy_rgba = d->makePlaneTexture(MTLPixelFormatRGBA8Unorm, 1, 1);
        if (!d->dummy_plane || !d->dummy_rgba)
            return fail(QStringLiteral("Failed to create Metal textures"));
        const uint32_t zero = 0;
        [d->dummy_plane replaceRegion:MTLRegionMake2D(0, 0, 1, 1) mipmapLevel:0 withBytes:&zero bytesPerRow:2];
        [d->dummy_rgba replaceRegion:MTLRegionMake2D(0, 0, 1, 1) mipmapLevel:0 withBytes:&zero bytesPerRow:4];

        d->retired_overlays = [NSMutableArray new];
        d->in_flight = dispatch_semaphore_create(kMaxCommandBuffersInFlight);

        qCInfo(chiakiGui) << "Metal renderer using" << QString::fromNSString(d->device.name);
        return renderer.release();
    }
}

void MetalRenderer::runInAutoreleasePool(const std::function<void()> &fn)
{
    @autoreleasepool {
        fn();
    }
}

void *MetalRenderer::device() const
{
    return (__bridge void *)d->device;
}

void *MetalRenderer::commandQueue() const
{
    return (__bridge void *)d->queue;
}

bool MetalRenderer::attachToView(void *view_ptr, bool vsync, QString *error)
{
    @autoreleasepool {
        if (d->attached.load(std::memory_order_acquire))
            return true;
        NSView *view = (__bridge NSView *)view_ptr;
        if (!view) {
            if (error)
                *error = QStringLiteral("Window has no native view");
            return false;
        }

        CALayer *root = view.layer;
        CAMetalLayer *metal_layer = nil;
        if ([root isKindOfClass:[CAMetalLayer class]]) {
            metal_layer = static_cast<CAMetalLayer *>(root);
        } else if (root && [root respondsToSelector:NSSelectorFromString(@"contentLayer")]) {
            // From Qt 6.10 the view's layer is a container holding the CAMetalLayer
            id content = [root valueForKey:@"contentLayer"];
            if ([content isKindOfClass:[CAMetalLayer class]])
                metal_layer = content;
        }
        if (!metal_layer) {
            if (!root) {
                view.wantsLayer = YES;
                root = view.layer;
            }
            if (!root) {
                if (error)
                    *error = QStringLiteral("Window view has no layer");
                return false;
            }
            metal_layer = [CAMetalLayer layer];
            metal_layer.frame = root.bounds;
            metal_layer.autoresizingMask = kCALayerWidthSizable | kCALayerHeightSizable;
            [root addSublayer:metal_layer];
            qCInfo(chiakiGui) << "Metal renderer added its own CAMetalLayer to the window";
        }

        [CATransaction begin];
        [CATransaction setDisableActions:YES];
        metal_layer.device = d->device;
        metal_layer.pixelFormat = MTLPixelFormatBGRA8Unorm;
        CGColorSpaceRef cs = CGColorSpaceCreateWithName(kCGColorSpaceSRGB);
        metal_layer.colorspace = cs;
        if (cs)
            CGColorSpaceRelease(cs);
        metal_layer.wantsExtendedDynamicRangeContent = NO;
        metal_layer.framebufferOnly = YES;
        metal_layer.opaque = YES;
        metal_layer.maximumDrawableCount = 2;
        metal_layer.displaySyncEnabled = vsync;
        metal_layer.allowsNextDrawableTimeout = YES;
        metal_layer.presentsWithTransaction = NO;
        if (view.window)
            metal_layer.contentsScale = view.window.backingScaleFactor;
        [CATransaction commit];

        d->layer = metal_layer;
        d->layer_mode = LayerMode::SDR;
        d->attached.store(true, std::memory_order_release);
        qCInfo(chiakiGui) << "Metal renderer attached, vsync" << vsync;
        return true;
    }
}

bool MetalRenderer::attached() const
{
    return d->attached.load(std::memory_order_acquire);
}

void MetalRenderer::setDrawableGeometry(const QSize &pixel_size, qreal scale)
{
    if (!attached() || pixel_size.isEmpty())
        return;
    @autoreleasepool {
        [CATransaction begin];
        [CATransaction setDisableActions:YES];
        if (scale > 0.0)
            d->layer.contentsScale = scale;
        d->layer.drawableSize = CGSizeMake(pixel_size.width(), pixel_size.height());
        [CATransaction commit];
    }
}

void MetalRenderer::setVSync(bool enabled)
{
    if (!attached())
        return;
    d->layer.displaySyncEnabled = enabled;
    qCInfo(chiakiGui) << "Metal renderer vsync" << enabled;
}

bool MetalRenderer::submitFrame(AVFrame *frame, uint64_t first_packet_us, uint64_t decoded_us)
{
    if (!frame)
        return false;
    FrameTiming timing;
    timing.first_packet_us = first_packet_us;
    timing.decoded_us = decoded_us;
    timing.handoff_us = chiaki_time_now_monotonic_us();
    if (os_signpost_enabled(d->signpost_log)) {
        timing.signpost = os_signpost_id_generate(d->signpost_log);
        os_signpost_interval_begin(d->signpost_log, timing.signpost, "render");
    }
    AVFrame *dropped = nullptr;
    FrameTiming dropped_timing;
    {
        std::lock_guard<std::mutex> locker(d->frame_mutex);
        dropped = d->pending;
        dropped_timing = d->pending_timing;
        d->pending = frame;
        d->pending_timing = timing;
    }
    if (!dropped)
        return false;
    d->endRenderSignpost(dropped_timing);
    d->stats->dropped_before_display.fetch_add(1, std::memory_order_relaxed);
    av_frame_free(&dropped);
    return true;
}

void MetalRenderer::clearFrames()
{
    AVFrame *dropped = nullptr;
    FrameTiming dropped_timing;
    {
        std::lock_guard<std::mutex> locker(d->frame_mutex);
        dropped = d->pending;
        dropped_timing = d->pending_timing;
        d->pending = nullptr;
    }
    if (dropped)
        d->endRenderSignpost(dropped_timing);
    av_frame_free(&dropped);
    d->clear_requested.store(true, std::memory_order_release);
}

bool MetalRenderer::hasPendingFrame() const
{
    std::lock_guard<std::mutex> locker(d->frame_mutex);
    return d->pending != nullptr;
}

void *MetalRenderer::ensureOverlayTexture(const QSize &pixel_size)
{
    if (pixel_size.isEmpty())
        return nullptr;
    if (d->overlay && d->overlay_size == pixel_size)
        return (__bridge void *)d->overlay;
    @autoreleasepool {
        MTLTextureDescriptor *td = [MTLTextureDescriptor texture2DDescriptorWithPixelFormat:MTLPixelFormatRGBA8Unorm
                                                                                      width:static_cast<NSUInteger>(pixel_size.width())
                                                                                     height:static_cast<NSUInteger>(pixel_size.height())
                                                                                  mipmapped:NO];
        td.usage = MTLTextureUsageRenderTarget | MTLTextureUsageShaderRead;
        td.storageMode = MTLStorageModePrivate;
        id<MTLTexture> tex = [d->device newTextureWithDescriptor:td];
        if (!tex) {
            qCWarning(chiakiGui) << "Failed to create Metal UI overlay texture" << pixel_size;
            return nullptr;
        }
        tex.label = @"chiaki UI overlay";

        // New private memory holds garbage until Qt Quick first renders into it. The queue runs
        // this before anything that reads the texture; nothing waits for it.
        id<MTLCommandBuffer> cb = [d->queue commandBuffer];
        MTLRenderPassDescriptor *rp = [MTLRenderPassDescriptor renderPassDescriptor];
        rp.colorAttachments[0].texture = tex;
        rp.colorAttachments[0].loadAction = MTLLoadActionClear;
        rp.colorAttachments[0].storeAction = MTLStoreActionStore;
        rp.colorAttachments[0].clearColor = MTLClearColorMake(0.0, 0.0, 0.0, 0.0);
        id<MTLRenderCommandEncoder> enc = [cb renderCommandEncoderWithDescriptor:rp];
        [enc endEncoding];
        [cb commit];

        // Qt Quick may still reference the previous texture until it switches render targets
        if (d->overlay) {
            [d->retired_overlays addObject:d->overlay];
            while (d->retired_overlays.count > 2)
                [d->retired_overlays removeObjectAtIndex:0];
        }
        d->overlay = tex;
        d->overlay_size = pixel_size;
        d->stats->overlay_generation.fetch_add(1);
        d->stats->overlay_has_content.store(0);
        d->overlay_check_pending = false;
        return (__bridge void *)tex;
    }
}

void MetalRenderer::noteOverlayRendered()
{
    // Composite the UI until the check on the GPU says it is empty
    d->stats->overlay_generation.fetch_add(1);
    d->stats->overlay_has_content.store(1);
    d->overlay_check_pending = d->overlay != nil;
}

MetalRenderer::RenderResult MetalRenderer::render(const RenderParams &params)
{
    RenderResult result;
    if (!attached())
        return result;

    @autoreleasepool {
        if (d->clear_requested.exchange(false)) {
            d->current.reset();
            if (d->current_fresh)
                d->endRenderSignpost(d->current_timing);
            d->current_fresh = false;
        }

        if (dispatch_semaphore_wait(d->in_flight, dispatch_time(DISPATCH_TIME_NOW, kInFlightWaitNs)) != 0) {
            d->noteBackpressure("GPU busy (2 command buffers in flight)");
            result.retry = true;
            return result;
        }
        struct SlotGuard {
            dispatch_semaphore_t sem;
            bool armed = true;
            ~SlotGuard()
            {
                if (armed)
                    dispatch_semaphore_signal(sem);
            }
        } slot_guard{d->in_flight};

        AVFrame *incoming = nullptr;
        FrameTiming incoming_timing;
        {
            std::lock_guard<std::mutex> locker(d->frame_mutex);
            std::swap(incoming, d->pending);
            incoming_timing = d->pending_timing;
        }
        if (incoming) {
            if (d->mapFrame(incoming, on_zero_copy_failure)) {
                // A frame mapped by an earlier render that was retried was never shown
                if (d->current_fresh) {
                    d->endRenderSignpost(d->current_timing);
                    d->stats->dropped_before_display.fetch_add(1, std::memory_order_relaxed);
                }
                d->current_timing = incoming_timing;
                d->current_fresh = true;
            } else {
                d->endRenderSignpost(incoming_timing);
            }
        }

        CAMetalLayer *layer = d->layer;
        d->applyLayerMode(layerModeForFrame(d->current.frame));

        const CGSize drawable_size = layer.drawableSize;
        if (drawable_size.width < 1.0 || drawable_size.height < 1.0)
            return result;

        // Bounded by allowsNextDrawableTimeout (1 s)
        id<CAMetalDrawable> drawable = [layer nextDrawable];
        if (!drawable) {
            d->noteBackpressure("nextDrawable timed out");
            result.retry = true;
            return result;
        }
        id<MTLTexture> target = drawable.texture;
        id<MTLRenderPipelineState> pso = d->pipelineFor(target.pixelFormat);
        if (!pso)
            return result;

        const double out_w = static_cast<double>(target.width);
        const double out_h = static_cast<double>(target.height);

        ShaderParams sp = {};
        sp.viewport[0] = static_cast<float>(out_w);
        sp.viewport[1] = static_cast<float>(out_h);
        sp.viewport[2] = static_cast<float>(1.0 / out_w);
        sp.viewport[3] = static_cast<float>(1.0 / out_h);
        sp.sdr_white = kSdrWhiteNits / 10000.0f;
        sp.bit_scale = 1.0f;

        MappedFrame &cur = d->current;
        const bool has_video = cur.frame && cur.planes[0] && cur.planes[1];
        if (has_video) {
            const AVFrame *frame = cur.frame;
            const double luma_w = static_cast<double>(cur.planes[0].width);
            const double luma_h = static_cast<double>(cur.planes[0].height);
            const double chroma_w = static_cast<double>(cur.planes[1].width);
            const double chroma_h = static_cast<double>(cur.planes[1].height);
            const double src_w = frame->width > 0 ? std::min<double>(frame->width, luma_w) : luma_w;
            const double src_h = frame->height > 0 ? std::min<double>(frame->height, luma_h) : luma_h;

            double dst_x = 0.0, dst_y = 0.0, dst_w = out_w, dst_h = out_h;
            switch (params.video_mode) {
            case VideoMode::Stretch:
                break;
            case VideoMode::Zoom:
                if (params.zoom_factor == -1.0f) {
                    const double scale = std::max(out_w / src_w, out_h / src_h);
                    dst_w = src_w * scale;
                    dst_h = src_h * scale;
                } else {
                    // Same as the libplacebo path: native size times 2^zoom, centered
                    const double z = std::pow(2.0, static_cast<double>(params.zoom_factor));
                    dst_w = src_w * z;
                    dst_h = src_h * z;
                }
                dst_x = (out_w - dst_w) * 0.5;
                dst_y = (out_h - dst_h) * 0.5;
                break;
            case VideoMode::Normal:
            default: {
                const double scale = std::min(out_w / src_w, out_h / src_h);
                dst_w = src_w * scale;
                dst_h = src_h * scale;
                dst_x = (out_w - dst_w) * 0.5;
                dst_y = (out_h - dst_h) * 0.5;
                break;
            }
            }

            sp.dst_rect[0] = static_cast<float>(dst_x);
            sp.dst_rect[1] = static_cast<float>(dst_y);
            sp.dst_rect[2] = static_cast<float>(dst_w);
            sp.dst_rect[3] = static_cast<float>(dst_h);
            sp.src_rect[0] = 0.0f;
            sp.src_rect[1] = 0.0f;
            sp.src_rect[2] = static_cast<float>(src_w);
            sp.src_rect[3] = static_cast<float>(src_h);
            sp.luma_size[0] = static_cast<float>(luma_w);
            sp.luma_size[1] = static_cast<float>(luma_h);
            sp.luma_size[2] = static_cast<float>(1.0 / luma_w);
            sp.luma_size[3] = static_cast<float>(1.0 / luma_h);
            sp.chroma_size[0] = static_cast<float>(chroma_w);
            sp.chroma_size[1] = static_cast<float>(chroma_h);
            sp.chroma_size[2] = static_cast<float>(1.0 / chroma_w);
            sp.chroma_size[3] = static_cast<float>(1.0 / chroma_h);

            // Chroma siting: H.264/HEVC default to left (co-sited horizontally, centered vertically)
            const double sx = chroma_w / luma_w;
            const double sy = chroma_h / luma_h;
            double off_x = 0.0, off_y = 0.0;
            switch (frame->chroma_location) {
            case AVCHROMA_LOC_CENTER:
                break;
            case AVCHROMA_LOC_TOPLEFT:
                off_x = 0.5 * (1.0 - sx);
                off_y = 0.5 * (1.0 - sy);
                break;
            case AVCHROMA_LOC_TOP:
                off_y = 0.5 * (1.0 - sy);
                break;
            case AVCHROMA_LOC_BOTTOMLEFT:
                off_x = 0.5 * (1.0 - sx);
                off_y = -0.5 * (1.0 - sy);
                break;
            case AVCHROMA_LOC_BOTTOM:
                off_y = -0.5 * (1.0 - sy);
                break;
            case AVCHROMA_LOC_LEFT:
            default:
                off_x = 0.5 * (1.0 - sx);
                break;
            }
            sp.chroma_map[0] = static_cast<float>(off_x);
            sp.chroma_map[1] = static_cast<float>(off_y);
            sp.chroma_map[2] = static_cast<float>(sx);
            sp.chroma_map[3] = static_cast<float>(sy);

            float m[3][4];
            computeYuvMatrix(frame, cur.full_range, cur.depth, m);
            std::memcpy(sp.m0, m[0], sizeof(sp.m0));
            std::memcpy(sp.m1, m[1], sizeof(sp.m1));
            std::memcpy(sp.m2, m[2], sizeof(sp.m2));
            sp.bit_scale = cur.bit_scale;

            sp.flags |= kFlagVideo;
            if (cur.triplanar)
                sp.flags |= kFlagTriplanar;
            // Catmull-Rom when magnifying; bilinear is fine when shrinking
            if (dst_w > src_w * 1.001 || dst_h > src_h * 1.001)
                sp.flags |= kFlagBicubic;
        }
        if (d->layer_mode == LayerMode::PQ)
            sp.flags |= kFlagPQ;
        const bool overlay = d->overlay && d->stats->overlay_has_content.load() != 0;
        if (overlay)
            sp.flags |= kFlagOverlay;

        id<MTLCommandBuffer> cb = [d->queue commandBuffer];
        cb.label = @"chiaki present";

        id<MTLBuffer> check_buffer = nil;
        uint32_t check_generation = 0;
        if (d->overlay_check_pending && d->overlay) {
            d->overlay_check_pending = false;
            check_buffer = [d->device newBufferWithLength:sizeof(uint32_t) options:MTLResourceStorageModeShared];
            if (check_buffer) {
                std::memset(check_buffer.contents, 0, sizeof(uint32_t));
                check_generation = d->stats->overlay_generation.load();
                id<MTLComputeCommandEncoder> ce = [cb computeCommandEncoder];
                ce.label = @"chiaki UI alpha check";
                [ce setComputePipelineState:d->overlay_check];
                [ce setTexture:d->overlay atIndex:0];
                [ce setBuffer:check_buffer offset:0 atIndex:0];
                const NSUInteger tw = d->overlay_check.threadExecutionWidth;
                const NSUInteger th = std::max<NSUInteger>(1, d->overlay_check.maxTotalThreadsPerThreadgroup / tw);
                [ce dispatchThreads:MTLSizeMake(d->overlay.width, d->overlay.height, 1)
                    threadsPerThreadgroup:MTLSizeMake(tw, th, 1)];
                [ce endEncoding];
            }
        }

        MTLRenderPassDescriptor *rp = [MTLRenderPassDescriptor renderPassDescriptor];
        rp.colorAttachments[0].texture = target;
        // Every pixel is written by the full-screen triangle
        rp.colorAttachments[0].loadAction = MTLLoadActionDontCare;
        rp.colorAttachments[0].storeAction = MTLStoreActionStore;
        id<MTLRenderCommandEncoder> enc = [cb renderCommandEncoderWithDescriptor:rp];
        enc.label = @"chiaki composite";
        [enc setRenderPipelineState:pso];
        [enc setFragmentBytes:&sp length:sizeof(sp) atIndex:0];
        [enc setFragmentTexture:(has_video ? cur.planes[0] : d->dummy_plane) atIndex:0];
        [enc setFragmentTexture:(has_video ? cur.planes[1] : d->dummy_plane) atIndex:1];
        [enc setFragmentTexture:(has_video && cur.planes[2] ? cur.planes[2] : d->dummy_plane) atIndex:2];
        [enc setFragmentTexture:(d->overlay ? d->overlay : d->dummy_rgba) atIndex:3];
        [enc drawPrimitives:MTLPrimitiveTypeTriangle vertexStart:0 vertexCount:3];
        [enc endEncoding];

        // The completion handler owns what the GPU reads, so nothing has to outlive it here
        AVFrame *hold_frame = has_video ? av_frame_clone(cur.frame) : nullptr;
        CVMetalTextureRef hold_cv0 = has_video && cur.cv[0] ? static_cast<CVMetalTextureRef>(const_cast<void *>(CFRetain(cur.cv[0]))) : nullptr;
        CVMetalTextureRef hold_cv1 = has_video && cur.cv[1] ? static_cast<CVMetalTextureRef>(const_cast<void *>(CFRetain(cur.cv[1]))) : nullptr;
        dispatch_semaphore_t sem = d->in_flight;
        std::shared_ptr<GpuStats> stats = d->stats;
        [cb addCompletedHandler:^(id<MTLCommandBuffer> done) {
            if (done.status == MTLCommandBufferStatusError) {
                stats->errors.fetch_add(1);
            } else {
                const double gpu_s = done.GPUEndTime - done.GPUStartTime;
                if (gpu_s > 0.0) {
                    const uint64_t gpu_us = static_cast<uint64_t>(gpu_s * 1e6);
                    stats->gpu_us_sum.fetch_add(gpu_us);
                    uint64_t prev = stats->gpu_us_max.load();
                    while (gpu_us > prev && !stats->gpu_us_max.compare_exchange_weak(prev, gpu_us)) {
                    }
                }
                stats->frames.fetch_add(1);
                if (check_buffer && stats->overlay_generation.load() == check_generation) {
                    const uint32_t found = *static_cast<const uint32_t *>(check_buffer.contents);
                    stats->overlay_has_content.store(found ? 1 : 0);
                }
            }
            AVFrame *frame = hold_frame;
            av_frame_free(&frame);
            if (hold_cv0)
                CFRelease(hold_cv0);
            if (hold_cv1)
                CFRelease(hold_cv1);
            dispatch_semaphore_signal(sem);
        }];

        // Latency of a newly decoded frame, once it is on glass. presentedTime is in
        // CACurrentMediaTime() (mach_absolute_time) seconds, which stops during sleep unlike
        // chiaki_time_now_monotonic_us(), so it is converted as "now minus how long ago".
        if (has_video && d->current_fresh) {
            d->current_fresh = false;
            const FrameTiming timing = d->current_timing;
            const uint64_t commit_us = chiaki_time_now_monotonic_us();
            [drawable addPresentedHandler:^(id<MTLDrawable> shown) {
                const CFTimeInterval presented_s = shown.presentedTime;
                if (presented_s <= 0.0) {
                    stats->not_presented.fetch_add(1, std::memory_order_relaxed);
                    return;
                }
                const uint64_t now_us = chiaki_time_now_monotonic_us();
                const double ago_us = std::max(0.0, (CACurrentMediaTime() - presented_s) * 1e6);
                const uint64_t glass_us = now_us - std::min<uint64_t>(now_us, static_cast<uint64_t>(ago_us));
                std::lock_guard<std::mutex> locker(stats->latency_mutex);
                if (timing.decoded_us && timing.handoff_us >= timing.decoded_us)
                    chiaki_latency_stats_add(&stats->handoff, timing.handoff_us - timing.decoded_us);
                if (commit_us >= timing.handoff_us)
                    chiaki_latency_stats_add(&stats->render, commit_us - timing.handoff_us);
                chiaki_latency_stats_add(&stats->display, glass_us >= commit_us ? glass_us - commit_us : 0);
                if (timing.first_packet_us && glass_us >= timing.first_packet_us)
                    chiaki_latency_stats_add(&stats->total, glass_us - timing.first_packet_us);
            }];
            d->endRenderSignpost(timing);
        }
        [cb presentDrawable:drawable];
        slot_guard.armed = false;
        [cb commit];

        result.presented = true;
        result.video_presented = has_video;
        d->logStats(QSize(static_cast<int>(out_w), static_cast<int>(out_h)));
        d->logLatency();
        return result;
    }
}
