#pragma once

// Native Metal video renderer for macOS (see metalrenderer.mm). The interface is plain C++ so
// qmlmainwindow.cpp can use it; Objective-C types cross it as void pointers.

#include <QSize>
#include <QString>

#include <cstdint>
#include <functional>

struct AVFrame;

class MetalRenderer
{
public:
    enum class VideoMode {
        Normal,
        Stretch,
        Zoom
    };

    // How the video is scaled up to the drawable
    enum class Upscaler {
        // Catmull-Rom luma, bilinear chroma, in the single composite pass
        Standard,
        // YUV -> RGB at source size, then MetalFX spatial upscaling (macOS 13+, Apple GPUs)
        MetalFX,
        // Reserved for VideoToolbox super resolution; drawn like Standard until it is wired up
        AppleSR
    };

    struct RenderParams {
        VideoMode video_mode = VideoMode::Normal;
        // log2 zoom like the libplacebo path; -1 fills the drawable
        float zoom_factor = 0.0f;
    };

    struct RenderResult {
        // A drawable was presented
        bool presented = false;
        // ...and it contained a decoded video frame
        bool video_presented = false;
        // Nothing was presented because of back-pressure (no drawable, GPU busy); try again soon
        bool retry = false;
    };

    // Runs fn inside an Objective-C autorelease pool. The render thread is a plain QThread with no
    // pool of its own, so Metal objects Qt Quick autoreleases there are drained per call.
    static void runInAutoreleasePool(const std::function<void()> &fn);

    // Creates the device, queue, pipelines and texture cache. Never waits on the GPU.
    // Returns nullptr and fills error if Metal can't be used.
    static MetalRenderer *create(QString *error);
    // Doesn't wait for the GPU: in-flight command buffers own everything they still use
    ~MetalRenderer();

    MetalRenderer(const MetalRenderer &) = delete;
    MetalRenderer &operator=(const MetalRenderer &) = delete;

    // id<MTLDevice> / id<MTLCommandQueue>, shared with Qt Quick
    void *device() const;
    void *commandQueue() const;

    // GUI thread. view is the window's NSView (QWindow::winId()).
    bool attachToView(void *view, bool vsync, QString *error);
    bool attached() const;
    void setDrawableGeometry(const QSize &pixel_size, qreal scale);
    void setVSync(bool enabled);
    // Any thread; takes effect with the next render. MetalFX falls back to Standard (logged once)
    // when the GPU or OS doesn't support it.
    void setUpscaler(Upscaler upscaler);

    // Any thread. Takes ownership of frame. The newest frame wins: returns true when an older
    // frame that was never rendered got dropped for it.
    // first_packet_us / decoded_us (chiaki_time_now_monotonic_us(), 0 if unknown) feed the
    // [latency] log: when the frame's first network packet arrived and when it was decoded.
    bool submitFrame(AVFrame *frame, uint64_t first_packet_us = 0, uint64_t decoded_us = 0);
    // Any thread. Drops the pending and the displayed frame; the next render shows black.
    void clearFrames();
    bool hasPendingFrame() const;

    // Called on the render thread when a VideoToolbox frame can't be used without a copy
    std::function<void()> on_zero_copy_failure;

    // Render thread. Returns the id<MTLTexture> (RGBA8Unorm) Qt Quick renders the UI into,
    // recreating it for a new size; nullptr on failure.
    void *ensureOverlayTexture(const QSize &pixel_size);
    // Render thread. The UI was rendered into the overlay texture again.
    void noteOverlayRendered();
    // Render thread. Draws the newest frame (or the last one again) plus the UI and presents.
    RenderResult render(const RenderParams &params);

private:
    MetalRenderer();
    struct Private;
    Private *d;
};
