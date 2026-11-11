// SPDX-License-Identifier: LicenseRef-AGPL-3.0-only-OpenSSL

#include <chiaki/vtdecoder.h>

#include <VideoToolbox/VideoToolbox.h>
#include <CoreMedia/CoreMedia.h>
#include <CoreVideo/CoreVideo.h>

#include <libavutil/buffer.h>
#include <libavutil/hwcontext.h>
#include <libavutil/hwcontext_videotoolbox.h>
#include <libavutil/pixdesc.h>
#include <libavutil/pixfmt.h>

#include <pthread.h>
#include <stdlib.h>
#include <string.h>

#define VT_NAL_LENGTH_SIZE 4
#define VT_MAX_NALS 256

typedef struct vt_nal_t
{
	const uint8_t *data;
	size_t size;
} VtNal;

typedef struct vt_param_set_t
{
	uint8_t *data;
	size_t size;
} VtParamSet;

struct chiaki_vt_decoder_t
{
	ChiakiLog **log;
	bool hevc;
	bool hdr;
	bool async;
	ChiakiVtDecoderOutput output_cb;
	void *output_cb_user;

	// latest parameter sets, without start code
	VtParamSet vps;
	VtParamSet sps;
	VtParamSet pps;
	bool params_changed;

	CMVideoFormatDescriptionRef format;
	VTDecompressionSessionRef session;
	OSType pixel_format;
	bool using_hardware;
	AVBufferRef *hw_device_ctx;

	// accessed from the output callback (VideoToolbox thread in async mode)
	pthread_mutex_t out_mutex;
	AVBufferRef *hw_frames_ctx;
	enum AVColorPrimaries color_primaries;
	enum AVColorTransferCharacteristic color_trc;
	enum AVColorSpace colorspace;
	enum AVColorRange color_range;
	int32_t display_width;
	int32_t display_height;
	bool active_logged;
};

static ChiakiLog *vt_log(ChiakiVtDecoder *vt)
{
	return vt->log ? *vt->log : NULL;
}

static uint8_t vt_nal_type(ChiakiVtDecoder *vt, const VtNal *nal)
{
	if(vt->hevc)
		return (nal->data[0] >> 1) & 0x3f;
	return nal->data[0] & 0x1f;
}

/**
 * Split an Annex B buffer into NAL units (without start codes).
 */
static size_t vt_split_nals(const uint8_t *buf, size_t size, VtNal *nals, size_t nals_max)
{
	size_t count = 0;
	size_t i = 0;
	size_t nal_start = SIZE_MAX;
	while(i + 3 <= size)
	{
		if(buf[i] == 0 && buf[i + 1] == 0 && buf[i + 2] == 1)
		{
			if(nal_start != SIZE_MAX && count < nals_max)
			{
				size_t end = i;
				while(end > nal_start && buf[end - 1] == 0) // trailing_zero_8bits / 4 byte start code
					end--;
				if(end > nal_start)
					nals[count++] = (VtNal){ buf + nal_start, end - nal_start };
			}
			i += 3;
			nal_start = i;
			continue;
		}
		i++;
	}
	if(nal_start != SIZE_MAX && nal_start < size && count < nals_max)
	{
		size_t end = size;
		while(end > nal_start && buf[end - 1] == 0)
			end--;
		if(end > nal_start)
			nals[count++] = (VtNal){ buf + nal_start, end - nal_start };
	}
	return count;
}

static bool vt_param_set_update(VtParamSet *ps, const VtNal *nal)
{
	if(ps->data && ps->size == nal->size && !memcmp(ps->data, nal->data, nal->size))
		return false;
	uint8_t *data = malloc(nal->size);
	if(!data)
		return false;
	memcpy(data, nal->data, nal->size);
	free(ps->data);
	ps->data = data;
	ps->size = nal->size;
	return true;
}

static void vt_param_set_free(VtParamSet *ps)
{
	free(ps->data);
	ps->data = NULL;
	ps->size = 0;
}

static void vt_pixbuf_release(void *opaque, uint8_t *data)
{
	(void)opaque;
	CVPixelBufferRelease((CVPixelBufferRef)data);
}

static enum AVColorPrimaries vt_map_primaries(CFTypeRef v)
{
	if(!v)
		return AVCOL_PRI_UNSPECIFIED;
	if(CFEqual(v, kCVImageBufferColorPrimaries_ITU_R_709_2))
		return AVCOL_PRI_BT709;
	if(CFEqual(v, kCVImageBufferColorPrimaries_ITU_R_2020))
		return AVCOL_PRI_BT2020;
	if(CFEqual(v, kCVImageBufferColorPrimaries_SMPTE_C))
		return AVCOL_PRI_SMPTE170M;
	if(CFEqual(v, kCVImageBufferColorPrimaries_EBU_3213))
		return AVCOL_PRI_BT470BG;
	if(CFEqual(v, kCVImageBufferColorPrimaries_P3_D65))
		return AVCOL_PRI_SMPTE432;
	return AVCOL_PRI_UNSPECIFIED;
}

static enum AVColorTransferCharacteristic vt_map_transfer(CFTypeRef v)
{
	if(!v)
		return AVCOL_TRC_UNSPECIFIED;
	if(CFEqual(v, kCVImageBufferTransferFunction_ITU_R_709_2))
		return AVCOL_TRC_BT709;
	if(CFEqual(v, kCVImageBufferTransferFunction_SMPTE_ST_2084_PQ))
		return AVCOL_TRC_SMPTE2084;
	if(CFEqual(v, kCVImageBufferTransferFunction_ITU_R_2100_HLG))
		return AVCOL_TRC_ARIB_STD_B67;
	if(CFEqual(v, kCVImageBufferTransferFunction_ITU_R_2020))
		return AVCOL_TRC_BT2020_10;
	if(CFEqual(v, kCVImageBufferTransferFunction_SMPTE_240M_1995))
		return AVCOL_TRC_SMPTE240M;
	if(CFEqual(v, kCVImageBufferTransferFunction_Linear))
		return AVCOL_TRC_LINEAR;
	return AVCOL_TRC_UNSPECIFIED;
}

static enum AVColorSpace vt_map_matrix(CFTypeRef v)
{
	if(!v)
		return AVCOL_SPC_UNSPECIFIED;
	if(CFEqual(v, kCVImageBufferYCbCrMatrix_ITU_R_709_2))
		return AVCOL_SPC_BT709;
	if(CFEqual(v, kCVImageBufferYCbCrMatrix_ITU_R_2020))
		return AVCOL_SPC_BT2020_NCL;
	if(CFEqual(v, kCVImageBufferYCbCrMatrix_ITU_R_601_4))
		return AVCOL_SPC_SMPTE170M;
	if(CFEqual(v, kCVImageBufferYCbCrMatrix_SMPTE_240M_1995))
		return AVCOL_SPC_SMPTE240M;
	return AVCOL_SPC_UNSPECIFIED;
}

static enum AVPixelFormat vt_sw_format(OSType pixel_format)
{
	switch(pixel_format)
	{
		case kCVPixelFormatType_420YpCbCr10BiPlanarVideoRange:
		case kCVPixelFormatType_420YpCbCr10BiPlanarFullRange:
			return AV_PIX_FMT_P010;
		default:
			return AV_PIX_FMT_NV12;
	}
}

static const char *vt_profile_name(ChiakiVtDecoder *vt)
{
	if(vt->hevc)
	{
		// sps: nal header (2), vps id / max sub layers / nesting (1), profile space / tier / profile_idc (1)
		uint8_t profile_idc = vt->sps.size > 3 ? (vt->sps.data[3] & 0x1f) : 0;
		switch(profile_idc)
		{
			case 1: return "Main";
			case 2: return "Main 10";
			case 3: return "Main Still Picture";
			case 4: return "Range Extensions";
			default: return "unknown profile";
		}
	}
	uint8_t profile_idc = vt->sps.size > 1 ? vt->sps.data[1] : 0;
	switch(profile_idc)
	{
		case 66: return "Baseline";
		case 77: return "Main";
		case 88: return "Extended";
		case 100: return "High";
		case 110: return "High 10";
		default: return "unknown profile";
	}
}

/**
 * Must be called with out_mutex held.
 */
static bool vt_ensure_frames_ctx(ChiakiVtDecoder *vt, int width, int height, enum AVPixelFormat sw_format)
{
	if(vt->hw_frames_ctx)
	{
		AVHWFramesContext *frames = (AVHWFramesContext *)vt->hw_frames_ctx->data;
		if(frames->width == width && frames->height == height && frames->sw_format == sw_format)
			return true;
		av_buffer_unref(&vt->hw_frames_ctx);
	}
	AVBufferRef *ref = av_hwframe_ctx_alloc(vt->hw_device_ctx);
	if(!ref)
		return false;
	AVHWFramesContext *frames = (AVHWFramesContext *)ref->data;
	frames->format = AV_PIX_FMT_VIDEOTOOLBOX;
	frames->sw_format = sw_format;
	frames->width = width;
	frames->height = height;
	AVVTFramesContext *vt_frames = frames->hwctx;
	vt_frames->color_range = vt->color_range;
	if(av_hwframe_ctx_init(ref) < 0)
	{
		av_buffer_unref(&ref);
		return false;
	}
	vt->hw_frames_ctx = ref;
	return true;
}

static AVFrame *vt_frame_create(ChiakiVtDecoder *vt, CVPixelBufferRef pixbuf, CMTime pts, CMTime duration)
{
	AVFrame *frame = av_frame_alloc();
	if(!frame)
		return NULL;
	CVPixelBufferRetain(pixbuf);
	frame->buf[0] = av_buffer_create((uint8_t *)pixbuf, 1, vt_pixbuf_release, NULL, AV_BUFFER_FLAG_READONLY);
	if(!frame->buf[0])
	{
		CVPixelBufferRelease(pixbuf);
		av_frame_free(&frame);
		return NULL;
	}
	frame->data[3] = (uint8_t *)pixbuf;
	frame->format = AV_PIX_FMT_VIDEOTOOLBOX;

	int width = (int)CVPixelBufferGetWidth(pixbuf);
	int height = (int)CVPixelBufferGetHeight(pixbuf);
	enum AVPixelFormat sw_format = vt_sw_format(CVPixelBufferGetPixelFormatType(pixbuf));

	pthread_mutex_lock(&vt->out_mutex);
	bool frames_ok = vt_ensure_frames_ctx(vt, width, height, sw_format);
	if(frames_ok)
		frame->hw_frames_ctx = av_buffer_ref(vt->hw_frames_ctx);
	frame->color_primaries = vt->color_primaries;
	frame->color_trc = vt->color_trc;
	frame->colorspace = vt->colorspace;
	frame->color_range = vt->color_range;
	// The pixel buffer may be padded to the coded size, crop to the display size.
	frame->width = vt->display_width > 0 && vt->display_width < width ? vt->display_width : width;
	frame->height = vt->display_height > 0 && vt->display_height < height ? vt->display_height : height;
	bool log_active = !vt->active_logged;
	vt->active_logged = true;
	pthread_mutex_unlock(&vt->out_mutex);

	if(!frames_ok || !frame->hw_frames_ctx)
	{
		CHIAKI_LOGE(vt_log(vt), "VideoToolbox native decoder failed to create hw frames context");
		av_frame_free(&frame);
		return NULL;
	}

	if(CMTIME_IS_VALID(pts))
	{
		frame->pts = pts.value;
		frame->best_effort_timestamp = pts.value;
	}
	if(CMTIME_IS_VALID(duration))
		frame->duration = duration.value;

	if(log_active)
	{
		CHIAKI_LOGI(vt_log(vt), "VideoToolbox hardware decode active (native): %s %s %d-bit %dx%d, hardware=%s, IOSurface=%s, %s",
				vt->hevc ? "HEVC" : "H.264", vt_profile_name(vt),
				sw_format == AV_PIX_FMT_P010 ? 10 : 8,
				frame->width, frame->height,
				vt->using_hardware ? "yes" : "NO",
				CVPixelBufferGetIOSurface(pixbuf) ? "yes" : "NO",
				vt->async ? "async" : "sync");
	}
	return frame;
}

static void vt_output_callback(void *opaque, void *source_frame_ref, OSStatus status, VTDecodeInfoFlags info_flags,
		CVImageBufferRef image_buffer, CMTime pts, CMTime duration)
{
	(void)source_frame_ref;
	ChiakiVtDecoder *vt = opaque;
	if(status != noErr || !image_buffer)
	{
		CHIAKI_LOGE(vt_log(vt), "VideoToolbox native decoder failed to decode frame: status %d, flags %#x",
				(int)status, (unsigned int)info_flags);
		vt->output_cb(NULL, vt->output_cb_user);
		return;
	}
	AVFrame *frame = vt_frame_create(vt, (CVPixelBufferRef)image_buffer, pts, duration);
	vt->output_cb(frame, vt->output_cb_user);
}

static void vt_session_destroy(ChiakiVtDecoder *vt)
{
	if(!vt->session)
		return;
	if(vt->async)
		VTDecompressionSessionWaitForAsynchronousFrames(vt->session);
	VTDecompressionSessionInvalidate(vt->session);
	CFRelease(vt->session);
	vt->session = NULL;
}

static void vt_dict_set_int(CFMutableDictionaryRef dict, CFStringRef key, int32_t value)
{
	CFNumberRef num = CFNumberCreate(kCFAllocatorDefault, kCFNumberSInt32Type, &value);
	CFDictionarySetValue(dict, key, num);
	CFRelease(num);
}

static ChiakiVtDecodeResult vt_session_create(ChiakiVtDecoder *vt)
{
	CFMutableDictionaryRef spec = CFDictionaryCreateMutable(kCFAllocatorDefault, 0,
			&kCFTypeDictionaryKeyCallBacks, &kCFTypeDictionaryValueCallBacks);
	CFDictionarySetValue(spec, kVTVideoDecoderSpecification_RequireHardwareAcceleratedVideoDecoder, kCFBooleanTrue);

	CFMutableDictionaryRef attrs = CFDictionaryCreateMutable(kCFAllocatorDefault, 0,
			&kCFTypeDictionaryKeyCallBacks, &kCFTypeDictionaryValueCallBacks);
	vt_dict_set_int(attrs, kCVPixelBufferPixelFormatTypeKey, (int32_t)vt->pixel_format);
	CFDictionaryRef iosurface_props = CFDictionaryCreate(kCFAllocatorDefault, NULL, NULL, 0,
			&kCFTypeDictionaryKeyCallBacks, &kCFTypeDictionaryValueCallBacks);
	CFDictionarySetValue(attrs, kCVPixelBufferIOSurfacePropertiesKey, iosurface_props);
	CFRelease(iosurface_props);
	CFDictionarySetValue(attrs, kCVPixelBufferMetalCompatibilityKey, kCFBooleanTrue);

	VTDecompressionOutputCallbackRecord callback = { vt_output_callback, vt };
	OSStatus status = VTDecompressionSessionCreate(kCFAllocatorDefault, vt->format, spec, attrs, &callback, &vt->session);
	CFRelease(spec);
	CFRelease(attrs);
	if(status != noErr || !vt->session)
	{
		vt->session = NULL;
		CHIAKI_LOGE(vt_log(vt), "VideoToolbox native decoder failed to create hardware decompression session: status %d", (int)status);
		return CHIAKI_VT_DECODE_FATAL;
	}

	status = VTSessionSetProperty(vt->session, kVTDecompressionPropertyKey_RealTime, kCFBooleanTrue);
	if(status != noErr)
		CHIAKI_LOGW(vt_log(vt), "VideoToolbox native decoder failed to set RealTime: status %d", (int)status);

	vt->using_hardware = false;
	CFBooleanRef using_hw = NULL;
	status = VTSessionCopyProperty(vt->session, kVTDecompressionPropertyKey_UsingHardwareAcceleratedVideoDecoder,
			kCFAllocatorDefault, &using_hw);
	if(status == noErr && using_hw)
	{
		vt->using_hardware = CFBooleanGetValue(using_hw);
		CFRelease(using_hw);
	}
	if(!vt->using_hardware)
	{
		CHIAKI_LOGE(vt_log(vt), "VideoToolbox native decoder session is not using the hardware decoder");
		vt_session_destroy(vt);
		return CHIAKI_VT_DECODE_FATAL;
	}
	return CHIAKI_VT_DECODE_OK;
}

static ChiakiVtDecodeResult vt_apply_params(ChiakiVtDecoder *vt)
{
	vt->params_changed = false;
	CMVideoFormatDescriptionRef format = NULL;
	OSStatus status;
	if(vt->hevc)
	{
		const uint8_t *sets[3] = { vt->vps.data, vt->sps.data, vt->pps.data };
		size_t sizes[3] = { vt->vps.size, vt->sps.size, vt->pps.size };
		status = CMVideoFormatDescriptionCreateFromHEVCParameterSets(kCFAllocatorDefault, 3, sets, sizes,
				VT_NAL_LENGTH_SIZE, NULL, &format);
	}
	else
	{
		const uint8_t *sets[2] = { vt->sps.data, vt->pps.data };
		size_t sizes[2] = { vt->sps.size, vt->pps.size };
		status = CMVideoFormatDescriptionCreateFromH264ParameterSets(kCFAllocatorDefault, 2, sets, sizes,
				VT_NAL_LENGTH_SIZE, &format);
	}
	if(status != noErr || !format)
	{
		CHIAKI_LOGE(vt_log(vt), "VideoToolbox native decoder failed to create format description from parameter sets: status %d", (int)status);
		return CHIAKI_VT_DECODE_FATAL;
	}

	CFTypeRef full_range = CMFormatDescriptionGetExtension(format, kCMFormatDescriptionExtension_FullRangeVideo);
	bool is_full_range = full_range && CFGetTypeID(full_range) == CFBooleanGetTypeID() && CFBooleanGetValue(full_range);
	bool ten_bit = vt->hdr;
	if(vt->hevc && vt->sps.size > 3 && (vt->sps.data[3] & 0x1f) == 2) // Main 10
		ten_bit = true;
	OSType pixel_format = ten_bit
		? (is_full_range ? kCVPixelFormatType_420YpCbCr10BiPlanarFullRange : kCVPixelFormatType_420YpCbCr10BiPlanarVideoRange)
		: (is_full_range ? kCVPixelFormatType_420YpCbCr8BiPlanarFullRange : kCVPixelFormatType_420YpCbCr8BiPlanarVideoRange);

	enum AVColorPrimaries primaries = vt_map_primaries(CMFormatDescriptionGetExtension(format, kCMFormatDescriptionExtension_ColorPrimaries));
	enum AVColorTransferCharacteristic trc = vt_map_transfer(CMFormatDescriptionGetExtension(format, kCMFormatDescriptionExtension_TransferFunction));
	enum AVColorSpace colorspace = vt_map_matrix(CMFormatDescriptionGetExtension(format, kCMFormatDescriptionExtension_YCbCrMatrix));
	if(vt->hdr)
	{
		// HDR streams are always BT.2020 PQ, even if the VUI does not say so
		if(primaries == AVCOL_PRI_UNSPECIFIED)
			primaries = AVCOL_PRI_BT2020;
		if(trc == AVCOL_TRC_UNSPECIFIED)
			trc = AVCOL_TRC_SMPTE2084;
		if(colorspace == AVCOL_SPC_UNSPECIFIED)
			colorspace = AVCOL_SPC_BT2020_NCL;
	}
	CGRect clean = CMVideoFormatDescriptionGetCleanAperture(format, true);
	CMVideoDimensions dims = CMVideoFormatDescriptionGetDimensions(format);
	int32_t display_width = clean.size.width > 0 ? (int32_t)clean.size.width : dims.width;
	int32_t display_height = clean.size.height > 0 ? (int32_t)clean.size.height : dims.height;

	bool keep_session = vt->session && pixel_format == vt->pixel_format
		&& VTDecompressionSessionCanAcceptFormatDescription(vt->session, format);
	if(!keep_session)
		vt_session_destroy(vt);
	else if(vt->async)
		VTDecompressionSessionWaitForAsynchronousFrames(vt->session);

	if(vt->format)
		CFRelease(vt->format);
	vt->format = format;
	vt->pixel_format = pixel_format;

	pthread_mutex_lock(&vt->out_mutex);
	vt->color_primaries = primaries;
	vt->color_trc = trc;
	vt->colorspace = colorspace;
	vt->color_range = is_full_range ? AVCOL_RANGE_JPEG : AVCOL_RANGE_MPEG;
	vt->display_width = display_width;
	vt->display_height = display_height;
	pthread_mutex_unlock(&vt->out_mutex);

	CHIAKI_LOGI(vt_log(vt), "VideoToolbox native decoder configured: %s %s %dx%d, %s, %s range, primaries %s, transfer %s, matrix %s%s",
			vt->hevc ? "HEVC" : "H.264", vt_profile_name(vt), (int)display_width, (int)display_height,
			ten_bit ? "P010" : "NV12", is_full_range ? "full" : "video",
			av_color_primaries_name(primaries) ? av_color_primaries_name(primaries) : "?",
			av_color_transfer_name(trc) ? av_color_transfer_name(trc) : "?",
			av_color_space_name(colorspace) ? av_color_space_name(colorspace) : "?",
			keep_session ? ", session kept" : "");

	if(keep_session)
		return CHIAKI_VT_DECODE_OK;
	return vt_session_create(vt);
}

ChiakiVtDecoder *chiaki_vt_decoder_new(ChiakiLog **log, ChiakiCodec codec, bool async,
		ChiakiVtDecoderOutput output_cb, void *output_cb_user)
{
	ChiakiVtDecoder *vt = calloc(1, sizeof(ChiakiVtDecoder));
	if(!vt)
		return NULL;
	vt->log = log;
	vt->hevc = codec == CHIAKI_CODEC_H265 || codec == CHIAKI_CODEC_H265_HDR;
	vt->hdr = codec == CHIAKI_CODEC_H265_HDR;
	vt->async = async;
	vt->output_cb = output_cb;
	vt->output_cb_user = output_cb_user;
	vt->color_primaries = AVCOL_PRI_UNSPECIFIED;
	vt->color_trc = AVCOL_TRC_UNSPECIFIED;
	vt->colorspace = AVCOL_SPC_UNSPECIFIED;
	vt->color_range = AVCOL_RANGE_MPEG;
	if(pthread_mutex_init(&vt->out_mutex, NULL) != 0)
	{
		free(vt);
		return NULL;
	}

	CMVideoCodecType codec_type = vt->hevc ? kCMVideoCodecType_HEVC : kCMVideoCodecType_H264;
	if(!VTIsHardwareDecodeSupported(codec_type))
	{
		CHIAKI_LOGE(vt_log(vt), "VideoToolbox reports no hardware decoder for %s", vt->hevc ? "HEVC" : "H.264");
		goto error;
	}
	if(av_hwdevice_ctx_create(&vt->hw_device_ctx, AV_HWDEVICE_TYPE_VIDEOTOOLBOX, NULL, NULL, 0) < 0)
	{
		CHIAKI_LOGE(vt_log(vt), "Failed to create VideoToolbox hw device context");
		goto error;
	}
	return vt;
error:
	pthread_mutex_destroy(&vt->out_mutex);
	free(vt);
	return NULL;
}

void chiaki_vt_decoder_free(ChiakiVtDecoder *vt)
{
	if(!vt)
		return;
	vt_session_destroy(vt);
	if(vt->format)
		CFRelease(vt->format);
	vt_param_set_free(&vt->vps);
	vt_param_set_free(&vt->sps);
	vt_param_set_free(&vt->pps);
	av_buffer_unref(&vt->hw_frames_ctx);
	av_buffer_unref(&vt->hw_device_ctx);
	pthread_mutex_destroy(&vt->out_mutex);
	free(vt);
}

bool chiaki_vt_decoder_using_hardware(ChiakiVtDecoder *vt)
{
	return vt->session && vt->using_hardware;
}

ChiakiVtDecodeResult chiaki_vt_decoder_decode(ChiakiVtDecoder *vt, const uint8_t *buf, size_t buf_size,
		int64_t pts, int64_t duration)
{
	// one extra slot detects access units with too many NAL units
	VtNal nals[VT_MAX_NALS + 1];
	size_t nals_count = vt_split_nals(buf, buf_size, nals, VT_MAX_NALS + 1);
	if(nals_count > VT_MAX_NALS)
	{
		CHIAKI_LOGE(vt_log(vt), "VideoToolbox native decoder: access unit has more than %d NAL units", VT_MAX_NALS);
		return CHIAKI_VT_DECODE_ERROR;
	}

	// collect parameter sets, size the sample
	size_t sample_size = 0;
	bool has_vcl = false;
	for(size_t i = 0; i < nals_count; i++)
	{
		uint8_t type = vt_nal_type(vt, &nals[i]);
		bool keep = false;
		if(vt->hevc)
		{
			switch(type)
			{
				case 32: vt->params_changed |= vt_param_set_update(&vt->vps, &nals[i]); break;
				case 33: vt->params_changed |= vt_param_set_update(&vt->sps, &nals[i]); break;
				case 34: vt->params_changed |= vt_param_set_update(&vt->pps, &nals[i]); break;
				case 35: // AUD
				case 38: // filler data
					break;
				default:
					keep = true;
					if(type < 32)
						has_vcl = true;
					break;
			}
		}
		else
		{
			switch(type)
			{
				case 7: vt->params_changed |= vt_param_set_update(&vt->sps, &nals[i]); break;
				case 8: vt->params_changed |= vt_param_set_update(&vt->pps, &nals[i]); break;
				case 9: // AUD
				case 12: // filler data
					break;
				default:
					keep = true;
					if(type >= 1 && type <= 5)
						has_vcl = true;
					break;
			}
		}
		if(keep)
			sample_size += VT_NAL_LENGTH_SIZE + nals[i].size;
		else
			nals[i].size = 0; // marks as not part of the sample
	}

	bool have_params = vt->sps.data && vt->pps.data && (!vt->hevc || vt->vps.data);
	if(vt->params_changed && have_params)
	{
		ChiakiVtDecodeResult r = vt_apply_params(vt);
		if(r != CHIAKI_VT_DECODE_OK)
			return r;
	}

	if(!has_vcl)
		return CHIAKI_VT_DECODE_OK;

	if(!vt->session)
	{
		if(vt->format)
		{
			// session was lost (e.g. kVTInvalidSessionErr), recreate it from the current format
			ChiakiVtDecodeResult r = vt_session_create(vt);
			if(r != CHIAKI_VT_DECODE_OK)
				return r;
		}
		else
		{
			CHIAKI_LOGE(vt_log(vt), "VideoToolbox native decoder received a frame before parameter sets");
			return CHIAKI_VT_DECODE_ERROR;
		}
	}

	CMBlockBufferRef block = NULL;
	OSStatus status = CMBlockBufferCreateWithMemoryBlock(kCFAllocatorDefault, NULL, sample_size, kCFAllocatorDefault,
			NULL, 0, sample_size, kCMBlockBufferAssureMemoryNowFlag, &block);
	if(status != kCMBlockBufferNoErr || !block)
	{
		CHIAKI_LOGE(vt_log(vt), "VideoToolbox native decoder failed to create block buffer: status %d", (int)status);
		return CHIAKI_VT_DECODE_ERROR;
	}
	char *dst = NULL;
	size_t contiguous = 0;
	status = CMBlockBufferGetDataPointer(block, 0, &contiguous, NULL, &dst);
	if(status != kCMBlockBufferNoErr || contiguous < sample_size)
	{
		CHIAKI_LOGE(vt_log(vt), "VideoToolbox native decoder failed to access block buffer: status %d", (int)status);
		CFRelease(block);
		return CHIAKI_VT_DECODE_ERROR;
	}
	// Annex B -> 4 byte length prefixed
	for(size_t i = 0; i < nals_count; i++)
	{
		size_t size = nals[i].size;
		if(!size)
			continue;
		dst[0] = (char)((size >> 24) & 0xff);
		dst[1] = (char)((size >> 16) & 0xff);
		dst[2] = (char)((size >> 8) & 0xff);
		dst[3] = (char)(size & 0xff);
		memcpy(dst + VT_NAL_LENGTH_SIZE, nals[i].data, size);
		dst += VT_NAL_LENGTH_SIZE + size;
	}

	CMSampleTimingInfo timing = {
		.duration = CMTimeMake(duration, 1000000),
		.presentationTimeStamp = CMTimeMake(pts, 1000000),
		.decodeTimeStamp = kCMTimeInvalid
	};
	CMSampleBufferRef sample = NULL;
	status = CMSampleBufferCreateReady(kCFAllocatorDefault, block, vt->format, 1, 1, &timing, 1, &sample_size, &sample);
	CFRelease(block);
	if(status != noErr || !sample)
	{
		CHIAKI_LOGE(vt_log(vt), "VideoToolbox native decoder failed to create sample buffer: status %d", (int)status);
		return CHIAKI_VT_DECODE_ERROR;
	}

	VTDecodeFrameFlags flags = vt->async ? kVTDecodeFrame_EnableAsynchronousDecompression : 0;
	VTDecodeInfoFlags info = 0;
	status = VTDecompressionSessionDecodeFrame(vt->session, sample, flags, NULL, &info);
	CFRelease(sample);
	if(status != noErr)
	{
		CHIAKI_LOGE(vt_log(vt), "VideoToolbox native decoder failed to submit frame: status %d", (int)status);
		if(status == kVTInvalidSessionErr)
			vt_session_destroy(vt); // recreated on the next frame
		return CHIAKI_VT_DECODE_ERROR;
	}
	return CHIAKI_VT_DECODE_OK;
}
