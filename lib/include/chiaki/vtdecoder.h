// SPDX-License-Identifier: LicenseRef-AGPL-3.0-only-OpenSSL

#ifndef CHIAKI_VT_DECODER_H
#define CHIAKI_VT_DECODER_H

/**
 * Native VideoToolbox backend of ChiakiFfmpegDecoder (macOS only).
 *
 * Decodes Annex B H.264/HEVC access units with a VTDecompressionSession that
 * requires the hardware decoder, and outputs AVFrames in the same form as
 * FFmpeg's videotoolbox hwaccel: format AV_PIX_FMT_VIDEOTOOLBOX, data[3] is a
 * retained IOSurface-backed, Metal-compatible CVPixelBufferRef (NV12 or P010),
 * buf[0] releases it and hw_frames_ctx describes it, so av_hwframe_transfer_data()
 * and av_hwframe_map() keep working.
 *
 * Not thread-safe: chiaki_vt_decoder_decode() and chiaki_vt_decoder_free() must
 * be called from one thread (the FFmpeg decoder's decode thread). The output
 * callback is called from inside chiaki_vt_decoder_decode() in synchronous mode,
 * or from a VideoToolbox thread in asynchronous mode.
 */

#include <chiaki/common.h>
#include <chiaki/log.h>

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#include <libavutil/frame.h>

typedef struct chiaki_vt_decoder_t ChiakiVtDecoder;

/**
 * @param frame decoded frame (ownership is passed to the callback) or NULL if decoding failed
 */
typedef void (*ChiakiVtDecoderOutput)(AVFrame *frame, void *user);

/**
 * @param log pointer to the log pointer to use (read on every log call, so the owner may replace the log)
 * @param async use kVTDecodeFrame_EnableAsynchronousDecompression
 * @return NULL on failure
 */
ChiakiVtDecoder *chiaki_vt_decoder_new(ChiakiLog **log, ChiakiCodec codec, bool async,
		ChiakiVtDecoderOutput output_cb, void *output_cb_user);
void chiaki_vt_decoder_free(ChiakiVtDecoder *vt);

typedef enum chiaki_vt_decode_result_t
{
	/** submitted (or only parameter sets, nothing to decode) */
	CHIAKI_VT_DECODE_OK,
	/** decoding this access unit failed, request an IDR */
	CHIAKI_VT_DECODE_ERROR,
	/** the hardware session can not be used for this stream, fall back to FFmpeg */
	CHIAKI_VT_DECODE_FATAL
} ChiakiVtDecodeResult;

/**
 * Decode one Annex B access unit. Parameter sets in the unit (re)configure the session.
 * @param pts presentation timestamp in microseconds
 * @param duration duration in microseconds
 */
ChiakiVtDecodeResult chiaki_vt_decoder_decode(ChiakiVtDecoder *vt, const uint8_t *buf, size_t buf_size,
		int64_t pts, int64_t duration);

/**
 * Whether the current session reports kVTDecompressionPropertyKey_UsingHardwareAcceleratedVideoDecoder.
 */
bool chiaki_vt_decoder_using_hardware(ChiakiVtDecoder *vt);

#ifdef __cplusplus
}
#endif

#endif // CHIAKI_VT_DECODER_H
