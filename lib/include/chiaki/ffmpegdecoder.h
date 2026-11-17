#ifndef CHIAKI_FFMPEG_DECODER_H
#define CHIAKI_FFMPEG_DECODER_H

#include <chiaki/config.h>
#include <chiaki/log.h>
#include <chiaki/thread.h>
#include <chiaki/latencystats.h>

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#include <libavcodec/avcodec.h>
#include <libavutil/rational.h>

typedef struct chiaki_ffmpeg_decoder_t ChiakiFfmpegDecoder;

typedef struct chiaki_ffmpeg_frame_t ChiakiFfmpegFrame;

typedef void (*ChiakiFfmpegFrameAvailable)(ChiakiFfmpegDecoder *decover, void *user);

struct chiaki_ffmpeg_frame_t
{
	AVFrame *frame;
	double pts;
	double duration;
	bool recovered;
	/**
	 * chiaki_time_now_monotonic_us() when the first network packet of this frame
	 * arrived and when it finished decoding, 0 if unknown. Latency instrumentation only.
	 */
	uint64_t first_packet_us;
	uint64_t decoded_us;
};

/**
 * Maximum number of access units (frames) waiting for the decode thread.
 * When a new frame arrives while this many are queued, all queued frames are
 * dropped and the video sample callback returns false so that an IDR frame
 * is requested (P-frames can not be dropped selectively).
 */
#define CHIAKI_FFMPEG_DECODER_QUEUE_MAX_FRAMES 8
/**
 * Ring buffer capacity. Larger than CHIAKI_FFMPEG_DECODER_QUEUE_MAX_FRAMES so
 * parameter set units (stream headers) are never dropped.
 */
#define CHIAKI_FFMPEG_DECODER_QUEUE_CAPACITY 16

struct chiaki_vt_decoder_t;

typedef struct chiaki_ffmpeg_decoder_unit_t
{
	AVPacket *packet;
	bool recovered;
	bool parameter_sets;
} ChiakiFfmpegDecoderUnit;

/**
 * Latency instrumentation: submit time and first packet arrival of a sample,
 * found again by pts when its frame comes out of the decoder.
 */
#define CHIAKI_FFMPEG_DECODER_LATENCY_RING 32

typedef struct chiaki_ffmpeg_decoder_latency_entry_t
{
	int64_t pts;
	uint64_t submit_us;
	uint64_t first_packet_us;
} ChiakiFfmpegDecoderLatencyEntry;

struct chiaki_ffmpeg_decoder_t
{
	ChiakiLog *log;
	/**
	 * Protects codec_context. Only held around avcodec calls,
	 * never while waiting for packets.
	 */
	ChiakiMutex mutex;
	const AVCodec *av_codec;
	AVCodecContext *codec_context;
	enum AVPixelFormat hw_pix_fmt;
	AVBufferRef *hw_device_ctx;
	char hw_decoder_name_buf[32];
	bool hdr_enabled;
	ChiakiMutex cb_mutex;
	ChiakiFfmpegFrameAvailable frame_available_cb;
	void *frame_available_cb_user;
	int32_t frames_lost;
	bool frame_recovered;
	int32_t session_bitrate_kbps;
	int64_t synthetic_packet_pts;
	AVRational synthetic_time_base;
	AVRational synthetic_framerate;
	double synthetic_frame_duration_us;
	double synthetic_candidate_duration_us;
	uint64_t synthetic_last_sample_time_us;
	uint8_t synthetic_candidate_count;

	/**
	 * Decode thread: the video sample callback (Takion receive thread) only
	 * queues access units, the decode thread runs avcodec and stores the newest
	 * decoded frame for chiaki_ffmpeg_decoder_pull_frame().
	 */
	ChiakiThread decode_thread;
	/**
	 * Protects the queue, latest_frame, frames_lost, frame_recovered,
	 * the synthetic_* timing state and the frame_* timing snapshot.
	 */
	ChiakiMutex queue_mutex;
	ChiakiCond queue_cond;
	bool decode_thread_stop;
	/**
	 * Set by the decode thread when avcodec failed on a frame. The next video
	 * sample callback clears it and returns false so that an IDR is requested.
	 */
	bool decode_error;
	ChiakiFfmpegDecoderUnit queue[CHIAKI_FFMPEG_DECODER_QUEUE_CAPACITY];
	size_t queue_head;
	size_t queue_count;
	AVFrame *latest_frame;
	AVRational frame_pkt_timebase;
	AVRational frame_ctx_timebase;
	AVRational frame_framerate;
	bool has_b_frames_logged; // only accessed by the decode thread
	int last_frame_format; // enum AVPixelFormat of the last decoded frame, decode thread only

	/**
	 * Native VideoToolbox backend (macOS), used instead of avcodec when the
	 * "videotoolbox" hw decoder is requested. NULL when avcodec decodes.
	 * Environment overrides: CHIAKI_VT_DECODER=ffmpeg uses FFmpeg's videotoolbox
	 * hwaccel instead, CHIAKI_VT_ASYNC=1 enables asynchronous decompression.
	 */
	struct chiaki_vt_decoder_t *vt_decoder;
	AVPacket *vt_last_header; // decode thread only, replayed to avcodec on fallback

	// Latency instrumentation, protected by queue_mutex
	ChiakiFfmpegDecoderLatencyEntry latency_ring[CHIAKI_FFMPEG_DECODER_LATENCY_RING];
	size_t latency_ring_next;
	ChiakiLatencyStats decode_stats; // submit (sample callback) -> frame ready, includes queue wait
	uint64_t decode_stats_start_us;
	uint64_t latest_frame_first_packet_us;
	uint64_t latest_frame_decoded_us;
};

CHIAKI_EXPORT ChiakiErrorCode chiaki_ffmpeg_decoder_init(ChiakiFfmpegDecoder *decoder, ChiakiLog *log,
		ChiakiCodec codec, unsigned int max_fps, const char *hw_decoder_name, AVBufferRef *hw_device_ctx,
		ChiakiFfmpegFrameAvailable frame_available_cb, void *frame_available_cb_user);
CHIAKI_EXPORT void chiaki_ffmpeg_decoder_fini(ChiakiFfmpegDecoder *decoder);
CHIAKI_EXPORT bool chiaki_ffmpeg_decoder_video_sample_cb(uint8_t *buf, size_t buf_size, int32_t frames_lost, bool frame_recovered, void *user);
CHIAKI_EXPORT ChiakiFfmpegFrame chiaki_ffmpeg_decoder_pull_frame(ChiakiFfmpegDecoder *decoder, int32_t *frames_lost);
CHIAKI_EXPORT enum AVPixelFormat chiaki_ffmpeg_decoder_get_pixel_format(ChiakiFfmpegDecoder *decoder);

/**
 * Compute the wall-clock pts (seconds) and frame duration (seconds) from raw
 * AVFrame timestamp fields and codec context timing metadata.  Exposed for
 * unit testing; normally called internally by chiaki_ffmpeg_decoder_pull_frame.
 *
 * Fallback chain for time_base: pkt_timebase → ctx_timebase → {1,1000000}.
 * Fallback chain for pts:       best_effort_timestamp → pts → 0.
 * Fallback for fps:             framerate or 60.0.
 */
CHIAKI_EXPORT void chiaki_ffmpeg_frame_get_timing(
	AVFrame *frame,
	AVRational pkt_timebase,
	AVRational ctx_timebase,
	AVRational framerate,
	double *out_pts,
	double *out_duration);

#ifdef __cplusplus
}
#endif

#endif // CHIAKI_FFMPEG_DECODER_H
