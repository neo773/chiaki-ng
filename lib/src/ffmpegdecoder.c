
#include <chiaki/ffmpegdecoder.h>
#include <chiaki/time.h>
#include <chiaki/videoreceiver.h>
#include <libavcodec/avcodec.h>
#include <libavutil/pixdesc.h>
#include <limits.h>
#include <math.h>
#include <string.h>

#ifdef __APPLE__
#include <TargetConditionals.h>
#if TARGET_OS_OSX
#define CHIAKI_FFMPEG_DECODER_NATIVE_VT 1
#include <chiaki/vtdecoder.h>
#include <os/signpost.h>
#include <stdlib.h>
#endif
#endif

static void *ffmpeg_decoder_thread_func(void *user);

#ifdef CHIAKI_FFMPEG_DECODER_NATIVE_VT
// "decode" intervals (sample callback -> frame ready) for Instruments' Points of Interest
static os_log_t ffmpeg_decoder_signpost_log = NULL;
#endif

/**
 * Remember when a sample was submitted and when its first packet arrived.
 * Must be called with queue_mutex held, from the video sample callback.
 */
static void ffmpeg_decoder_latency_note_submit(ChiakiFfmpegDecoder *decoder, int64_t pts)
{
	ChiakiFfmpegDecoderLatencyEntry *entry = &decoder->latency_ring[decoder->latency_ring_next];
	decoder->latency_ring_next = (decoder->latency_ring_next + 1) % CHIAKI_FFMPEG_DECODER_LATENCY_RING;
	entry->pts = pts;
	entry->submit_us = chiaki_time_now_monotonic_us();
	entry->first_packet_us = chiaki_video_receiver_current_sample_first_packet_us();
#ifdef CHIAKI_FFMPEG_DECODER_NATIVE_VT
	if(ffmpeg_decoder_signpost_log)
		os_signpost_interval_begin(ffmpeg_decoder_signpost_log, (os_signpost_id_t)(pts + 1), "decode");
#endif
}

/**
 * The decoder output frame, which becomes latest_frame: record submit -> frame ready.
 * Must be called with queue_mutex held.
 * @return true if the decode stats are due to be logged (after unlocking), summary is set then
 */
static bool ffmpeg_decoder_latency_note_frame(ChiakiFfmpegDecoder *decoder, const AVFrame *frame, ChiakiLatencySummary *summary)
{
	uint64_t now_us = chiaki_time_now_monotonic_us();
	decoder->latest_frame_first_packet_us = 0;
	decoder->latest_frame_decoded_us = now_us;
	for(size_t i = 0; frame->pts != AV_NOPTS_VALUE && i < CHIAKI_FFMPEG_DECODER_LATENCY_RING; i++)
	{
		ChiakiFfmpegDecoderLatencyEntry *entry = &decoder->latency_ring[i];
		if(entry->pts != frame->pts)
			continue;
		decoder->latest_frame_first_packet_us = entry->first_packet_us;
		if(now_us >= entry->submit_us)
			chiaki_latency_stats_add(&decoder->decode_stats, now_us - entry->submit_us);
		entry->pts = AV_NOPTS_VALUE;
#ifdef CHIAKI_FFMPEG_DECODER_NATIVE_VT
		if(ffmpeg_decoder_signpost_log)
			os_signpost_interval_end(ffmpeg_decoder_signpost_log, (os_signpost_id_t)(frame->pts + 1), "decode");
#endif
		break;
	}
	if(!decoder->decode_stats_start_us)
		decoder->decode_stats_start_us = now_us;
	if(now_us - decoder->decode_stats_start_us < CHIAKI_LATENCY_STATS_INTERVAL_US)
		return false;
	decoder->decode_stats_start_us = now_us;
	*summary = chiaki_latency_stats_summarize(&decoder->decode_stats);
	return true;
}

static void ffmpeg_decoder_latency_log(ChiakiFfmpegDecoder *decoder, const ChiakiLatencySummary *s)
{
	CHIAKI_LOGI(decoder->log, "[latency] decode_ms p50=%.2f p99=%.2f max=%.2f frames=%u",
			s->p50_ms, s->p99_ms, s->max_ms, (unsigned int)s->frames);
}

#ifdef CHIAKI_FFMPEG_DECODER_NATIVE_VT
static bool ffmpeg_decoder_native_vt_enabled(void)
{
	const char *env = getenv("CHIAKI_VT_DECODER");
	return !env || strcmp(env, "ffmpeg") != 0;
}

static bool ffmpeg_decoder_native_vt_async(void)
{
	const char *env = getenv("CHIAKI_VT_ASYNC");
	return env && strcmp(env, "1") == 0;
}

static void ffmpeg_decoder_vt_output(AVFrame *frame, void *user);
#endif

#ifdef __APPLE__
/**
 * Pick the hardware format, and make a fallback to software decoding loud.
 * FFmpeg calls this again without the hardware format if the hwaccel failed to initialize.
 */
static enum AVPixelFormat ffmpeg_decoder_get_format(AVCodecContext *ctx, const enum AVPixelFormat *fmts)
{
	ChiakiFfmpegDecoder *decoder = ctx->opaque;
	for(const enum AVPixelFormat *p = fmts; *p != AV_PIX_FMT_NONE; p++)
	{
		if(*p == decoder->hw_pix_fmt)
			return *p;
	}
	for(const enum AVPixelFormat *p = fmts; *p != AV_PIX_FMT_NONE; p++)
	{
		const AVPixFmtDescriptor *desc = av_pix_fmt_desc_get(*p);
		if(desc && !(desc->flags & AV_PIX_FMT_FLAG_HWACCEL))
		{
			CHIAKI_LOGE(decoder->log, "Hardware decoder %s could not be initialized for this stream, falling back to SOFTWARE decoding (%s)",
					av_get_pix_fmt_name(decoder->hw_pix_fmt), desc->name);
			return *p;
		}
	}
	CHIAKI_LOGE(decoder->log, "No usable pixel format offered by the decoder");
	return AV_PIX_FMT_NONE;
}
#endif

static enum AVCodecID chiaki_codec_av_codec_id(ChiakiCodec codec)
{
	switch(codec)
	{
		case CHIAKI_CODEC_H265:
		case CHIAKI_CODEC_H265_HDR:
			return AV_CODEC_ID_H265;
		default:
			return AV_CODEC_ID_H264;
	}
}

static double chiaki_ffmpeg_decoder_default_frame_duration_us(unsigned int max_fps)
{
	double fps = max_fps > 0 ? (double)max_fps : 60.0;
	if(fps <= 0.0)
		fps = 60.0;
	return 1000000.0 / fps;
}

/**
 * Open the avcodec context (and its hw_device_ctx if configured) if not already open.
 * Must be called with decoder->mutex held.
 */
static bool ffmpeg_decoder_open_avcodec(ChiakiFfmpegDecoder *decoder)
{
	if(decoder->codec_context)
		return true;

	decoder->codec_context = avcodec_alloc_context3(decoder->av_codec);
	if(!decoder->codec_context)
	{
		CHIAKI_LOGE(decoder->log, "Failed to alloc codec context");
		return false;
	}

	const char *hw_decoder_name = decoder->hw_decoder_name_buf[0] ? decoder->hw_decoder_name_buf : NULL;
	if(hw_decoder_name)
	{
		CHIAKI_LOGI(decoder->log, "Trying to use hardware decoder \"%s\"", hw_decoder_name);
		enum AVHWDeviceType type = av_hwdevice_find_type_by_name(hw_decoder_name);
		if(type == AV_HWDEVICE_TYPE_NONE)
		{
			CHIAKI_LOGE(decoder->log, "Hardware decoder \"%s\" not found", hw_decoder_name);
			goto fail;
		}

		for(int i = 0;; i++)
		{
			const AVCodecHWConfig *config = avcodec_get_hw_config(decoder->av_codec, i);
			if(!config)
			{
				CHIAKI_LOGE(decoder->log, "avcodec_get_hw_config failed");
				goto fail;
			}
			if(config->methods & AV_CODEC_HW_CONFIG_METHOD_HW_DEVICE_CTX && config->device_type == type)
			{
				decoder->hw_pix_fmt = config->pix_fmt;
				break;
			}
		}

		if(!decoder->hw_device_ctx && av_hwdevice_ctx_create(&decoder->hw_device_ctx, type, NULL, NULL, 0) < 0)
		{
			CHIAKI_LOGE(decoder->log, "Failed to create hwdevice context");
			goto fail;
		}
		decoder->codec_context->hw_device_ctx = av_buffer_ref(decoder->hw_device_ctx);
#ifdef __APPLE__
		decoder->codec_context->opaque = decoder;
		decoder->codec_context->get_format = ffmpeg_decoder_get_format;
#endif
		CHIAKI_LOGI(decoder->log, "Using hardware decoder \"%s\" with pix_fmt=%s", hw_decoder_name, av_get_pix_fmt_name(decoder->hw_pix_fmt));
	}

	decoder->codec_context->framerate = decoder->synthetic_framerate;
	decoder->codec_context->pkt_timebase = decoder->synthetic_time_base;
	decoder->codec_context->time_base = decoder->synthetic_time_base;
	// The stream has no B-frames, output every frame as soon as it is decoded
	decoder->codec_context->flags |= AV_CODEC_FLAG_LOW_DELAY;

	if(avcodec_open2(decoder->codec_context, decoder->av_codec, NULL) < 0)
	{
		CHIAKI_LOGE(decoder->log, "Failed to open codec context");
		goto fail;
	}
	decoder->frame_pkt_timebase = decoder->codec_context->pkt_timebase;
	decoder->frame_ctx_timebase = decoder->codec_context->time_base;
	decoder->frame_framerate = decoder->codec_context->framerate;
	return true;

fail:
	avcodec_free_context(&decoder->codec_context);
	return false;
}

CHIAKI_EXPORT ChiakiErrorCode chiaki_ffmpeg_decoder_init(ChiakiFfmpegDecoder *decoder, ChiakiLog *log,
		ChiakiCodec codec, unsigned int max_fps, const char *hw_decoder_name, AVBufferRef *hw_device_ctx,
		ChiakiFfmpegFrameAvailable frame_available_cb, void *frame_available_cb_user)
{
	ChiakiErrorCode err = chiaki_mutex_init(&decoder->mutex, false);
	if(err != CHIAKI_ERR_SUCCESS)
		return err;
	err = chiaki_mutex_init(&decoder->queue_mutex, false);
	if(err != CHIAKI_ERR_SUCCESS)
	{
		chiaki_mutex_fini(&decoder->mutex);
		return err;
	}
	err = chiaki_cond_init(&decoder->queue_cond);
	if(err != CHIAKI_ERR_SUCCESS)
	{
		chiaki_mutex_fini(&decoder->queue_mutex);
		chiaki_mutex_fini(&decoder->mutex);
		return err;
	}
	decoder->decode_thread_stop = false;
	decoder->decode_error = false;
	memset(decoder->queue, 0, sizeof(decoder->queue));
	decoder->queue_head = 0;
	decoder->queue_count = 0;
	decoder->latest_frame = NULL;
	decoder->has_b_frames_logged = false;
	decoder->last_frame_format = AV_PIX_FMT_NONE;
	decoder->vt_decoder = NULL;
	decoder->vt_last_header = NULL;
	for(size_t i = 0; i < CHIAKI_FFMPEG_DECODER_LATENCY_RING; i++)
		decoder->latency_ring[i] = (ChiakiFfmpegDecoderLatencyEntry){ .pts = AV_NOPTS_VALUE };
	decoder->latency_ring_next = 0;
	memset(&decoder->decode_stats, 0, sizeof(decoder->decode_stats));
	decoder->decode_stats_start_us = 0;
	decoder->latest_frame_first_packet_us = 0;
	decoder->latest_frame_decoded_us = 0;
#ifdef CHIAKI_FFMPEG_DECODER_NATIVE_VT
	// Off unless asked for: each interval costs ~0.4 us even when Instruments is not recording
	if(!ffmpeg_decoder_signpost_log && getenv("CHIAKI_SIGNPOSTS"))
		ffmpeg_decoder_signpost_log = os_log_create("chiaki-ng", OS_LOG_CATEGORY_POINTS_OF_INTEREST);
#endif

	chiaki_mutex_lock(&decoder->mutex);
	decoder->log = log;
	decoder->frame_available_cb = frame_available_cb;
	decoder->frame_available_cb_user = frame_available_cb_user;
	decoder->hdr_enabled = codec == CHIAKI_CODEC_H265_HDR;
	decoder->frames_lost = 0;
	decoder->frame_recovered = false;
	decoder->synthetic_packet_pts = 0;
	decoder->synthetic_framerate = (AVRational){max_fps > 0 ? (int)max_fps : 60, 1};
	decoder->synthetic_time_base = (AVRational){1, 1000000};
	decoder->synthetic_frame_duration_us = chiaki_ffmpeg_decoder_default_frame_duration_us(max_fps);
	decoder->synthetic_candidate_duration_us = decoder->synthetic_frame_duration_us;
	decoder->synthetic_last_sample_time_us = 0;
	decoder->synthetic_candidate_count = 0;

	decoder->codec_context = NULL;
	decoder->hw_device_ctx = hw_device_ctx ? av_buffer_ref(hw_device_ctx) : NULL;
	decoder->hw_pix_fmt = AV_PIX_FMT_NONE;
	memset(decoder->hw_decoder_name_buf, 0, sizeof(decoder->hw_decoder_name_buf));
	if(hw_decoder_name)
		snprintf(decoder->hw_decoder_name_buf, sizeof(decoder->hw_decoder_name_buf), "%s", hw_decoder_name);

	decoder->frame_pkt_timebase = decoder->synthetic_time_base;
	decoder->frame_ctx_timebase = decoder->synthetic_time_base;
	decoder->frame_framerate = decoder->synthetic_framerate;

#if LIBAVCODEC_VERSION_INT < AV_VERSION_INT(58, 10, 100)
	avcodec_register_all();
#endif
	enum AVCodecID av_codec = chiaki_codec_av_codec_id(codec);
	decoder->av_codec = avcodec_find_decoder(av_codec);
	if(!decoder->av_codec)
	{
		CHIAKI_LOGE(log, "%s Codec not available", chiaki_codec_name(codec));
		goto error_mutex;
	}
	chiaki_mutex_unlock(&decoder->mutex);

#ifdef CHIAKI_FFMPEG_DECODER_NATIVE_VT
	// Initialize native VideoToolbox directly without opening an unused FFmpeg hwaccel context;
	// if native VideoToolbox ever fails fatally, ffmpeg_decoder_open_avcodec() opens the fallback on demand.
	if(hw_decoder_name && strcmp(hw_decoder_name, "videotoolbox") == 0 && ffmpeg_decoder_native_vt_enabled())
	{
		decoder->vt_decoder = chiaki_vt_decoder_new(&decoder->log, codec, ffmpeg_decoder_native_vt_async(),
				ffmpeg_decoder_vt_output, decoder);
		if(decoder->vt_decoder)
			CHIAKI_LOGI(log, "Using native VideoToolbox decoder (%s), set CHIAKI_VT_DECODER=ffmpeg to use FFmpeg's videotoolbox hwaccel instead",
					ffmpeg_decoder_native_vt_async() ? "async" : "sync");
		else
			CHIAKI_LOGE(log, "Native VideoToolbox decoder is not available, falling back to FFmpeg's videotoolbox hwaccel");
	}
#endif

	if(!decoder->vt_decoder)
	{
		chiaki_mutex_lock(&decoder->mutex);
		if(!ffmpeg_decoder_open_avcodec(decoder))
			goto error_codec_context;
		chiaki_mutex_unlock(&decoder->mutex);
	}

	err = chiaki_thread_create(&decoder->decode_thread, ffmpeg_decoder_thread_func, decoder);
	if(err != CHIAKI_ERR_SUCCESS)
	{
		CHIAKI_LOGE(log, "Failed to create decoder thread");
#ifdef CHIAKI_FFMPEG_DECODER_NATIVE_VT
		chiaki_vt_decoder_free(decoder->vt_decoder);
		decoder->vt_decoder = NULL;
#endif
		chiaki_mutex_lock(&decoder->mutex);
		goto error_codec_context;
	}
	chiaki_thread_set_name(&decoder->decode_thread, "Chiaki Decoder");
	return CHIAKI_ERR_SUCCESS;
error_codec_context:
	if(decoder->hw_device_ctx)
		av_buffer_unref(&decoder->hw_device_ctx);
	avcodec_free_context(&decoder->codec_context);
error_mutex:
	chiaki_mutex_unlock(&decoder->mutex);
	chiaki_cond_fini(&decoder->queue_cond);
	chiaki_mutex_fini(&decoder->queue_mutex);
	chiaki_mutex_fini(&decoder->mutex);
	return CHIAKI_ERR_UNKNOWN;
}

CHIAKI_EXPORT void chiaki_ffmpeg_decoder_fini(ChiakiFfmpegDecoder *decoder)
{
	chiaki_mutex_lock(&decoder->queue_mutex);
	decoder->decode_thread_stop = true;
	chiaki_cond_signal(&decoder->queue_cond);
	chiaki_mutex_unlock(&decoder->queue_mutex);
	chiaki_thread_join(&decoder->decode_thread, NULL);
#ifdef CHIAKI_FFMPEG_DECODER_NATIVE_VT
	// waits for asynchronous frames, the output callback does not notify after stop
	chiaki_vt_decoder_free(decoder->vt_decoder);
	decoder->vt_decoder = NULL;
#endif
	av_packet_free(&decoder->vt_last_header);

	for(size_t i = 0; i < CHIAKI_FFMPEG_DECODER_QUEUE_CAPACITY; i++)
		av_packet_free(&decoder->queue[i].packet);
	decoder->queue_count = 0;
	av_frame_free(&decoder->latest_frame);

	chiaki_mutex_lock(&decoder->mutex);
	avcodec_free_context(&decoder->codec_context);
	if(decoder->hw_device_ctx)
		av_buffer_unref(&decoder->hw_device_ctx);
	chiaki_mutex_unlock(&decoder->mutex);
	chiaki_cond_fini(&decoder->queue_cond);
	chiaki_mutex_fini(&decoder->queue_mutex);
	chiaki_mutex_fini(&decoder->mutex);
}

/**
 * Classify an Annex B access unit by its NAL unit types.
 * @param parameter_sets_only set if it contains parameter sets (H.264 SPS/PPS,
 * H.265 VPS/SPS/PPS) but no slices, i.e. it is a stream header
 * @param keyframe set if it contains a random access slice (H.264 IDR, H.265 IRAP),
 * which the decoder can start from without any earlier frame
 */
static void ffmpeg_decoder_classify_unit(ChiakiFfmpegDecoder *decoder, const uint8_t *buf, size_t buf_size,
		bool *parameter_sets_only, bool *keyframe)
{
	bool hevc = decoder->av_codec->id == AV_CODEC_ID_H265;
	bool parameter_sets = false;
	bool slices = false;
	*keyframe = false;
	if(buf_size >= 4)
	{
		const uint8_t *search = buf + 2;
		const uint8_t *search_end = buf + buf_size - 1;
		while(search < search_end)
		{
			const uint8_t *one = memchr(search, 1, (size_t)(search_end - search));
			if(!one)
				break;
			if(one[-1] != 0 || one[-2] != 0)
			{
				search = one + 1;
				continue;
			}
			uint8_t nal_header = one[1];
			if(hevc)
			{
				uint8_t nal_type = (nal_header >> 1) & 0x3f;
				if(nal_type >= 32 && nal_type <= 34) // VPS, SPS, PPS
					parameter_sets = true;
				else if(nal_type < 32)
				{
					slices = true;
					if(nal_type >= 16 && nal_type <= 21) // BLA, IDR, CRA
						*keyframe = true;
				}
			}
			else
			{
				uint8_t nal_type = nal_header & 0x1f;
				if(nal_type == 7 || nal_type == 8) // SPS, PPS
					parameter_sets = true;
				else if(nal_type >= 1 && nal_type <= 5)
				{
					slices = true;
					if(nal_type == 5) // IDR
						*keyframe = true;
				}
			}
			search = one + 4;
		}
	}
	*parameter_sets_only = parameter_sets && !slices;
}

/**
 * Update the synthetic timing state from the arrival time of a sample.
 * Must be called with queue_mutex held.
 * @return the synthetic duration of one frame in synthetic_time_base units
 */
static int64_t ffmpeg_decoder_update_synthetic_timing(ChiakiFfmpegDecoder *decoder, int32_t frames_lost)
{
	if(decoder->synthetic_last_sample_time_us)
	{
		double observed_duration_us = (double)(chiaki_time_now_monotonic_us() - decoder->synthetic_last_sample_time_us);
		int64_t delivered_frames = (int64_t)frames_lost + 1;
		double default_duration_us = chiaki_ffmpeg_decoder_default_frame_duration_us((unsigned int)decoder->synthetic_framerate.num);
		if(delivered_frames > 1)
			observed_duration_us /= (double)delivered_frames;
		if(observed_duration_us < default_duration_us)
			observed_duration_us = default_duration_us;
		else if(observed_duration_us > 1000000.0 / 15.0)
			observed_duration_us = 1000000.0 / 15.0;

		double candidate_diff = decoder->synthetic_candidate_duration_us > 0.0
			? fabs(observed_duration_us - decoder->synthetic_candidate_duration_us) / decoder->synthetic_candidate_duration_us
			: 1.0;
		double current_diff = decoder->synthetic_frame_duration_us > 0.0
			? fabs(observed_duration_us - decoder->synthetic_frame_duration_us) / decoder->synthetic_frame_duration_us
			: 1.0;
		if(current_diff >= 0.20)
		{
			if(candidate_diff <= 0.10)
				decoder->synthetic_candidate_count++;
			else
			{
				decoder->synthetic_candidate_duration_us = observed_duration_us;
				decoder->synthetic_candidate_count = 1;
			}
			if(decoder->synthetic_candidate_count >= 3)
			{
				decoder->synthetic_frame_duration_us = decoder->synthetic_candidate_duration_us;
				decoder->synthetic_candidate_count = 0;
			}
		}
		else
		{
			decoder->synthetic_candidate_duration_us = decoder->synthetic_frame_duration_us;
			decoder->synthetic_candidate_count = 0;
		}
	}
	decoder->synthetic_last_sample_time_us = chiaki_time_now_monotonic_us();

	int64_t synthetic_duration_pts = (int64_t)(decoder->synthetic_frame_duration_us + 0.5);
	if(synthetic_duration_pts < 1)
		synthetic_duration_pts = 1;
	return synthetic_duration_pts;
}

/**
 * Drop all queued units except parameter sets.
 * Must be called with queue_mutex held.
 * @return number of dropped units
 */
static size_t ffmpeg_decoder_queue_drop_frames(ChiakiFfmpegDecoder *decoder)
{
	ChiakiFfmpegDecoderUnit kept[CHIAKI_FFMPEG_DECODER_QUEUE_CAPACITY];
	size_t kept_count = 0;
	size_t dropped = 0;
	for(size_t i = 0; i < decoder->queue_count; i++)
	{
		ChiakiFfmpegDecoderUnit *unit = &decoder->queue[(decoder->queue_head + i) % CHIAKI_FFMPEG_DECODER_QUEUE_CAPACITY];
		if(unit->parameter_sets)
			kept[kept_count++] = *unit;
		else
		{
			av_packet_free(&unit->packet);
			dropped++;
		}
		unit->packet = NULL;
	}
	decoder->queue_head = 0;
	decoder->queue_count = kept_count;
	for(size_t i = 0; i < kept_count; i++)
		decoder->queue[i] = kept[i];
	return dropped;
}

CHIAKI_EXPORT bool chiaki_ffmpeg_decoder_video_sample_cb(uint8_t *buf, size_t buf_size, int32_t frames_lost, bool frame_recovered, void *user)
{
	ChiakiFfmpegDecoder *decoder = user;

	if(buf_size > (size_t)(INT_MAX - AV_INPUT_BUFFER_PADDING_SIZE))
	{
		CHIAKI_LOGE(decoder->log, "Video sample too large (%zu bytes)", buf_size);
		return false;
	}

	// Copy the access unit outside of the lock, buf is only valid during this call.
	AVPacket *packet = av_packet_alloc();
	if(!packet || av_new_packet(packet, (int)buf_size) < 0)
	{
		CHIAKI_LOGE(decoder->log, "Failed to alloc AVPacket");
		av_packet_free(&packet);
		return false;
	}
	memcpy(packet->data, buf, buf_size);
	bool parameter_sets, keyframe;
	ffmpeg_decoder_classify_unit(decoder, buf, buf_size, &parameter_sets, &keyframe);

	chiaki_mutex_lock(&decoder->queue_mutex);
	if(decoder->decode_thread_stop)
	{
		chiaki_mutex_unlock(&decoder->queue_mutex);
		av_packet_free(&packet);
		return false;
	}
	decoder->frames_lost += frames_lost;

	int64_t synthetic_duration_pts = ffmpeg_decoder_update_synthetic_timing(decoder, frames_lost);
	if(frames_lost > 0)
		decoder->synthetic_packet_pts += synthetic_duration_pts * (int64_t)frames_lost;
	packet->pts = decoder->synthetic_packet_pts;
	packet->dts = decoder->synthetic_packet_pts;
	ffmpeg_decoder_latency_note_submit(decoder, packet->pts);
	packet->duration = synthetic_duration_pts;
#if LIBAVCODEC_VERSION_INT >= AV_VERSION_INT(59, 8, 100)
	packet->time_base = decoder->synthetic_time_base;
#endif
	decoder->synthetic_packet_pts += synthetic_duration_pts;

	if(keyframe && decoder->decode_error)
	{
		// the keyframe does not depend on the frame that failed
		decoder->decode_error = false;
		CHIAKI_LOGI(decoder->log, "Keyframe after a failed frame, decoding resumes");
	}
	else if(!parameter_sets && decoder->decode_error)
	{
		// A previous frame failed to decode, so the reference chain is broken.
		// Reject this frame like the synchronous decoder did on failure; the video
		// receiver then skips P-frames and requests an IDR.
		decoder->decode_error = false;
		decoder->frames_lost++;
		chiaki_mutex_unlock(&decoder->queue_mutex);
		av_packet_free(&packet);
		CHIAKI_LOGW(decoder->log, "Previous frame failed to decode, rejecting frame to request IDR");
		return false;
	}

	if(keyframe && decoder->queue_count >= CHIAKI_FFMPEG_DECODER_QUEUE_MAX_FRAMES)
	{
		// The new frame does not depend on the queued ones: drop them and start from it.
		size_t dropped = ffmpeg_decoder_queue_drop_frames(decoder);
		decoder->frames_lost += (int32_t)dropped;
		CHIAKI_LOGW(decoder->log, "Decoder queue overflow, dropped %zu queued frames, continuing with the new keyframe", dropped);
	}
	else if(decoder->queue_count >= CHIAKI_FFMPEG_DECODER_QUEUE_CAPACITY
		|| (!parameter_sets && decoder->queue_count >= CHIAKI_FFMPEG_DECODER_QUEUE_MAX_FRAMES))
	{
		// Decoding fell behind. Queued P-frames reference each other, so the only
		// safe way out is to drop all of them and start again from an IDR frame.
		// Returning false makes the video receiver request it.
		size_t queued = decoder->queue_count;
		size_t dropped = ffmpeg_decoder_queue_drop_frames(decoder);
		decoder->frames_lost += (int32_t)dropped + 1;
		chiaki_mutex_unlock(&decoder->queue_mutex);
		av_packet_free(&packet);
		CHIAKI_LOGW(decoder->log, "Decoder queue overflow (%zu units queued), dropped %zu queued frames and the new one, requesting IDR",
				queued, dropped);
		return false;
	}

	ChiakiFfmpegDecoderUnit *unit = &decoder->queue[(decoder->queue_head + decoder->queue_count) % CHIAKI_FFMPEG_DECODER_QUEUE_CAPACITY];
	unit->packet = packet;
	unit->recovered = frame_recovered;
	unit->parameter_sets = parameter_sets;
	decoder->queue_count++;
	chiaki_cond_signal(&decoder->queue_cond);
	chiaki_mutex_unlock(&decoder->queue_mutex);
	return true;
}

/**
 * Receive one frame from the codec into *spare and swap it with *newest.
 * Must be called with decoder->mutex held.
 * @return true if a frame was received
 */
static bool ffmpeg_decoder_receive_frame(ChiakiFfmpegDecoder *decoder, AVFrame **newest, AVFrame **spare, bool *error)
{
	if(!*spare)
	{
		*spare = av_frame_alloc();
		if(!*spare)
		{
			CHIAKI_LOGE(decoder->log, "Failed to alloc AVFrame");
			*error = true;
			return false;
		}
	}
	int r = avcodec_receive_frame(decoder->codec_context, *spare);
	if(r != 0)
	{
		if(r != AVERROR(EAGAIN) && r != AVERROR_EOF)
		{
			CHIAKI_LOGE(decoder->log, "Decoding with FFMPEG failed");
			*error = true;
		}
		return false;
	}
	// newest frame wins
	AVFrame *older = *newest;
	*newest = *spare;
	*spare = older;
	if(*spare)
		av_frame_unref(*spare);
	return true;
}

/**
 * Log the reorder delay once and every switch between hardware and software output.
 * Must be called with decoder->mutex held.
 */
static void ffmpeg_decoder_check_frame(ChiakiFfmpegDecoder *decoder, AVFrame *frame)
{
	AVCodecContext *ctx = decoder->codec_context;
	if(!decoder->has_b_frames_logged)
	{
		decoder->has_b_frames_logged = true;
		if(ctx->has_b_frames > 0)
			CHIAKI_LOGI(decoder->log, "First frame decoded, decoder delays output by has_b_frames=%d frame(s)", ctx->has_b_frames);
		else
			CHIAKI_LOGI(decoder->log, "First frame decoded, has_b_frames=0 (no reorder delay)");
	}
	if(frame->format == decoder->last_frame_format)
		return;
	decoder->last_frame_format = frame->format;
	if(decoder->hw_pix_fmt == AV_PIX_FMT_NONE)
		return; // software decoding was requested
	const AVPixFmtDescriptor *sw_desc = av_pix_fmt_desc_get(ctx->sw_pix_fmt);
	const char *profile = avcodec_profile_name(ctx->codec_id, ctx->profile);
	int depth = sw_desc ? sw_desc->comp[0].depth : 0;
	if(frame->format == decoder->hw_pix_fmt)
	{
		if(frame->format == AV_PIX_FMT_VIDEOTOOLBOX)
			CHIAKI_LOGI(decoder->log, "VideoToolbox hardware decode active: %s %s %d-bit %dx%d",
					avcodec_get_name(ctx->codec_id), profile ? profile : "unknown profile", depth, frame->width, frame->height);
		else
			CHIAKI_LOGI(decoder->log, "Hardware decode active (%s): %s %s %d-bit %dx%d", av_get_pix_fmt_name(frame->format),
					avcodec_get_name(ctx->codec_id), profile ? profile : "unknown profile", depth, frame->width, frame->height);
	}
	else
	{
		const char *fmt_name = av_get_pix_fmt_name(frame->format);
		CHIAKI_LOGE(decoder->log, "Hardware decoder %s is configured but frames come out in SOFTWARE format %s: %s %s %d-bit %dx%d",
				av_get_pix_fmt_name(decoder->hw_pix_fmt), fmt_name ? fmt_name : "unknown",
				avcodec_get_name(ctx->codec_id), profile ? profile : "unknown profile", depth, frame->width, frame->height);
	}
}

/**
 * Feed one packet to the codec and return the newest frame it produced, if any.
 * Called on the decode thread only.
 * @param error set to true if avcodec failed
 */
static AVFrame *ffmpeg_decoder_decode_packet(ChiakiFfmpegDecoder *decoder, AVPacket *packet,
		AVRational *pkt_timebase, AVRational *ctx_timebase, AVRational *framerate, bool *error)
{
	AVFrame *newest = NULL;
	AVFrame *spare = NULL;

	chiaki_mutex_lock(&decoder->mutex);
	while(true)
	{
		int r = avcodec_send_packet(decoder->codec_context, packet);
		if(r == 0)
			break;
		if(r == AVERROR(EAGAIN))
		{
			// The codec output is full: take a frame out (newest wins) and push again.
			if(ffmpeg_decoder_receive_frame(decoder, &newest, &spare, error))
				continue;
			CHIAKI_LOGE(decoder->log, "Failed to pull frame from full codec buffer");
		}
		else
		{
			char errbuf[128];
			av_make_error_string(errbuf, sizeof(errbuf), r);
			CHIAKI_LOGE(decoder->log, "Failed to push frame: %s", errbuf);
		}
		*error = true;
		break;
	}
	while(ffmpeg_decoder_receive_frame(decoder, &newest, &spare, error));

	if(newest)
		ffmpeg_decoder_check_frame(decoder, newest);
	*pkt_timebase = decoder->codec_context->pkt_timebase;
	*ctx_timebase = decoder->codec_context->time_base;
	*framerate = decoder->codec_context->framerate;
	chiaki_mutex_unlock(&decoder->mutex);

	av_frame_free(&spare);
	return newest;
}

#ifdef CHIAKI_FFMPEG_DECODER_NATIVE_VT
/**
 * Output of the native VideoToolbox decoder: called on the decode thread
 * (synchronous mode) or a VideoToolbox thread (asynchronous mode).
 */
static void ffmpeg_decoder_vt_output(AVFrame *frame, void *user)
{
	ChiakiFfmpegDecoder *decoder = user;
	chiaki_mutex_lock(&decoder->queue_mutex);
	if(!frame)
	{
		decoder->decode_error = true;
		chiaki_mutex_unlock(&decoder->queue_mutex);
		return;
	}
	av_frame_free(&decoder->latest_frame);
	decoder->latest_frame = frame;
	decoder->frame_pkt_timebase = decoder->synthetic_time_base;
	decoder->frame_ctx_timebase = decoder->synthetic_time_base;
	decoder->frame_framerate = decoder->synthetic_framerate;
	ChiakiLatencySummary latency;
	bool log_latency = ffmpeg_decoder_latency_note_frame(decoder, frame, &latency);
	bool stop = decoder->decode_thread_stop;
	chiaki_mutex_unlock(&decoder->queue_mutex);
	if(log_latency)
		ffmpeg_decoder_latency_log(decoder, &latency);
	if(!stop)
		decoder->frame_available_cb(decoder, decoder->frame_available_cb_user);
}

/**
 * Decode one unit with the native VideoToolbox decoder. Frames are delivered through
 * ffmpeg_decoder_vt_output(). If the hardware session can not be used, switch to
 * avcodec for good and return its frame, if any.
 */
static AVFrame *ffmpeg_decoder_vt_decode_unit(ChiakiFfmpegDecoder *decoder, ChiakiFfmpegDecoderUnit *unit,
		AVRational *pkt_timebase, AVRational *ctx_timebase, AVRational *framerate, bool *error)
{
	static int s_dump_checked = 0;
	static FILE *s_dump_fp = NULL;
	if(!s_dump_checked)
	{
		s_dump_checked = 1;
		const char *dump_path = getenv("CHIAKI_DUMP_HEVC");
		if(dump_path && dump_path[0])
		{
			s_dump_fp = fopen(dump_path, "wb");
			if(s_dump_fp)
				CHIAKI_LOGI(decoder->log, "Dumping raw Annex-B stream to %s", dump_path);
		}
	}
	if(s_dump_fp && unit->packet && unit->packet->data && unit->packet->size > 0)
	{
		fwrite(unit->packet->data, 1, (size_t)unit->packet->size, s_dump_fp);
		fflush(s_dump_fp);
	}
	if(unit->parameter_sets)
	{
		av_packet_free(&decoder->vt_last_header);
		decoder->vt_last_header = av_packet_clone(unit->packet);
	}
	ChiakiVtDecodeResult r = chiaki_vt_decoder_decode(decoder->vt_decoder, unit->packet->data, (size_t)unit->packet->size,
			unit->packet->pts, unit->packet->duration);
	if(r == CHIAKI_VT_DECODE_OK)
		return NULL;
	if(r == CHIAKI_VT_DECODE_ERROR)
	{
		*error = true;
		return NULL;
	}

	CHIAKI_LOGE(decoder->log, "Native VideoToolbox decoder failed, falling back to FFmpeg's videotoolbox hwaccel");
	chiaki_vt_decoder_free(decoder->vt_decoder);
	decoder->vt_decoder = NULL;
	chiaki_mutex_lock(&decoder->mutex);
	bool fallback_open = ffmpeg_decoder_open_avcodec(decoder);
	chiaki_mutex_unlock(&decoder->mutex);
	if(!fallback_open)
	{
		av_packet_free(&decoder->vt_last_header);
		*error = true;
		return NULL;
	}
	if(decoder->vt_last_header && !unit->parameter_sets)
	{
		// avcodec has not seen the stream header yet
		bool header_error = false;
		AVFrame *header_frame = ffmpeg_decoder_decode_packet(decoder, decoder->vt_last_header,
				pkt_timebase, ctx_timebase, framerate, &header_error);
		av_frame_free(&header_frame);
	}
	av_packet_free(&decoder->vt_last_header);
	AVFrame *frame = ffmpeg_decoder_decode_packet(decoder, unit->packet, pkt_timebase, ctx_timebase, framerate, error);
	// avcodec has none of the reference frames, start from an IDR
	if(!unit->parameter_sets)
		*error = true;
	return frame;
}
#endif

static void *ffmpeg_decoder_thread_func(void *user)
{
	ChiakiFfmpegDecoder *decoder = user;
	chiaki_thread_set_affinity(CHIAKI_THREAD_NAME_DECODER);

	chiaki_mutex_lock(&decoder->queue_mutex);
	while(true)
	{
		while(!decoder->decode_thread_stop && decoder->queue_count == 0)
			chiaki_cond_wait(&decoder->queue_cond, &decoder->queue_mutex);
		if(decoder->decode_thread_stop)
			break;

		ChiakiFfmpegDecoderUnit unit = decoder->queue[decoder->queue_head];
		decoder->queue[decoder->queue_head].packet = NULL;
		decoder->queue_head = (decoder->queue_head + 1) % CHIAKI_FFMPEG_DECODER_QUEUE_CAPACITY;
		decoder->queue_count--;
		chiaki_mutex_unlock(&decoder->queue_mutex);

		AVRational pkt_timebase, ctx_timebase, framerate;
		bool error = false;
		AVFrame *frame;
#ifdef CHIAKI_FFMPEG_DECODER_NATIVE_VT
		if(decoder->vt_decoder)
			frame = ffmpeg_decoder_vt_decode_unit(decoder, &unit, &pkt_timebase, &ctx_timebase, &framerate, &error);
		else
#endif
		frame = ffmpeg_decoder_decode_packet(decoder, unit.packet, &pkt_timebase, &ctx_timebase, &framerate, &error);
		av_packet_free(&unit.packet);

		chiaki_mutex_lock(&decoder->queue_mutex);
		// Parameter-set-only units (stream headers) are rejected by some decoders
		// without any harm, don't turn that into an IDR request.
		if(error && !unit.parameter_sets)
			decoder->decode_error = true;
		if(unit.recovered)
			decoder->frame_recovered = true;
		if(!frame)
			continue;
		av_frame_free(&decoder->latest_frame);
		decoder->latest_frame = frame;
		decoder->frame_pkt_timebase = pkt_timebase;
		decoder->frame_ctx_timebase = ctx_timebase;
		decoder->frame_framerate = framerate;
		ChiakiLatencySummary latency;
		bool log_latency = ffmpeg_decoder_latency_note_frame(decoder, frame, &latency);
		if(decoder->decode_thread_stop)
			break;
		chiaki_mutex_unlock(&decoder->queue_mutex);
		if(log_latency)
			ffmpeg_decoder_latency_log(decoder, &latency);
		decoder->frame_available_cb(decoder, decoder->frame_available_cb_user);
		chiaki_mutex_lock(&decoder->queue_mutex);
	}
	chiaki_mutex_unlock(&decoder->queue_mutex);
	return NULL;
}

CHIAKI_EXPORT ChiakiFfmpegFrame chiaki_ffmpeg_decoder_pull_frame(ChiakiFfmpegDecoder *decoder, int32_t *frames_lost)
{
	// Only takes the newest frame stored by the decode thread, never waits for avcodec.
	chiaki_mutex_lock(&decoder->queue_mutex);
	double synthetic_duration = decoder->synthetic_frame_duration_us / 1000000.0;
	AVFrame *frame = decoder->latest_frame;
	decoder->latest_frame = NULL;
	*frames_lost = decoder->frames_lost;
	bool recovered = false;
	if(frame && decoder->frame_recovered)
	{
		recovered = true;
		decoder->frame_recovered = false;
		frame->decode_error_flags |= 1;
	}
	decoder->frames_lost = 0;
	AVRational pkt_timebase = decoder->frame_pkt_timebase;
	AVRational ctx_timebase = decoder->frame_ctx_timebase;
	AVRational framerate = decoder->frame_framerate;
	uint64_t first_packet_us = decoder->latest_frame_first_packet_us;
	uint64_t decoded_us = decoder->latest_frame_decoded_us;
	chiaki_mutex_unlock(&decoder->queue_mutex);

	ChiakiFfmpegFrame frame_plus_stats = {};
	frame_plus_stats.frame = frame;
	frame_plus_stats.recovered = recovered;
	if(frame)
	{
		frame_plus_stats.first_packet_us = first_packet_us;
		frame_plus_stats.decoded_us = decoded_us;
	}
	if(frame)
	{
		chiaki_ffmpeg_frame_get_timing(
			frame,
			pkt_timebase,
			ctx_timebase,
			framerate,
			&frame_plus_stats.pts,
			&frame_plus_stats.duration);
		if(frame->duration <= 0)
			frame_plus_stats.duration = synthetic_duration;
	}

	return frame_plus_stats;
}

CHIAKI_EXPORT void chiaki_ffmpeg_frame_get_timing(
	AVFrame *frame,
	AVRational pkt_timebase,
	AVRational ctx_timebase,
	AVRational framerate,
	double *out_pts,
	double *out_duration)
{
	AVRational time_base = pkt_timebase;
	if(time_base.num <= 0 || time_base.den <= 0)
		time_base = ctx_timebase;
	if(time_base.num <= 0 || time_base.den <= 0)
		time_base = (AVRational){1, 1000000};

	int64_t best_effort_pts = frame->best_effort_timestamp;
	int64_t raw_pts = frame->pts;
	int64_t pts = best_effort_pts;
	if(pts == AV_NOPTS_VALUE)
		pts = raw_pts;
	if(pts == AV_NOPTS_VALUE)
		pts = 0;
	*out_pts = av_q2d(time_base) * (double)pts;

	if(frame->duration > 0)
	{
		*out_duration = av_q2d(time_base) * (double)frame->duration;
		return;
	}

	double fps = (framerate.num > 0 && framerate.den > 0) ? av_q2d(framerate) : 60.0;
	if(fps <= 0.0)
		fps = 60.0;
	*out_duration = 1.0 / fps;
}

CHIAKI_EXPORT enum AVPixelFormat chiaki_ffmpeg_decoder_get_pixel_format(ChiakiFfmpegDecoder *decoder)
{
	if (decoder->vt_decoder || decoder->hw_device_ctx) {
		return decoder->hdr_enabled
			? AV_PIX_FMT_P010LE
			: AV_PIX_FMT_NV12;
	} else {
		return decoder->hdr_enabled
			? AV_PIX_FMT_YUV420P10LE
			: AV_PIX_FMT_YUV420P;
	}
}
