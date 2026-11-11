#ifndef MAC_AUDIO_OUTPUT_H
#define MAC_AUDIO_OUTPUT_H

// Native low-latency audio output for macOS: an AUHAL output unit pulling from a lock-free
// single-producer/single-consumer ring.
//
// MacAudioJitterBuffer below is the platform independent core (ring, jitter regulation, the
// realtime render function) and is kept header-only and free of CoreAudio/Qt so it can be
// exercised offline. MacAudioOutput (macAudioOutput.mm) wraps it in the CoreAudio unit.

#include <algorithm>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <memory>

#if defined(__has_cpp_attribute)
#if __has_cpp_attribute(clang::nonblocking)
#define MAC_AUDIO_NONBLOCKING [[clang::nonblocking]]
#endif
#endif
#ifndef MAC_AUDIO_NONBLOCKING
#define MAC_AUDIO_NONBLOCKING
#endif

struct MacAudioJitterConfig
{
	uint32_t channels = 2;
	uint32_t rate = 48000;
	// Desired average queue depth between producer and render callback (the Audio Buffer Size setting)
	uint32_t target_frames = 480;
	// Crossfade at a drop or insert, fade-in after an underrun, fade-out tail into silence
	uint32_t fade_frames = 96;
	// Rendered audio over which the queue's low watermark is measured before correcting it
	uint32_t window_frames = 24000;
	// Ring size in frames, power of two
	uint32_t capacity_frames = 32768;
	// Already played audio kept for inserts, power of two
	uint32_t history_frames = 4096;

	// Settings in bytes of interleaved s16, as used by the SDL path
	static MacAudioJitterConfig FromBufferBytes(uint32_t channels, uint32_t rate, uint32_t buffer_bytes)
	{
		MacAudioJitterConfig c;
		c.channels = channels ? channels : 2;
		c.rate = rate ? rate : 48000;
		const uint32_t frame_bytes = c.channels * (uint32_t)sizeof(int16_t);
		const uint32_t target = buffer_bytes / frame_bytes;
		const uint32_t min_target = c.rate / 500;  // 2 ms
		const uint32_t max_target = c.rate / 5;    // 200 ms
		c.target_frames = std::min(std::max(target, min_target), max_target);
		c.fade_frames = c.rate / 500;               // 2 ms
		c.window_frames = c.rate / 2;               // 500 ms
		uint32_t cap = 1;
		while(cap < c.rate / 2 + 4 * c.target_frames) // >= 500 ms + room for big targets
			cap <<= 1;
		c.capacity_frames = cap;
		uint32_t hist = 1;
		while(hist < c.rate / 12) // >= 83 ms
			hist <<= 1;
		c.history_frames = hist;
		return c;
	}
};

class MacAudioJitterBuffer
{
public:
	static constexpr uint32_t kMaxChannels = 8;
	// Queue depth histogram for the periodic log, 0.5 ms buckets, last bucket collects everything above
	static constexpr uint32_t kHistBuckets = 512;

	MacAudioJitterBuffer() = default;
	MacAudioJitterBuffer(const MacAudioJitterBuffer &) = delete;
	MacAudioJitterBuffer &operator=(const MacAudioJitterBuffer &) = delete;

	// Not thread safe, call before producer and consumer start
	bool Init(const MacAudioJitterConfig &config)
	{
		auto pow2 = [](uint32_t v) { return v && !(v & (v - 1)); };
		if(config.channels == 0 || config.channels > kMaxChannels || config.rate == 0
			|| !pow2(config.capacity_frames) || !pow2(config.history_frames))
			return false;
		cfg = config;
		mask = cfg.capacity_frames - 1;
		hist_mask = cfg.history_frames - 1;
		buf.reset(new int16_t[(size_t)cfg.capacity_frames * cfg.channels]());
		hist.reset(new int16_t[(size_t)cfg.history_frames * cfg.channels]());
		write_pos.store(0, std::memory_order_relaxed);
		read_pos.store(0, std::memory_order_relaxed);
		packet_frames.store(0, std::memory_order_relaxed);
		for(auto &h : depth_hist)
			h.store(0, std::memory_order_relaxed);
		consumed = pos = hist_from = 0;
		ResetConsumerState();
		return true;
	}

	const MacAudioJitterConfig &Config() const { return cfg; }

	// ---- Producer (single thread) ----

	// Returns the number of frames accepted; the rest only happens with a stalled consumer and is counted
	size_t Push(const int16_t *frames, size_t count)
	{
		if(!buf || !count)
			return 0;
		const uint64_t w = write_pos.load(std::memory_order_relaxed);
		const uint64_t r = read_pos.load(std::memory_order_acquire);
		const uint64_t space = cfg.capacity_frames - (w - r);
		const size_t n = (size_t)std::min<uint64_t>(count, space);
		if(n < count)
			overflow_frames.fetch_add(count - n, std::memory_order_relaxed);
		const size_t ch = cfg.channels;
		const size_t start = (size_t)(w & mask);
		const size_t first = std::min(n, (size_t)cfg.capacity_frames - start);
		memcpy(buf.get() + start * ch, frames, first * ch * sizeof(int16_t));
		if(n > first)
			memcpy(buf.get(), frames + first * ch, (n - first) * ch * sizeof(int16_t));
		write_pos.store(w + n, std::memory_order_release);
		packet_frames.store((uint32_t)std::min<size_t>(count, UINT32_MAX), std::memory_order_relaxed);
		return n;
	}

	// ---- Consumer (single thread, normally the CoreAudio IO thread) ----

	// Fills frames * channels interleaved floats. Realtime safe: no locks, allocation or system calls.
	//
	// The play position `pos` normally equals `consumed` (what was taken out of the ring). An insert
	// moves it back into the history of already played frames; it then catches up with `consumed`
	// before the ring is read again. Queue depth is always write position - pos.
	void Render(float *out, uint32_t frames) noexcept MAC_AUDIO_NONBLOCKING
	{
		if(!buf || !out || !frames)
			return;
		const uint32_t ch = cfg.channels;
		const uint64_t w = write_pos.load(std::memory_order_acquire);
		uint64_t avail = w - pos;
		RecordDepth(avail);

		uint32_t packet = packet_frames.load(std::memory_order_relaxed);
		if(!packet)
			packet = cfg.target_frames;
		const uint64_t resume = ResumeFrames(packet, frames);
		// Low watermark the queue settles at when playback starts at `resume`
		const uint64_t keep = resume - packet - frames / 2;
		target_pub.store(resume - packet / 2, std::memory_order_relaxed);

		if(starved)
		{
			if(avail < resume)
			{
				RenderTail(out, 0, frames);
				return;
			}
			// Coming out of silence with a fade-in anyway, so anything a stall piled up beyond the
			// resume level is cut here without a crossfade
			if(avail > resume)
			{
				const uint64_t skip = avail - resume;
				pos += skip;
				avail = resume;
				Consume(pos);
				dropped_frames.fetch_add(skip, std::memory_order_relaxed);
				if(played)
					drops.fetch_add(1, std::memory_order_relaxed);
			}
			starved = false;
			played = true;
			fade_in_pos = 0;
			ResetWindow();
			prev_window_min = UINT64_MAX;
		}

		// Correct the queue's low watermark over the window towards `keep`: above keep + margin the
		// excess was never needed to ride out jitter and is pure latency (clock drift one way, bursts);
		// well below keep, the producer's clock is slower than the device's and an underrun is coming.
		// A huge burst is cut at once.
		uint64_t drop = 0, insert = 0;
		const uint64_t hard_limit = resume + (uint64_t)cfg.rate / 20; // + 50 ms
		if(avail > hard_limit)
			drop = avail - resume;
		else if(window_rendered >= cfg.window_frames)
		{
			// Only act when two windows in a row agree, so jitter spikes don't make it seesaw;
			// drift and standing backlog build up far slower than that
			if(window_min != UINT64_MAX && prev_window_min != UINT64_MAX)
			{
				const uint64_t lo = std::min(window_min, prev_window_min);
				const uint64_t hi = std::max(window_min, prev_window_min);
				if(lo > keep + MarginFrames())
					drop = lo - keep;
				else if(hi < keep / 2)
					insert = keep - hi;
			}
			prev_window_min = window_min;
			ResetWindow();
		}
		const uint32_t cross = std::min(cfg.fade_frames, frames);
		if(drop)
			drop = avail >= drop + frames ? drop : (avail > frames ? avail - frames : 0);
		if(insert)
		{
			const uint64_t oldest = std::max(hist_from, consumed > cfg.history_frames ? consumed - cfg.history_frames : 0);
			insert = std::min<uint64_t>(insert, pos > oldest ? pos - oldest : 0);
			if(avail < cross)
				insert = 0;
		}

		uint32_t done = 0;
		if(drop || insert)
		{
			// Crossfade from the audio that continues what was just played into the audio after the jump
			const uint64_t to = drop ? pos + drop : pos - insert;
			for(uint32_t i = 0; i < cross; i++)
			{
				const float wgt = (float)(i + 1) / (float)(cross + 1);
				const float gain = NextFadeInGain();
				const float tail = NextTailGain();
				for(uint32_t c = 0; c < ch; c++)
					out[i * ch + c] = (Get(pos + i, c) * (1.0f - wgt) + Get(to + i, c) * wgt) * gain + tail_value[c] * tail;
			}
			pos = to + cross;
			if(pos > consumed)
				Consume(pos);
			avail = w - pos;
			done = cross;
			if(drop)
			{
				drops.fetch_add(1, std::memory_order_relaxed);
				dropped_frames.fetch_add(drop, std::memory_order_relaxed);
			}
			else
			{
				inserts.fetch_add(1, std::memory_order_relaxed);
				inserted_frames.fetch_add(insert, std::memory_order_relaxed);
			}
			ResetWindow();
			prev_window_min = UINT64_MAX;
		}

		const uint32_t take = (uint32_t)std::min<uint64_t>(frames - done, avail);
		for(uint32_t i = 0; i < take; i++)
		{
			const float gain = NextFadeInGain();
			const float tail = NextTailGain();
			float *o = out + (size_t)(done + i) * ch;
			for(uint32_t c = 0; c < ch; c++)
				o[c] = Get(pos + i, c) * gain + tail_value[c] * tail;
		}
		pos += take;
		avail -= take;
		done += take;
		if(pos > consumed)
			Consume(pos);

		if(done < frames)
		{
			// Underrun: ramp from the last output value down to zero instead of stepping to silence
			if(done)
				for(uint32_t c = 0; c < ch; c++)
					tail_value[c] = out[(size_t)(done - 1) * ch + c];
			else
				for(uint32_t c = 0; c < ch; c++)
					tail_value[c] = last_out[c];
			tail_pos = 0;
			starved = true;
			underruns.fetch_add(1, std::memory_order_relaxed);
			extra_frames = std::min<uint64_t>(extra_frames + cfg.rate / 1000, (uint64_t)cfg.rate / 50); // +1 ms, max 20 ms
			since_underrun = 0;
			RenderTail(out, done, frames);
		}
		else
		{
			for(uint32_t c = 0; c < ch; c++)
				last_out[c] = out[(size_t)(frames - 1) * ch + c];
			window_min = std::min(window_min, avail);
			window_rendered += frames;
			since_underrun += frames;
			// Give back 1 ms of extra cushion after every 20 s without an underrun
			if(extra_frames && since_underrun >= (uint64_t)cfg.rate * 20)
			{
				extra_frames -= std::min<uint64_t>(extra_frames, cfg.rate / 1000);
				since_underrun = 0;
			}
		}
		read_pos.store(consumed, std::memory_order_release);
	}

	// Discards everything queued and starts over with a prebuffer. Consumer side: only call while
	// Render() can't run (output unit stopped).
	void ConsumerReset()
	{
		consumed = pos = hist_from = write_pos.load(std::memory_order_acquire);
		read_pos.store(consumed, std::memory_order_release);
		ResetConsumerState();
	}

	// ---- Stats (any thread) ----

	struct Stats
	{
		double p50_ms = 0.0;
		double max_ms = 0.0;
		uint64_t samples = 0;
	};

	// Queue depth distribution since the last call (seen at the start of each render cycle)
	Stats TakeDepthStats()
	{
		Stats s;
		uint32_t counts[kHistBuckets];
		for(uint32_t i = 0; i < kHistBuckets; i++)
		{
			counts[i] = depth_hist[i].exchange(0, std::memory_order_relaxed);
			s.samples += counts[i];
		}
		const uint64_t max_frames = depth_max.exchange(0, std::memory_order_relaxed);
		if(!s.samples)
			return s;
		uint64_t acc = 0;
		for(uint32_t i = 0; i < kHistBuckets; i++)
		{
			acc += counts[i];
			if(acc * 2 >= s.samples)
			{
				s.p50_ms = i * 0.5 + 0.25;
				break;
			}
		}
		s.max_ms = (double)max_frames * 1000.0 / cfg.rate;
		return s;
	}

	uint64_t Underruns() const { return underruns.load(std::memory_order_relaxed); }
	uint64_t Drops() const { return drops.load(std::memory_order_relaxed); }
	uint64_t DroppedFrames() const { return dropped_frames.load(std::memory_order_relaxed); }
	uint64_t Inserts() const { return inserts.load(std::memory_order_relaxed); }
	uint64_t InsertedFrames() const { return inserted_frames.load(std::memory_order_relaxed); }
	uint64_t OverflowFrames() const { return overflow_frames.load(std::memory_order_relaxed); }
	// Frames in the ring (not counting replayed history)
	uint64_t Depth() const { return write_pos.load(std::memory_order_relaxed) - read_pos.load(std::memory_order_relaxed); }
	// Average queue depth the regulator currently aims for, including cushion added after underruns
	double TargetMs() const
	{
		uint64_t target = target_pub.load(std::memory_order_relaxed);
		if(!target)
			target = cfg.target_frames;
		return (double)target * 1000.0 / cfg.rate;
	}

private:
	float Get(uint64_t p, uint32_t c) const noexcept MAC_AUDIO_NONBLOCKING
	{
		const int16_t s = p < consumed ? hist[(size_t)(p & hist_mask) * cfg.channels + c]
			: buf[(size_t)(p & mask) * cfg.channels + c];
		return (float)s * (1.0f / 32768.0f);
	}

	// Takes [consumed, upto) out of the ring, keeping the newest of it as history for inserts
	void Consume(uint64_t upto) noexcept MAC_AUDIO_NONBLOCKING
	{
		const uint32_t ch = cfg.channels;
		uint64_t q = std::max(consumed, upto > cfg.history_frames ? upto - cfg.history_frames : 0);
		for(; q < upto; q++)
		{
			const int16_t *src = &buf[(size_t)(q & mask) * ch];
			int16_t *dst = &hist[(size_t)(q & hist_mask) * ch];
			for(uint32_t c = 0; c < ch; c++)
				dst[c] = src[c];
		}
		consumed = upto;
	}

	// Queue depth at which playback (re)starts. Packets arrive in bursts of `packet` frames, so
	// starting at target + packet / 2 averages out at target. Callbacks take `frames` at a time,
	// so the queue must also cover one packet and one callback on top of a minimum cushion of
	// target / 4 (that wins for large callbacks). Plus whatever cushion underruns asked for.
	uint64_t ResumeFrames(uint32_t packet, uint32_t frames) const noexcept MAC_AUDIO_NONBLOCKING
	{
		const uint64_t t = cfg.target_frames;
		return std::max<uint64_t>(t + packet / 2, t / 4 + packet + frames) + extra_frames;
	}
	uint64_t MarginFrames() const noexcept MAC_AUDIO_NONBLOCKING
	{
		return std::max<uint64_t>(cfg.target_frames / 2, (uint64_t)cfg.rate * 3 / 1000); // >= 3 ms
	}

	float NextFadeInGain() noexcept MAC_AUDIO_NONBLOCKING
	{
		if(fade_in_pos >= cfg.fade_frames)
			return 1.0f;
		fade_in_pos++;
		return (float)fade_in_pos / (float)(cfg.fade_frames + 1);
	}

	float NextTailGain() noexcept MAC_AUDIO_NONBLOCKING
	{
		if(tail_pos >= cfg.fade_frames)
			return 0.0f;
		tail_pos++;
		return (float)(cfg.fade_frames + 1 - tail_pos) / (float)(cfg.fade_frames + 1);
	}

	void RenderTail(float *out, uint32_t from, uint32_t frames) noexcept MAC_AUDIO_NONBLOCKING
	{
		const uint32_t ch = cfg.channels;
		for(uint32_t i = from; i < frames; i++)
		{
			const float tail = NextTailGain();
			for(uint32_t c = 0; c < ch; c++)
				out[(size_t)i * ch + c] = tail_value[c] * tail;
		}
		for(uint32_t c = 0; c < ch; c++)
			last_out[c] = out[(size_t)(frames - 1) * ch + c];
		read_pos.store(consumed, std::memory_order_release);
	}

	void RecordDepth(uint64_t avail) noexcept MAC_AUDIO_NONBLOCKING
	{
		uint64_t bucket = avail * 2000 / cfg.rate;
		if(bucket >= kHistBuckets)
			bucket = kHistBuckets - 1;
		depth_hist[bucket].fetch_add(1, std::memory_order_relaxed);
		// Only this thread raises it; a reset racing with this store just loses one interval's maximum
		if(avail > depth_max.load(std::memory_order_relaxed))
			depth_max.store(avail, std::memory_order_relaxed);
	}

	void ResetWindow() noexcept MAC_AUDIO_NONBLOCKING
	{
		window_min = UINT64_MAX;
		window_rendered = 0;
	}

	void ResetConsumerState()
	{
		starved = true;
		played = false;
		fade_in_pos = cfg.fade_frames;
		tail_pos = cfg.fade_frames;
		for(uint32_t c = 0; c < kMaxChannels; c++)
		{
			tail_value[c] = 0.0f;
			last_out[c] = 0.0f;
		}
		since_underrun = 0;
		ResetWindow();
		prev_window_min = UINT64_MAX;
	}

	MacAudioJitterConfig cfg;
	uint64_t mask = 0;
	uint64_t hist_mask = 0;
	std::unique_ptr<int16_t[]> buf;

	alignas(64) std::atomic<uint64_t> write_pos{0};
	std::atomic<uint32_t> packet_frames{0};
	std::atomic<uint64_t> overflow_frames{0};

	alignas(64) std::atomic<uint64_t> read_pos{0};
	std::atomic<uint64_t> underruns{0};
	std::atomic<uint64_t> drops{0};
	std::atomic<uint64_t> dropped_frames{0};
	std::atomic<uint64_t> inserts{0};
	std::atomic<uint64_t> inserted_frames{0};
	std::atomic<uint64_t> depth_max{0};
	std::atomic<uint64_t> target_pub{0};
	std::atomic<uint32_t> depth_hist[kHistBuckets] = {};

	// Consumer-only state
	std::unique_ptr<int16_t[]> hist;
	uint64_t consumed = 0;  // ring frames taken (published as read_pos)
	uint64_t pos = 0;       // play position, <= consumed while replaying history
	uint64_t hist_from = 0; // history is valid from here (last reset)
	bool starved = true;
	bool played = false;
	uint32_t fade_in_pos = 0;
	uint32_t tail_pos = 0;
	float tail_value[kMaxChannels] = {};
	float last_out[kMaxChannels] = {};
	uint64_t window_min = UINT64_MAX;
	uint64_t prev_window_min = UINT64_MAX;
	uint64_t window_rendered = 0;
	uint64_t extra_frames = 0;
	uint64_t since_underrun = 0;
};

#if defined(__APPLE__)

#include <chiaki/log.h>

// AUHAL output on the default output device (or the one named in the settings), following default
// device and sample rate changes. Create() returns nullptr (after logging why) if the unit can't be
// set up, so the caller can fall back to SDL.
class MacAudioOutput
{
public:
	struct Impl;

	// device_name: nullptr, "" or "Auto" follow the system default output device
	static MacAudioOutput *Create(ChiakiLog *log, unsigned int channels, unsigned int rate,
		unsigned int buffer_bytes, const char *device_name);
	~MacAudioOutput();

	// Interleaved s16 at the rate/channels given to Create(). Single producer thread.
	void Push(const int16_t *samples, size_t frames);

	// Stops the unit, device listeners and the stats log; Push() stays safe to call afterwards.
	// Idempotent, call from any thread except the output's own queue.
	void Stop();

private:
	explicit MacAudioOutput(std::shared_ptr<Impl> impl);
	std::shared_ptr<Impl> impl;
};

#endif

#endif
