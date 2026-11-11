#include "macAudioOutput.h"

#import <AudioToolbox/AudioToolbox.h>
#import <CoreAudio/CoreAudio.h>
#include <dispatch/dispatch.h>

#include <cmath>
#include <string>
#include <vector>

#if !__has_feature(objc_arc)
#error "macAudioOutput.mm must be compiled with -fobjc-arc"
#endif

namespace
{

// IO buffer asked of the device, scaled to its rate: 128 frames = 2.7 ms at 48 kHz
constexpr double kPreferredIOFrames48k = 128.0;
constexpr int64_t kStatsIntervalNs = 5 * (int64_t)NSEC_PER_SEC;
// Default device and rate changes tend to arrive as a burst of notifications; restart once after it
constexpr int64_t kRestartDebounceNs = 100 * (int64_t)NSEC_PER_MSEC;

AudioObjectPropertyAddress Addr(AudioObjectPropertySelector selector, AudioObjectPropertyScope scope = kAudioObjectPropertyScopeGlobal)
{
	return { selector, scope, kAudioObjectPropertyElementMain };
}

template<typename T>
bool GetProp(AudioObjectID object, AudioObjectPropertySelector selector, AudioObjectPropertyScope scope, T &out)
{
	AudioObjectPropertyAddress addr = Addr(selector, scope);
	UInt32 size = sizeof(T);
	return AudioObjectGetPropertyData(object, &addr, 0, nullptr, &size, &out) == noErr && size == sizeof(T);
}

std::string DeviceName(AudioObjectID device)
{
	CFStringRef name = nullptr;
	if(!GetProp(device, kAudioObjectPropertyName, kAudioObjectPropertyScopeGlobal, name) || !name)
		return "unknown";
	char buf[256];
	std::string result = CFStringGetCString(name, buf, sizeof(buf), kCFStringEncodingUTF8) ? buf : "unknown";
	CFRelease(name);
	return result;
}

std::vector<AudioObjectID> OutputStreams(AudioObjectID device)
{
	AudioObjectPropertyAddress addr = Addr(kAudioDevicePropertyStreams, kAudioObjectPropertyScopeOutput);
	UInt32 size = 0;
	if(AudioObjectGetPropertyDataSize(device, &addr, 0, nullptr, &size) != noErr || size == 0)
		return {};
	std::vector<AudioObjectID> streams(size / sizeof(AudioObjectID));
	if(AudioObjectGetPropertyData(device, &addr, 0, nullptr, &size, streams.data()) != noErr)
		return {};
	streams.resize(size / sizeof(AudioObjectID));
	return streams;
}

AudioObjectID DefaultOutputDevice()
{
	AudioObjectID device = kAudioObjectUnknown;
	if(!GetProp(kAudioObjectSystemObject, kAudioHardwarePropertyDefaultOutputDevice, kAudioObjectPropertyScopeGlobal, device))
		return kAudioObjectUnknown;
	return device;
}

// Matches the names SDL lists in the audio output setting (kAudioObjectPropertyName)
AudioObjectID FindOutputDevice(const std::string &name)
{
	AudioObjectPropertyAddress addr = Addr(kAudioHardwarePropertyDevices);
	UInt32 size = 0;
	if(AudioObjectGetPropertyDataSize(kAudioObjectSystemObject, &addr, 0, nullptr, &size) != noErr || size == 0)
		return kAudioObjectUnknown;
	std::vector<AudioObjectID> devices(size / sizeof(AudioObjectID));
	if(AudioObjectGetPropertyData(kAudioObjectSystemObject, &addr, 0, nullptr, &size, devices.data()) != noErr)
		return kAudioObjectUnknown;
	devices.resize(size / sizeof(AudioObjectID));
	for(AudioObjectID device : devices)
		if(!OutputStreams(device).empty() && DeviceName(device) == name)
			return device;
	return kAudioObjectUnknown;
}

struct DeviceLatency
{
	Float64 rate = 0;
	UInt32 buffer_frames = 0;
	UInt32 device_frames = 0;
	UInt32 safety_frames = 0;
	UInt32 stream_frames = 0;

	double TotalMs() const
	{
		if(rate <= 0)
			return 0.0;
		return (double)(device_frames + safety_frames + buffer_frames + stream_frames) * 1000.0 / rate;
	}

	static DeviceLatency Query(AudioObjectID device)
	{
		DeviceLatency l;
		GetProp(device, kAudioDevicePropertyNominalSampleRate, kAudioObjectPropertyScopeGlobal, l.rate);
		GetProp(device, kAudioDevicePropertyBufferFrameSize, kAudioObjectPropertyScopeGlobal, l.buffer_frames);
		GetProp(device, kAudioDevicePropertyLatency, kAudioObjectPropertyScopeOutput, l.device_frames);
		GetProp(device, kAudioDevicePropertySafetyOffset, kAudioObjectPropertyScopeOutput, l.safety_frames);
		std::vector<AudioObjectID> streams = OutputStreams(device);
		if(!streams.empty())
			GetProp(streams[0], kAudioStreamPropertyLatency, kAudioObjectPropertyScopeGlobal, l.stream_frames);
		return l;
	}
};

// CoreAudio IO thread. Everything it touches is the jitter buffer's lock-free consumer side.
OSStatus RenderProc(void *ref, AudioUnitRenderActionFlags *, const AudioTimeStamp *, UInt32, UInt32 frames,
	AudioBufferList *io) noexcept CA_REALTIME_API
{
	auto *jb = static_cast<MacAudioJitterBuffer *>(ref);
	if(!io || io->mNumberBuffers < 1 || !io->mBuffers[0].mData)
		return noErr;
	const UInt32 frame_bytes = (UInt32)sizeof(float) * jb->Config().channels;
	const UInt32 capacity = io->mBuffers[0].mDataByteSize / frame_bytes;
	jb->Render(static_cast<float *>(io->mBuffers[0].mData), frames < capacity ? frames : capacity);
	return noErr;
}

}

struct MacAudioOutput::Impl
{
	ChiakiLog *log = nullptr;
	MacAudioJitterBuffer jb;
	unsigned int channels = 2;
	unsigned int rate = 48000;
	std::string requested_name;
	bool follow_default = true;
	std::weak_ptr<Impl> self;

	// Everything below is only touched on `queue`
	dispatch_queue_t queue = nil;
	dispatch_source_t stats_timer = nil;
	AudioComponentInstance unit = nullptr;
	AudioObjectID device = kAudioObjectUnknown;
	DeviceLatency latency;
	bool stopped = false;
	bool restart_pending = false;
	uint64_t logged_overflow = 0;
	AudioObjectPropertyListenerBlock default_listener = nil;
	AudioObjectPropertyListenerBlock device_listener = nil;
	AudioObjectID device_listener_object = kAudioObjectUnknown;

	bool Start()
	{
		std::weak_ptr<Impl> weak = self;
		default_listener = ^(UInt32, const AudioObjectPropertyAddress *) {
			std::shared_ptr<Impl> s = weak.lock();
			if(!s || s->stopped || !s->follow_default)
				return;
			if(DefaultOutputDevice() != s->device)
				s->ScheduleRestart("default output device change");
		};
		AudioObjectPropertyAddress addr = Addr(kAudioHardwarePropertyDefaultOutputDevice);
		OSStatus err = AudioObjectAddPropertyListenerBlock(kAudioObjectSystemObject, &addr, queue, default_listener);
		if(err != noErr)
		{
			CHIAKI_LOGW(log, "[audio] Can't listen for default output device changes (%d)", (int)err);
			default_listener = nil;
		}

		if(!Open())
		{
			RemoveDefaultListener();
			return false;
		}

		stats_timer = dispatch_source_create(DISPATCH_SOURCE_TYPE_TIMER, 0, 0, queue);
		dispatch_source_set_timer(stats_timer, dispatch_time(DISPATCH_TIME_NOW, kStatsIntervalNs), kStatsIntervalNs, 100 * NSEC_PER_MSEC);
		dispatch_source_set_event_handler(stats_timer, ^{
			std::shared_ptr<Impl> s = weak.lock();
			if(s && !s->stopped)
				s->LogStats();
		});
		dispatch_resume(stats_timer);
		return true;
	}

	void Shutdown()
	{
		if(stopped)
			return;
		stopped = true;
		if(stats_timer)
		{
			dispatch_source_cancel(stats_timer);
			stats_timer = nil;
		}
		RemoveDefaultListener();
		Close();
	}

	void RemoveDefaultListener()
	{
		if(!default_listener)
			return;
		AudioObjectPropertyAddress addr = Addr(kAudioHardwarePropertyDefaultOutputDevice);
		AudioObjectRemovePropertyListenerBlock(kAudioObjectSystemObject, &addr, queue, default_listener);
		default_listener = nil;
	}

	void AddDeviceListener()
	{
		std::weak_ptr<Impl> weak = self;
		device_listener = ^(UInt32 count, const AudioObjectPropertyAddress *addrs) {
			std::shared_ptr<Impl> s = weak.lock();
			if(!s || s->stopped)
				return;
			for(UInt32 i = 0; i < count; i++)
			{
				if(addrs[i].mSelector == kAudioDevicePropertyDeviceIsAlive)
				{
					UInt32 alive = 1;
					if(!GetProp(s->device, kAudioDevicePropertyDeviceIsAlive, kAudioObjectPropertyScopeGlobal, alive) || !alive)
					{
						// A specifically chosen device went away: carry on with the default one
						s->follow_default = true;
						s->ScheduleRestart("output device removal");
					}
				}
				else if(addrs[i].mSelector == kAudioDevicePropertyNominalSampleRate)
				{
					Float64 new_rate = 0;
					if(GetProp(s->device, kAudioDevicePropertyNominalSampleRate, kAudioObjectPropertyScopeGlobal, new_rate)
						&& new_rate != s->latency.rate)
						s->ScheduleRestart("output sample rate change");
				}
			}
		};
		device_listener_object = device;
		const AudioObjectPropertySelector selectors[] = { kAudioDevicePropertyDeviceIsAlive, kAudioDevicePropertyNominalSampleRate };
		for(AudioObjectPropertySelector selector : selectors)
		{
			AudioObjectPropertyAddress addr = Addr(selector);
			OSStatus err = AudioObjectAddPropertyListenerBlock(device, &addr, queue, device_listener);
			if(err != noErr)
				CHIAKI_LOGW(log, "[audio] Can't listen for output device property '%c%c%c%c' (%d)",
					(char)(selector >> 24), (char)(selector >> 16), (char)(selector >> 8), (char)selector, (int)err);
		}
	}

	void RemoveDeviceListener()
	{
		if(!device_listener)
			return;
		const AudioObjectPropertySelector selectors[] = { kAudioDevicePropertyDeviceIsAlive, kAudioDevicePropertyNominalSampleRate };
		for(AudioObjectPropertySelector selector : selectors)
		{
			AudioObjectPropertyAddress addr = Addr(selector);
			AudioObjectRemovePropertyListenerBlock(device_listener_object, &addr, queue, device_listener);
		}
		device_listener = nil;
		device_listener_object = kAudioObjectUnknown;
	}

	bool Open()
	{
		AudioObjectID dev = kAudioObjectUnknown;
		if(!follow_default)
		{
			dev = FindOutputDevice(requested_name);
			if(dev == kAudioObjectUnknown)
			{
				CHIAKI_LOGW(log, "[audio] Output device '%s' not found, using the default output device", requested_name.c_str());
				follow_default = true;
			}
		}
		if(follow_default)
			dev = DefaultOutputDevice();
		if(dev == kAudioObjectUnknown)
		{
			CHIAKI_LOGE(log, "[audio] No default output device");
			return false;
		}
		const std::string name = DeviceName(dev);

		AudioComponentDescription desc = {};
		desc.componentType = kAudioUnitType_Output;
		desc.componentSubType = kAudioUnitSubType_HALOutput;
		desc.componentManufacturer = kAudioUnitManufacturer_Apple;
		AudioComponent component = AudioComponentFindNext(nullptr, &desc);
		if(!component)
		{
			CHIAKI_LOGE(log, "[audio] HAL output audio unit not found");
			return false;
		}
		OSStatus err = AudioComponentInstanceNew(component, &unit);
		if(err != noErr || !unit)
		{
			CHIAKI_LOGE(log, "[audio] Can't create HAL output unit (%d)", (int)err);
			unit = nullptr;
			return false;
		}

#define CHECK_AU(call, what) \
		do { \
			err = (call); \
			if(err != noErr) \
			{ \
				CHIAKI_LOGE(log, "[audio] %s failed on '%s' (%d)", what, name.c_str(), (int)err); \
				Close(); \
				return false; \
			} \
		} while(0)

		CHECK_AU(AudioUnitSetProperty(unit, kAudioOutputUnitProperty_CurrentDevice, kAudioUnitScope_Global, 0, &dev, sizeof(dev)),
			"Selecting the output device");
		device = dev;

		// Small IO buffer, within what the device allows
		Float64 device_rate = 0;
		GetProp(dev, kAudioDevicePropertyNominalSampleRate, kAudioObjectPropertyScopeGlobal, device_rate);
		AudioValueRange range = {};
		GetProp(dev, kAudioDevicePropertyBufferFrameSizeRange, kAudioObjectPropertyScopeGlobal, range);
		double want = kPreferredIOFrames48k * (device_rate > 0 ? device_rate : 48000.0) / 48000.0;
		if(range.mMaximum > 0)
			want = std::min(std::max(want, std::ceil(range.mMinimum)), std::floor(range.mMaximum));
		UInt32 requested_frames = (UInt32)std::lround(want);
		err = AudioUnitSetProperty(unit, kAudioDevicePropertyBufferFrameSize, kAudioUnitScope_Global, 0, &requested_frames, sizeof(requested_frames));
		if(err != noErr)
			CHIAKI_LOGW(log, "[audio] Can't set IO buffer to %u frames on '%s' (%d)", requested_frames, name.c_str(), (int)err);

		// What we feed in: interleaved float at the stream's rate; the unit converts to the device format
		AudioStreamBasicDescription fmt = {};
		fmt.mSampleRate = rate;
		fmt.mFormatID = kAudioFormatLinearPCM;
		fmt.mFormatFlags = kAudioFormatFlagIsFloat | kAudioFormatFlagIsPacked;
		fmt.mFramesPerPacket = 1;
		fmt.mChannelsPerFrame = channels;
		fmt.mBitsPerChannel = 32;
		fmt.mBytesPerFrame = (UInt32)sizeof(float) * channels;
		fmt.mBytesPerPacket = fmt.mBytesPerFrame;
		CHECK_AU(AudioUnitSetProperty(unit, kAudioUnitProperty_StreamFormat, kAudioUnitScope_Input, 0, &fmt, sizeof(fmt)),
			"Setting the input stream format");

		AURenderCallbackStruct callback = {};
		callback.inputProc = RenderProc;
		callback.inputProcRefCon = &jb;
		CHECK_AU(AudioUnitSetProperty(unit, kAudioUnitProperty_SetRenderCallback, kAudioUnitScope_Input, 0, &callback, sizeof(callback)),
			"Setting the render callback");

		CHECK_AU(AudioUnitInitialize(unit), "Initializing the output unit");

		latency = DeviceLatency::Query(dev);
		// Unit isn't running, so this thread is the only consumer: start over with an empty queue
		jb.ConsumerReset();

#ifndef MAC_AUDIO_OUTPUT_NO_START
		CHECK_AU(AudioOutputUnitStart(unit), "Starting the output unit");
#endif
#undef CHECK_AU

		AddDeviceListener();

		CHIAKI_LOGI(log, "[audio] CoreAudio output on '%s'%s: device %.0f Hz, IO buffer %u frames (requested %u, device range %.0f-%.0f), "
			"input %u ch @ %u Hz, jitter target %.1f ms",
			name.c_str(), follow_default ? " (system default)" : "", latency.rate, latency.buffer_frames, requested_frames,
			range.mMinimum, range.mMaximum, channels, rate, jb.TargetMs());
		CHIAKI_LOGI(log, "[audio] Output latency %.1f ms = device %u + safety offset %u + IO buffer %u + stream %u frames @ %.0f Hz",
			latency.TotalMs(), latency.device_frames, latency.safety_frames, latency.buffer_frames, latency.stream_frames, latency.rate);
		return true;
	}

	void Close()
	{
		RemoveDeviceListener();
		if(!unit)
			return;
		// Synchronous: no render callback runs after this returns
		AudioOutputUnitStop(unit);
		AudioUnitUninitialize(unit);
		AudioComponentInstanceDispose(unit);
		unit = nullptr;
		device = kAudioObjectUnknown;
	}

	void ScheduleRestart(const char *why)
	{
		if(stopped || restart_pending)
			return;
		restart_pending = true;
		std::weak_ptr<Impl> weak = self;
		dispatch_after(dispatch_time(DISPATCH_TIME_NOW, kRestartDebounceNs), queue, ^{
			std::shared_ptr<Impl> s = weak.lock();
			if(!s)
				return;
			s->restart_pending = false;
			if(s->stopped)
				return;
			CHIAKI_LOGI(s->log, "[audio] Restarting CoreAudio output after %s", why);
			s->Close();
			if(!s->Open())
				CHIAKI_LOGE(s->log, "[audio] Could not reopen CoreAudio output after %s, audio stays silent until the next device change", why);
		});
	}

	void LogStats()
	{
		const MacAudioJitterBuffer::Stats stats = jb.TakeDepthStats();
		// Counters are totals since the output was created
		CHIAKI_LOGI(log, "[audio] queue_ms p50=%.1f max=%.1f underruns=%llu drops=%llu device_latency_ms=%.1f inserts=%llu target_ms=%.1f",
			stats.p50_ms, stats.max_ms, (unsigned long long)jb.Underruns(), (unsigned long long)jb.Drops(),
			unit ? latency.TotalMs() : 0.0, (unsigned long long)jb.Inserts(), jb.TargetMs());
		const uint64_t overflow = jb.OverflowFrames();
		if(overflow != logged_overflow)
		{
			CHIAKI_LOGW(log, "[audio] Output queue full, discarded %.1f ms of incoming audio",
				(double)(overflow - logged_overflow) * 1000.0 / rate);
			logged_overflow = overflow;
		}
	}
};

MacAudioOutput *MacAudioOutput::Create(ChiakiLog *log, unsigned int channels, unsigned int rate,
	unsigned int buffer_bytes, const char *device_name)
{
	auto impl = std::make_shared<Impl>();
	impl->self = impl;
	impl->log = log;
	impl->channels = channels;
	impl->rate = rate;
	if(device_name && *device_name && strcmp(device_name, "Auto") != 0)
	{
		impl->requested_name = device_name;
		impl->follow_default = false;
	}
	if(!impl->jb.Init(MacAudioJitterConfig::FromBufferBytes(channels, rate, buffer_bytes)))
	{
		CHIAKI_LOGE(log, "[audio] Unsupported output format: %u channels @ %u Hz", channels, rate);
		return nullptr;
	}

	dispatch_queue_attr_t attr = dispatch_queue_attr_make_with_qos_class(DISPATCH_QUEUE_SERIAL, QOS_CLASS_USER_INITIATED, 0);
	impl->queue = dispatch_queue_create("chiaki.audio.output", attr);
	__block bool ok = false;
	dispatch_sync(impl->queue, ^{
		ok = impl->Start();
	});
	if(!ok)
	{
		dispatch_sync(impl->queue, ^{
			impl->Shutdown();
		});
		return nullptr;
	}
	return new MacAudioOutput(impl);
}

MacAudioOutput::MacAudioOutput(std::shared_ptr<Impl> impl)
	: impl(std::move(impl))
{
}

MacAudioOutput::~MacAudioOutput()
{
	Stop();
}

void MacAudioOutput::Push(const int16_t *samples, size_t frames)
{
	impl->jb.Push(samples, frames);
}

void MacAudioOutput::Stop()
{
	std::shared_ptr<Impl> i = impl;
	dispatch_sync(i->queue, ^{
		i->Shutdown();
	});
}
