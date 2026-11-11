#include "macGameController.h"
#include "controllermanager.h" // PS_TOUCHPAD_MAXX / PS_TOUCHPAD_MAXY, same touchpad space as the SDL path

#include <chiaki/orientation.h>
#include <chiaki/time.h>

#import <Foundation/Foundation.h>
#import <GameController/GameController.h>

#include <cmath>
#include <map>

namespace
{

struct Pad
{
	MacGameControllerClass cls = MacGameControllerClassNone;
	ChiakiControllerState state;
	ChiakiOrientationTracker orientation_tracker;
	ChiakiAccelNewZero accel_zero;
	ChiakiAccelNewZero real_accel;
	uint32_t last_motion_timestamp = 0;
	int8_t touch_ids[2] = { -1, -1 };

	Pad(MacGameControllerClass cls = MacGameControllerClassNone) : cls(cls)
	{
		// Same initial state as Controller::Controller()
		chiaki_controller_state_set_idle(&state);
		chiaki_orientation_tracker_init(&orientation_tracker);
		chiaki_accel_new_zero_set_inactive(&accel_zero, false);
		chiaki_accel_new_zero_set_inactive(&real_accel, true);
	}
};

MacGameControllerClass ClassifyController(GCController *controller)
{
	GCExtendedGamepad *gamepad = controller.extendedGamepad;
	if(!gamepad)
		return MacGameControllerClassNone;
	if(@available(macOS 11.3, *))
	{
		if([gamepad isKindOfClass:[GCDualSenseGamepad class]])
			return MacGameControllerClassDualSense;
	}
	if(@available(macOS 11.0, *))
	{
		if([gamepad isKindOfClass:[GCDualShockGamepad class]])
			return MacGameControllerClassDualShock4;
	}
	return MacGameControllerClassNone;
}

int16_t AxisToS16(float v)
{
	v = std::fmax(-1.0f, std::fmin(1.0f, v));
	return (int16_t)std::lrintf(v * 32767.0f);
}

uint8_t TriggerToU8(float v)
{
	v = std::fmax(0.0f, std::fmin(1.0f, v));
	return (uint8_t)std::lrintf(v * 255.0f);
}

void SetHomeGestureState(GCController *controller, GCSystemGestureState gesture_state)
{
	if(@available(macOS 11.0, *))
	{
		GCControllerButtonInput *home = controller.physicalInputProfile.buttons[GCInputButtonHome];
		if(home)
			home.preferredSystemGestureState = gesture_state;
	}
}

}

@interface ChiakiGameControllerInput : NSObject
{
@public
	ChiakiLog *log;
	MacGameControllerInput::StateCallback state_cb;
	MacGameControllerInput::DevicesCallback devices_cb;
	MacGameControllerInput::InputAllowedCallback input_allowed_cb;
	dispatch_queue_t queue;

	// Main thread only
	NSMutableArray *controllers;
	NSMutableArray *observers;

	// Input queue only
	std::map<const void *, Pad> pads;
	unsigned merged_mask;
	uint32_t last_buttons;
	bool stopped;
}
@end

@implementation ChiakiGameControllerInput

- (instancetype)initWithLog:(ChiakiLog *)chiaki_log
{
	self = [super init];
	if(!self)
		return nil;
	log = chiaki_log;
	merged_mask = 0;
	last_buttons = 0;
	stopped = false;
	controllers = [[NSMutableArray alloc] init];
	observers = [[NSMutableArray alloc] init];
	dispatch_queue_attr_t attr = dispatch_queue_attr_make_with_qos_class(DISPATCH_QUEUE_SERIAL, QOS_CLASS_USER_INTERACTIVE, 0);
	queue = dispatch_queue_create("org.streetpea.chiaking.gamecontroller-input", attr);
	return self;
}

- (void)start
{
	if(@available(macOS 11.3, *))
		GCController.shouldMonitorBackgroundEvents = YES; // input_allowed_cb decides what to drop, as on the SDL path

	NSNotificationCenter *center = [NSNotificationCenter defaultCenter];
	NSOperationQueue *main_queue = [NSOperationQueue mainQueue];
	id connect_observer = [center addObserverForName:GCControllerDidConnectNotification object:nil queue:main_queue usingBlock:^(NSNotification *note) {
		[self attach:(GCController *)note.object];
	}];
	id disconnect_observer = [center addObserverForName:GCControllerDidDisconnectNotification object:nil queue:main_queue usingBlock:^(NSNotification *note) {
		[self detach:(GCController *)note.object];
	}];
	[observers addObject:connect_observer];
	[observers addObject:disconnect_observer];

	for(GCController *controller in [GCController controllers])
		[self attach:controller];

	if(@available(macOS 27.0, *))
	{
		// The PS button only reaches the app if the user's in-app Home button setting lets apps handle it
		GCControllerHomeButtonSettingsManager *manager = [[GCControllerHomeButtonSettingsManager alloc] initWithQueue:nil];
		NSError *error = nil;
		GCControllerHomeButtonSettingInAppAction action = [manager readControllerHomeButtonInAppActionWithError:&error];
		switch(action)
		{
			case GCControllerHomeButtonSettingInAppActionDefault:
				CHIAKI_LOGW(log, "GameController: the controller Home button keeps its system action in apps, so the PS button will not reach the console. "
					"Change it in System Settings > Game Controllers to let apps handle the Home button.");
				break;
			case GCControllerHomeButtonSettingInAppActionDefer:
			case GCControllerHomeButtonSettingInAppActionDisabled:
				CHIAKI_LOGI(log, "GameController: Home button in-app action lets the PS button reach the console");
				break;
			default:
				CHIAKI_LOGI(log, "GameController: could not read the Home button setting: %s",
					error ? error.localizedDescription.UTF8String : "unknown error");
				break;
		}
#if !__has_feature(objc_arc)
		[manager release];
#endif
	}
}

- (void)attach:(GCController *)controller
{
	if(!controller || [controllers containsObject:controller])
		return;
	MacGameControllerClass cls = ClassifyController(controller);
	if(cls == MacGameControllerClassNone)
		return;
	[controllers addObject:controller];

	const void *key = (__bridge const void *)controller;
	dispatch_async(queue, ^{
		if(!stopped)
			pads[key] = Pad(cls);
	});

	// All handlers below run on the input queue, after the Pad above was created
	controller.handlerQueue = queue;
	SetHomeGestureState(controller, GCSystemGestureStateDisabled);
	controller.extendedGamepad.valueChangedHandler = ^(GCExtendedGamepad *gamepad, GCControllerElement *element) {
		[self gamepadChanged:gamepad key:key];
	};
	GCMotion *motion = controller.motion;
	if(motion)
	{
		if(@available(macOS 11.0, *))
		{
			if(motion.sensorsRequireManualActivation)
				motion.sensorsActive = YES;
		}
		motion.valueChangedHandler = ^(GCMotion *m) {
			[self motionChanged:m key:key];
		};
	}

	CHIAKI_LOGI(log, "GameController: handling input of \"%s\" (%s) on a user-interactive queue",
		controller.vendorName ? controller.vendorName.UTF8String : "?",
		controller.productCategory ? controller.productCategory.UTF8String : "?");
	if(devices_cb)
		devices_cb();
}

- (void)releaseController:(GCController *)controller
{
	controller.extendedGamepad.valueChangedHandler = nil;
	GCMotion *motion = controller.motion;
	if(motion)
	{
		motion.valueChangedHandler = nil;
		if(@available(macOS 11.0, *))
		{
			if(motion.sensorsRequireManualActivation)
				motion.sensorsActive = NO;
		}
	}
	SetHomeGestureState(controller, GCSystemGestureStateEnabled);
	controller.handlerQueue = dispatch_get_main_queue();
}

- (void)detach:(GCController *)controller
{
	if(!controller || ![controllers containsObject:controller])
		return;
	[self releaseController:controller];
	const void *key = (__bridge const void *)controller;
	[controllers removeObject:controller];
	dispatch_async(queue, ^{
		if(stopped)
			return;
		const bool merged = pads.count(key) && (pads[key].cls & merged_mask);
		pads.erase(key);
		if(merged)
			[self emitState:true];
	});
	if(devices_cb)
		devices_cb();
}

- (int)count:(MacGameControllerClass)cls
{
	int n = 0;
	for(GCController *controller in controllers)
		if(ClassifyController(controller) == cls)
			n++;
	return n;
}

// Input queue
- (void)emitState:(bool)force
{
	if(stopped || !state_cb)
		return;
	ChiakiControllerState combined;
	chiaki_controller_state_set_idle(&combined);
	for(auto &it : pads)
	{
		if(it.second.cls & merged_mask)
			chiaki_controller_state_or(&combined, &combined, &it.second.state);
	}
	const bool buttons_changed = combined.buttons != last_buttons;
	last_buttons = combined.buttons;
	state_cb(combined, buttons_changed || force);
}

// Input queue
- (Pad *)padForKey:(const void *)key
{
	if(stopped)
		return nullptr;
	if(input_allowed_cb && !input_allowed_cb())
		return nullptr;
	auto it = pads.find(key);
	return it == pads.end() ? nullptr : &it->second;
}

// Input queue
- (void)updateTouch:(Pad *)pad index:(int)index dpad:(GCControllerDirectionPad *)dpad
{
	if(!dpad)
		return;
	const float x = dpad.xAxis.value;
	const float y = dpad.yAxis.value;
	int8_t &id = pad->touch_ids[index];
	// GameController reports a lifted finger as (0, 0); SDL's GameController backend reads it the same way
	if(x == 0.0f && y == 0.0f)
	{
		if(id >= 0)
		{
			chiaki_controller_state_stop_touch(&pad->state, (uint8_t)id);
			id = -1;
		}
		return;
	}
	const float x01 = std::fmax(0.0f, std::fmin(1.0f, (1.0f + x) * 0.5f));
	const float y01 = std::fmax(0.0f, std::fmin(1.0f, 1.0f - (1.0f + y) * 0.5f));
	const uint16_t px = (uint16_t)(x01 * PS_TOUCHPAD_MAXX);
	const uint16_t py = (uint16_t)(y01 * PS_TOUCHPAD_MAXY);
	if(id >= 0)
		chiaki_controller_state_set_touch_pos(&pad->state, (uint8_t)id, px, py);
	else
		id = chiaki_controller_state_start_touch(&pad->state, px, py);
}

// Input queue
- (void)gamepadChanged:(GCExtendedGamepad *)gamepad key:(const void *)key
{
	Pad *pad = [self padForKey:key];
	if(!pad)
		return;
	ChiakiControllerState &s = pad->state;

	// Positional mapping, as SDL's: A = cross, B = circle, X = square, Y = triangle,
	// Menu = Options, Options = Create, Home = PS
	uint32_t buttons = 0;
	if(gamepad.buttonA.pressed) buttons |= CHIAKI_CONTROLLER_BUTTON_CROSS;
	if(gamepad.buttonB.pressed) buttons |= CHIAKI_CONTROLLER_BUTTON_MOON;
	if(gamepad.buttonX.pressed) buttons |= CHIAKI_CONTROLLER_BUTTON_BOX;
	if(gamepad.buttonY.pressed) buttons |= CHIAKI_CONTROLLER_BUTTON_PYRAMID;
	if(gamepad.dpad.left.pressed) buttons |= CHIAKI_CONTROLLER_BUTTON_DPAD_LEFT;
	if(gamepad.dpad.right.pressed) buttons |= CHIAKI_CONTROLLER_BUTTON_DPAD_RIGHT;
	if(gamepad.dpad.up.pressed) buttons |= CHIAKI_CONTROLLER_BUTTON_DPAD_UP;
	if(gamepad.dpad.down.pressed) buttons |= CHIAKI_CONTROLLER_BUTTON_DPAD_DOWN;
	if(gamepad.leftShoulder.pressed) buttons |= CHIAKI_CONTROLLER_BUTTON_L1;
	if(gamepad.rightShoulder.pressed) buttons |= CHIAKI_CONTROLLER_BUTTON_R1;
	if(gamepad.leftThumbstickButton.pressed) buttons |= CHIAKI_CONTROLLER_BUTTON_L3;
	if(gamepad.rightThumbstickButton.pressed) buttons |= CHIAKI_CONTROLLER_BUTTON_R3;
	if(gamepad.buttonMenu.pressed) buttons |= CHIAKI_CONTROLLER_BUTTON_OPTIONS;
	if(gamepad.buttonOptions.pressed) buttons |= CHIAKI_CONTROLLER_BUTTON_SHARE;
	if(@available(macOS 11.0, *))
	{
		if(gamepad.buttonHome.pressed) buttons |= CHIAKI_CONTROLLER_BUTTON_PS;
	}

	GCControllerButtonInput *touchpad_button = nil;
	GCControllerDirectionPad *touch_primary = nil;
	GCControllerDirectionPad *touch_secondary = nil;
	if(@available(macOS 11.3, *))
	{
		if([gamepad isKindOfClass:[GCDualSenseGamepad class]])
		{
			GCDualSenseGamepad *ds = (GCDualSenseGamepad *)gamepad;
			touchpad_button = ds.touchpadButton;
			touch_primary = ds.touchpadPrimary;
			touch_secondary = ds.touchpadSecondary;
		}
	}
	if(@available(macOS 11.0, *))
	{
		if(!touchpad_button && [gamepad isKindOfClass:[GCDualShockGamepad class]])
		{
			GCDualShockGamepad *ds4 = (GCDualShockGamepad *)gamepad;
			touchpad_button = ds4.touchpadButton;
			touch_primary = ds4.touchpadPrimary;
			touch_secondary = ds4.touchpadSecondary;
		}
	}
	if(touchpad_button.pressed)
		buttons |= CHIAKI_CONTROLLER_BUTTON_TOUCHPAD;
	s.buttons = buttons;

	s.l2_state = TriggerToU8(gamepad.leftTrigger.value);
	s.r2_state = TriggerToU8(gamepad.rightTrigger.value);
	// GameController's y axes point up, SDL's (and chiaki's) point down
	s.left_x = AxisToS16(gamepad.leftThumbstick.xAxis.value);
	s.left_y = AxisToS16(-gamepad.leftThumbstick.yAxis.value);
	s.right_x = AxisToS16(gamepad.rightThumbstick.xAxis.value);
	s.right_y = AxisToS16(-gamepad.rightThumbstick.yAxis.value);

	[self updateTouch:pad index:0 dpad:touch_primary];
	[self updateTouch:pad index:1 dpad:touch_secondary];

	if(pad->cls & merged_mask)
		[self emitState:false];
}

// Input queue
- (void)motionChanged:(GCMotion *)motion key:(const void *)key
{
	Pad *pad = [self padForKey:key];
	if(!pad)
		return;
	if(@available(macOS 11.0, *))
	{
		if(!motion.hasRotationRate || !motion.hasGravityAndUserAcceleration)
			return;
	}
	// Same axis mapping SDL's GameController backend uses to match its HIDAPI sensor axes.
	// Rotation rate is rad/s and acceleration is in g, the units the SDL path feeds the tracker.
	const GCRotationRate rate = motion.rotationRate;
	const float gyro_x = (float)rate.x;
	const float gyro_y = (float)rate.z;
	const float gyro_z = (float)-rate.y;
	GCAcceleration accel = motion.gravity;
	if(@available(macOS 11.0, *))
		accel = motion.acceleration;
	else
	{
		accel.x += motion.userAcceleration.x;
		accel.y += motion.userAcceleration.y;
		accel.z += motion.userAcceleration.z;
	}
	const float accel_x = (float)-accel.x;
	const float accel_y = (float)-accel.y;
	const float accel_z = (float)-accel.z;

	const uint32_t timestamp_us = (uint32_t)chiaki_time_now_monotonic_us();
	chiaki_accel_new_zero_set_active(&pad->real_accel, accel_x, accel_y, accel_z, true);
	chiaki_orientation_tracker_update(&pad->orientation_tracker, gyro_x, gyro_y, gyro_z,
		accel_x, accel_y, accel_z, &pad->accel_zero, false, timestamp_us);
	pad->last_motion_timestamp = timestamp_us;
	chiaki_orientation_tracker_apply_to_controller_state(&pad->orientation_tracker, &pad->state);

	if(pad->cls & merged_mask)
		[self emitState:false];
}

- (void)setMergedMask:(unsigned)mask
{
	dispatch_async(queue, ^{
		if(stopped || merged_mask == mask)
			return;
		merged_mask = mask;
		[self emitState:true];
	});
}

- (void)resetMotion
{
	dispatch_async(queue, ^{
		if(stopped)
			return;
		// Same as Controller::resetMotionControls()
		for(auto &it : pads)
		{
			Pad &pad = it.second;
			chiaki_accel_new_zero_set_active(&pad.accel_zero, pad.real_accel.accel_x, pad.real_accel.accel_y, pad.real_accel.accel_z, false);
			chiaki_orientation_tracker_init(&pad.orientation_tracker);
			chiaki_orientation_tracker_update(&pad.orientation_tracker, pad.state.gyro_x, pad.state.gyro_y, pad.state.gyro_z,
				pad.real_accel.accel_x, pad.real_accel.accel_y, pad.real_accel.accel_z, &pad.accel_zero, false, pad.last_motion_timestamp);
			chiaki_orientation_tracker_apply_to_controller_state(&pad.orientation_tracker, &pad.state);
		}
		[self emitState:false];
	});
}

- (void)shutdown
{
	NSNotificationCenter *center = [NSNotificationCenter defaultCenter];
	for(id observer in observers)
		[center removeObserver:observer];
	[observers removeAllObjects];
	for(GCController *controller in controllers)
		[self releaseController:controller];
	[controllers removeAllObjects];
	// Wait for callbacks already queued; anything that still arrives sees stopped
	dispatch_sync(queue, ^{
		stopped = true;
		pads.clear();
		state_cb = nullptr;
		input_allowed_cb = nullptr;
	});
	devices_cb = nullptr;
}

#if !__has_feature(objc_arc)
- (void)dealloc
{
	[controllers release];
	[observers release];
	dispatch_release(queue);
	[super dealloc];
}
#endif

@end

MacGameControllerInput::MacGameControllerInput(ChiakiLog *log, StateCallback state_cb, DevicesCallback devices_cb, InputAllowedCallback input_allowed_cb)
{
	ChiakiGameControllerInput *input = [[ChiakiGameControllerInput alloc] initWithLog:log];
	input->state_cb = std::move(state_cb);
	input->devices_cb = std::move(devices_cb);
	input->input_allowed_cb = std::move(input_allowed_cb);
#if __has_feature(objc_arc)
	impl = (__bridge_retained void *)input;
#else
	impl = (void *)input;
#endif
	[input start];
}

MacGameControllerInput::~MacGameControllerInput()
{
#if __has_feature(objc_arc)
	ChiakiGameControllerInput *input = (__bridge_transfer ChiakiGameControllerInput *)impl;
	[input shutdown];
#else
	ChiakiGameControllerInput *input = (ChiakiGameControllerInput *)impl;
	[input shutdown];
	[input release];
#endif
	impl = nullptr;
}

int MacGameControllerInput::Count(MacGameControllerClass cls) const
{
	return [(__bridge ChiakiGameControllerInput *)impl count:cls];
}

void MacGameControllerInput::SetMergedClasses(unsigned mask)
{
	[(__bridge ChiakiGameControllerInput *)impl setMergedMask:mask];
}

void MacGameControllerInput::ResetMotion()
{
	[(__bridge ChiakiGameControllerInput *)impl resetMotion];
}
