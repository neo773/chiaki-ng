#ifndef MAC_GAME_CONTROLLER_H
#define MAC_GAME_CONTROLLER_H

#include <chiaki/controller.h>
#include <chiaki/log.h>

#include <functional>

// Controller classes GameController.framework input can replace SDL input for. Used as bit flags.
enum MacGameControllerClass : unsigned
{
	MacGameControllerClassNone = 0,
	MacGameControllerClassDualSense = 1 << 0, // DualSense and DualSense Edge (GCDualSenseGamepad)
	MacGameControllerClassDualShock4 = 1 << 1, // GCDualShockGamepad
};

/**
 * Event-driven controller input through GameController.framework.
 *
 * Each supported controller gets its handlerQueue set to one serial QOS_CLASS_USER_INTERACTIVE
 * dispatch queue, and its gamepad and motion valueChangedHandlers build a ChiakiControllerState there.
 * Whenever one changes, the combined state of all controllers whose class is in the merged mask is
 * handed to state_cb on that queue, so input never waits for the main thread or a timer.
 *
 * While the object exists, the Home (PS) button's system gesture is disabled so it reaches the console.
 * Create and destroy on the main thread. The destructor waits for any callback in flight.
 */
class MacGameControllerInput
{
	public:
		// Called on the input queue. buttons_changed is true if the combined buttons differ from the last call.
		using StateCallback = std::function<void(const ChiakiControllerState &state, bool buttons_changed)>;
		// Called on the main thread when a supported controller connects or disconnects.
		using DevicesCallback = std::function<void()>;
		// Called on the input queue; input is dropped while it returns false (app in background etc.).
		using InputAllowedCallback = std::function<bool()>;

		MacGameControllerInput(ChiakiLog *log, StateCallback state_cb, DevicesCallback devices_cb, InputAllowedCallback input_allowed_cb);
		~MacGameControllerInput();
		MacGameControllerInput(const MacGameControllerInput &) = delete;
		MacGameControllerInput &operator=(const MacGameControllerInput &) = delete;

		// Number of connected controllers of a class (main thread)
		int Count(MacGameControllerClass cls) const;
		// Which classes feed the combined state (any thread). Starts at none.
		void SetMergedClasses(unsigned mask);
		// Re-zero motion like Controller::resetMotionControls() (any thread)
		void ResetMotion();

	private:
		void *impl; // ChiakiGameControllerInput *, retained

};

#endif
