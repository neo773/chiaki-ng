#ifndef MAC_ACTIVITY_H
#define MAC_ACTIVITY_H

// Holds an NSProcessInfo activity (NSActivityUserInitiated | NSActivityLatencyCritical) while a
// stream is running, so macOS doesn't App Nap the process or coalesce its timers.
// Both calls are idempotent per owner and safe to call from any thread.
void macBeginStreamingActivity(const void *owner);
void macEndStreamingActivity(const void *owner);

#endif
