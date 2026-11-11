#include "macActivity.h"

#import <Foundation/Foundation.h>

#include <map>
#include <mutex>

static std::mutex activities_mutex;
static std::map<const void *, id<NSObject>> activities;

void macBeginStreamingActivity(const void *owner)
{
	std::lock_guard<std::mutex> lock(activities_mutex);
	if(activities.count(owner))
		return;
	@autoreleasepool {
		id<NSObject> token = [[NSProcessInfo processInfo]
			beginActivityWithOptions:NSActivityUserInitiated | NSActivityLatencyCritical
			reason:@"Remote Play streaming"];
		if(!token)
			return;
#if !__has_feature(objc_arc)
		[token retain];
#endif
		activities[owner] = token;
	}
}

void macEndStreamingActivity(const void *owner)
{
	std::lock_guard<std::mutex> lock(activities_mutex);
	auto it = activities.find(owner);
	if(it == activities.end())
		return;
	@autoreleasepool {
		[[NSProcessInfo processInfo] endActivity:it->second];
#if !__has_feature(objc_arc)
		[it->second release];
#endif
	}
	activities.erase(it);
}
