// SPDX-License-Identifier: LicenseRef-AGPL-3.0-only-OpenSSL

#ifndef CHIAKI_LATENCYSTATS_H
#define CHIAKI_LATENCYSTATS_H

#include <stdint.h>
#include <stdlib.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Fixed-size per-frame latency samples for the [latency] log lines. Adding a sample
 * never allocates; percentiles are computed only when a window is summarized.
 * Not thread-safe, callers lock if they need to.
 */
#define CHIAKI_LATENCY_STATS_CAPACITY 2048
#define CHIAKI_LATENCY_STATS_INTERVAL_US 5000000

typedef struct chiaki_latency_stats_t
{
	uint32_t samples_us[CHIAKI_LATENCY_STATS_CAPACITY];
	uint32_t stored; // samples in samples_us
	uint32_t count; // all samples of the window, including those that did not fit
	uint32_t max_us;
} ChiakiLatencyStats;

typedef struct chiaki_latency_summary_t
{
	double p50_ms;
	double p99_ms;
	double max_ms;
	uint32_t frames;
} ChiakiLatencySummary;

static inline void chiaki_latency_stats_add(ChiakiLatencyStats *stats, uint64_t us)
{
	uint32_t v = us > UINT32_MAX ? UINT32_MAX : (uint32_t)us;
	if(stats->stored < CHIAKI_LATENCY_STATS_CAPACITY)
		stats->samples_us[stats->stored++] = v;
	stats->count++;
	if(v > stats->max_us)
		stats->max_us = v;
}

static inline int chiaki_latency_stats_cmp(const void *a, const void *b)
{
	uint32_t x = *(const uint32_t *)a, y = *(const uint32_t *)b;
	return (x > y) - (x < y);
}

/**
 * Compute nearest-rank p50/p99/max of the window and start a new one.
 */
static inline ChiakiLatencySummary chiaki_latency_stats_summarize(ChiakiLatencyStats *stats)
{
	ChiakiLatencySummary summary = { 0.0, 0.0, 0.0, stats->count };
	if(stats->stored > 0)
	{
		qsort(stats->samples_us, stats->stored, sizeof(stats->samples_us[0]), chiaki_latency_stats_cmp);
		summary.p50_ms = stats->samples_us[(stats->stored - 1) * 50 / 100] / 1000.0;
		summary.p99_ms = stats->samples_us[(stats->stored - 1) * 99 / 100] / 1000.0;
		summary.max_ms = stats->max_us / 1000.0;
	}
	stats->stored = 0;
	stats->count = 0;
	stats->max_us = 0;
	return summary;
}

#ifdef __cplusplus
}
#endif

#endif // CHIAKI_LATENCYSTATS_H
