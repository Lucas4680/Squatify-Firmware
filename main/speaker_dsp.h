// Speaker processing for the DMA45-4 + DMA45-PR enclosure.
//
// Signal path, one mono sample at a time:
//
//     input (after volume)
//         |
//     80 Hz protection high-pass
//         |
//     response EQ + voicing shelf
//         |
//     split at 250 Hz ------------------------.
//         |                                   |
//     bass band                           mid/high band
//         |                                   |
//     speaker model predicts excursion        |
//     bass gain (look-ahead)                  |
//         |                                   |
//         '----------------- sum -------------'
//                             |
//                    look-ahead peak limiter
//                             |
//                     power-budget gain
//                             |
//                        safety clamp
//                             |
//                           output
//
// A second copy of the speaker model runs on the output. It reports the
// excursion, current and power the speaker is really being asked for and
// drives the power-budget gain.
//
// All settings are in speaker_config.h.

#pragma once

#include <stdint.h>

typedef struct {
    float xd_peak_mm;       // largest predicted driver excursion since last read
    float xp_peak_mm;       // largest predicted passive radiator excursion
    float out_peak_v;       // largest output voltage requested
    float power_avg_w;      // average electrical power into the speaker
    float bass_gain_min;    // lowest bass-band gain (1.0 = not limiting)
    float peak_gain_min;    // lowest peak-limiter gain (1.0 = not limiting)
    float power_gain;       // current power-budget gain (1.0 = not limiting)
    uint32_t clamped;       // samples that reached the safety clamp
} speaker_dsp_stats_t;

// Build the filters and the speaker model. Call once before processing.
void speaker_dsp_init(void);

// Process one mono sample. Input and output are in digital full-scale units
// (1.0 = full scale). The output never exceeds the voltage ceiling.
float speaker_dsp_process(float x);

// Copy the statistics and restart the peak/minimum tracking.
void speaker_dsp_read_stats(speaker_dsp_stats_t *stats);

// Total delay through the processing, in samples.
uint32_t speaker_dsp_latency_samples(void);
