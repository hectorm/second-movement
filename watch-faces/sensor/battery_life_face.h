/*
 * MIT License
 *
 * Copyright (c) 2026 Héctor Molinero Fernández
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#ifndef BATTERY_LIFE_FACE_H_
#define BATTERY_LIFE_FACE_H_

/*
 * BATTERY LIFE FACE
 *
 * Logs the battery voltage once a day and fits a trend to estimate how many
 * days the cell has left.
 *
 * Voltage cannot be extrapolated directly. A lithium coin cell stays on a
 * nearly flat plateau for most of its life and then drops quickly, so a line
 * through recent voltages predicts years for months and then days all at once.
 * This face converts each reading to a state of charge first (see the discharge
 * curve in the .c file) and fits the trend to that, because charge drains at a
 * steady rate under a steady load.
 *
 * Pages, cycled with the alarm button:
 *
 *   DAY  days of life remaining, or ---- when there is not enough data yet
 *   CHG  estimated state of charge, in percent
 *   BAT  the battery voltage now, updated live
 *   RTE  rate of charge use, in percent per 30 days
 *
 * The top right shows how many daily samples are behind the estimate. It reads
 * " -" if the log cannot be written to the filesystem, which happens when the
 * 8 KB of storage is full. The estimate still works from memory, but the
 * history will not survive a reset until space is freed.
 *
 * A really long press of the alarm button clears the log. This is rarely
 * needed: the face detects a swapped cell at the next daily sample when the
 * voltage rises, and at cold boot if pulling the cell reset the clock.
 *
 * Accuracy is limited by three things, in order:
 *
 *  1. The discharge curve is an approximation, and published curves for this
 *     cell differ by about 2x in the tail. Tune it against your own cell first.
 *  2. Temperature. A coin cell's terminal voltage moves by a few millivolts per
 *     degree, which over a season exceeds the discharge being measured. The
 *     face logs the thermistor reading with each sample and corrects for it.
 *     BATTERY_LIFE_TEMPCO_UV_PER_C is the coefficient and the least certain
 *     value here. Raw readings are stored, so a better coefficient can be
 *     applied later without losing data.
 *  3. Current drawn at the moment of the sample. Samples are taken away from
 *     the top of the hour to avoid the hourly chime, and are skipped while USB
 *     is attached or the buzzer or LED is on.
 *
 * Expect the first weeks after a battery change to be unreliable. A fresh cell
 * loses its surface charge quickly, and while that dominates the window the
 * trend can be well out in either direction.
 */

#include "movement.h"

/* Daily samples kept in the rolling log. Nine weeks of baseline gives a more
 * stable trend than shorter windows at almost no cost: the log is rewritten in
 * place, and littlefs allocates in 256-byte blocks, so any size from 42 to 64
 * samples uses the same 1 KB of the 8 KB filesystem (measured). The per-sample
 * temperature byte is free for the same reason. */
#define BATTERY_LIFE_NUM_SAMPLES 64

/* Sentinel stored in temperatures[] when there was no usable thermistor reading. */
#define BATTERY_LIFE_NO_TEMPERATURE INT8_MIN

/* The on-disk log. Each field has its own array to keep everything naturally
 * aligned. The Cortex-M0+ cannot do unaligned loads, so an array of packed
 * structs is not an option. */
typedef struct {
    uint32_t magic;
    uint32_t timestamps[BATTERY_LIFE_NUM_SAMPLES];  // UTC, unix time
    uint16_t voltages[BATTERY_LIFE_NUM_SAMPLES];    // tenths of a millivolt
    int8_t temperatures[BATTERY_LIFE_NUM_SAMPLES];  // half-degrees C, or BATTERY_LIFE_NO_TEMPERATURE
    uint16_t count;                                 // samples taken since the log was last reset
    uint8_t sample_hour;                            // UTC hour that daily samples are taken at
    uint8_t pending_high;                           // consecutive high readings; two means a new cell
} battery_life_log_t;

typedef enum {
    BATTERY_LIFE_PAGE_DAYS_LEFT = 0,
    BATTERY_LIFE_PAGE_CHARGE,
    BATTERY_LIFE_PAGE_VOLTAGE,
    BATTERY_LIFE_PAGE_RATE,
    BATTERY_LIFE_PAGE_COUNT,
} battery_life_page_t;

typedef struct {
    battery_life_log_t log;
    uint8_t page;
    uint8_t reset_ticks;  // counts down while the "CLEAr" message is on screen
    bool save_failed;     // the log could not be written; shown in the top right
} battery_life_state_t;

void battery_life_face_setup(uint8_t watch_face_index, void ** context_ptr);
void battery_life_face_activate(void *context);
bool battery_life_face_loop(movement_event_t event, void *context);
void battery_life_face_resign(void *context);
movement_watch_face_advisory_t battery_life_face_advise(void *context);

#define battery_life_face ((const watch_face_t){ \
    battery_life_face_setup, \
    battery_life_face_activate, \
    battery_life_face_loop, \
    battery_life_face_resign, \
    battery_life_face_advise, \
})

#endif // BATTERY_LIFE_FACE_H_
