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

#include <limits.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "battery_life_face.h"
#include "filesystem.h"
#include "tcc.h"
#include "usb.h"
#include "watch.h"

#define BATTERY_LIFE_FILENAME "battery.dat"
#define BATTERY_LIFE_MAGIC 0x424C4702  // "BLG", revision 2

/* ADC reads averaged into one stored sample. The hardware already averages 16
 * conversions per read, so this only guards against a single bad conversion.
 * It does not improve resolution, and it does not need to: the fit uses 64
 * samples, so the 1 mV quantization of watch_get_vcc_voltage averages out to
 * far less than the error in the discharge curve. */
#define BATTERY_LIFE_OVERSAMPLE 4

/* The minute of the hour at which samples are taken.
 *
 * This must not be minute 0. clock_face requests a background task at minute 0
 * and plays the hourly chime from it (clock_face.c). Movement runs background
 * tasks in watch-face order, so a sample at :00 would be measured while the
 * buzzer is sounding. That lowers every sample by about the same amount, and a
 * trend fit cannot average out an error that is the same every time. */
#define BATTERY_LIFE_SAMPLE_MINUTE 37

/* Minimum time between samples. */
#define BATTERY_LIFE_MIN_INTERVAL (20 * 3600)
/* If we are this overdue, take the next clean hour instead of waiting for the
 * pinned one. Samples get skipped when USB is attached or the buzzer is on. */
#define BATTERY_LIFE_RETRY_AFTER (25 * 3600)

/* A gap larger than this means the clock was changed. Such a sample would
 * stretch the fit's time axis and flatten the slope, so skip it. */
#define BATTERY_LIFE_MAX_GAP (14 * 86400)

/* Minimum data before fitting a trend. Over a shorter span the fit is mostly
 * measurement noise. */
#define BATTERY_LIFE_MIN_SAMPLES 4
#define BATTERY_LIFE_MIN_SPAN (14 * 86400)

/* Voltage rise that means the cell was swapped, in tenths of a millivolt. A
 * cell never gains voltage on its own. 100 mV is well above any temperature
 * swing. Two consecutive high readings are required, because one is more often
 * a sample taken on USB power than a new cell. */
#define BATTERY_LIFE_NEW_CELL_JUMP 1000

/* Longest life we will show a number for. */
#define BATTERY_LIFE_MAX_DAYS 999

/* Temperature coefficient of the cell's terminal voltage, in microvolts per
 * degree C, and the temperature the curve below is referenced to.
 *
 * This is the least certain value in this file. Manufacturers publish loaded
 * terminal voltage against temperature, not a coefficient, so this is read off
 * such a curve. It matters: over a season, temperature moves the reading more
 * than discharge does. A wrong coefficient is little better than none, so
 * measure your own cell if you can. Readings and temperatures are stored
 * separately, so a corrected coefficient can be applied to existing data. */
#define BATTERY_LIFE_TEMPCO_UV_PER_C 3200
#define BATTERY_LIFE_REFERENCE_TEMPERATURE 20

/* Approximate discharge curve for the CR2016 in a Sensor Watch: voltage in
 * millivolts against the charge remaining at that voltage, in permille.
 *
 * Raw voltage cannot be extrapolated. A lithium coin cell stays on a flat
 * plateau for most of its life and then drops quickly, so a straight line
 * through recent voltages predicts years for months and then days all at once.
 * Converting to charge first makes the problem roughly linear, because charge
 * drains at a steady rate under a steady load.
 *
 * Two limits on how much to trust the result:
 *
 *  - The shape is for the microamp drain this watch pulls, not the
 *    hundred-microamp curves shown on datasheet front pages. At low drain the
 *    plateau is much flatter and longer, so most of the capacity below falls
 *    within a few tens of millivolts.
 *  - Published curves for this cell differ by about 2x in the tail, so the
 *    tail here is a compromise and the error bar is wide. Zero is set at 2.2 V,
 *    where the watch stops being dependable, not where the cell is empty. The
 *    firmware warns at 2.4 V (clock_face.c) and the regulator runs well below
 *    that (watch.c).
 *
 * Change this table if your own measurements differ. Everything else uses it.
 */
static const struct {
    uint16_t millivolts;
    uint16_t permille;
} battery_life_curve[] = {
    { 3250, 1000 },
    { 3050,  980 },
    { 3020,  900 },
    { 3000,  800 },
    { 2985,  700 },
    { 2965,  600 },
    { 2940,  500 },
    { 2910,  400 },
    { 2870,  300 },
    { 2820,  200 },
    { 2740,  120 },
    { 2600,   60 },
    { 2400,   20 },
    { 2200,    0 },
};

#define BATTERY_LIFE_CURVE_POINTS (sizeof(battery_life_curve) / sizeof(battery_life_curve[0]))

typedef struct {
    bool valid;             // enough samples over enough time to fit a trend
    bool projecting;        // valid, and a number of days can be given
    float charge_permille;  // charge now, taken from the fitted line
    float permille_per_day; // negative while discharging
    uint16_t days_left;
} battery_life_estimate_t;

/* Stored units (tenths of a millivolt) -> millivolts. */
static float _battery_life_millivolts(uint16_t tenth_mv) {
    return tenth_mv / 10.0f;
}

/* Convert a reading to what it would have been at the reference temperature.
 * Samples logged without a thermistor reading are returned unchanged. */
static float _battery_life_compensated_millivolts(uint16_t tenth_mv, int8_t half_degrees) {
    float millivolts = _battery_life_millivolts(tenth_mv);

    if (half_degrees == BATTERY_LIFE_NO_TEMPERATURE) return millivolts;

    float celsius = half_degrees / 2.0f;
    return millivolts - (BATTERY_LIFE_TEMPCO_UV_PER_C / 1000.0f) *
                        (celsius - BATTERY_LIFE_REFERENCE_TEMPERATURE);
}

/* Charge remaining at a given terminal voltage, in permille. */
static float _battery_life_charge_at(float millivolts) {
    if (millivolts >= battery_life_curve[0].millivolts) return battery_life_curve[0].permille;

    for (size_t i = 1; i < BATTERY_LIFE_CURVE_POINTS; i++) {
        if (millivolts >= battery_life_curve[i].millivolts) {
            float span = battery_life_curve[i - 1].millivolts - battery_life_curve[i].millivolts;
            float fraction = (millivolts - battery_life_curve[i].millivolts) / span;
            return battery_life_curve[i].permille +
                   fraction * (battery_life_curve[i - 1].permille - battery_life_curve[i].permille);
        }
    }

    return 0.0f;
}

/* Charge for a stored sample, with temperature correction applied. */
static float _battery_life_charge(uint16_t tenth_mv, int8_t half_degrees) {
    return _battery_life_charge_at(_battery_life_compensated_millivolts(tenth_mv, half_degrees));
}

/* Read the battery, in tenths of a millivolt. */
static uint16_t _battery_life_read_battery(void) {
    uint32_t total = 0;

    for (uint8_t i = 0; i < BATTERY_LIFE_OVERSAMPLE; i++) {
        total += watch_get_vcc_voltage();
    }

    return (uint16_t)((total * 10 + BATTERY_LIFE_OVERSAMPLE / 2) / BATTERY_LIFE_OVERSAMPLE);
}

/* Thermistor reading in half-degrees C, or the sentinel if there is no sensor.
 * Call this after reading the battery, never before: enabling the thermistor
 * puts a divider across the rail and lowers the voltage being measured. */
static int8_t _battery_life_read_temperature(void) {
    float celsius = movement_get_temperature();

    // movement returns 0xFFFFFFFF cast to float when there is no sensor.
    if (!(celsius > -60.0f && celsius < 60.0f)) return BATTERY_LIFE_NO_TEMPERATURE;

    int32_t half_degrees = (int32_t)lroundf(celsius * 2.0f);
    if (half_degrees <= BATTERY_LIFE_NO_TEMPERATURE) return BATTERY_LIFE_NO_TEMPERATURE;
    if (half_degrees > INT8_MAX) return INT8_MAX;

    return (int8_t)half_degrees;
}

/* Number of samples currently in the ring. */
static uint16_t _battery_life_stored(const battery_life_log_t *log) {
    return log->count < BATTERY_LIFE_NUM_SAMPLES ? log->count : BATTERY_LIFE_NUM_SAMPLES;
}

/* Ring slot of the i'th stored sample, counting from the oldest. */
static uint16_t _battery_life_slot(const battery_life_log_t *log, uint16_t i) {
    return (log->count - _battery_life_stored(log) + i) % BATTERY_LIFE_NUM_SAMPLES;
}

static void _battery_life_reset(battery_life_log_t *log) {
    memset(log, 0, sizeof(battery_life_log_t));
    log->magic = BATTERY_LIFE_MAGIC;
}

/* Returns false if the log could not be written. filesystem_write_file refuses
 * once free space is down to one block. Nothing is corrupted and the in-memory
 * log keeps working, but the caller needs to know so it can show it. */
static bool _battery_life_save(battery_life_log_t *log) {
    return filesystem_write_file(BATTERY_LIFE_FILENAME, (char *)log, sizeof(battery_life_log_t));
}

static void _battery_life_load(battery_life_log_t *log) {
    if (filesystem_get_file_size(BATTERY_LIFE_FILENAME) != (int32_t)sizeof(battery_life_log_t) ||
        !filesystem_read_file(BATTERY_LIFE_FILENAME, (char *)log, sizeof(battery_life_log_t)) ||
        log->magic != BATTERY_LIFE_MAGIC) {
        _battery_life_reset(log);
        return;
    }

    // A log timestamped in the future means the clock went backwards while we
    // were not running. That happens when the cell is pulled, which also resets
    // the RTC. Cold boot is the only reliable place to check: by the next daily
    // sample the user will have set the clock and the evidence is gone.
    uint16_t stored = _battery_life_stored(log);
    if (stored && watch_rtc_get_unix_time() + 3600 < log->timestamps[_battery_life_slot(log, stored - 1)]) {
        _battery_life_reset(log);
    }
}

/* True when something is loading the rail: the buzzer or LED (both use TCC0),
 * or a USB host powering the regulator instead of the cell. A reading taken now
 * would measure the load, not the battery. */
static bool _battery_life_supply_is_disturbed(void) {
    return usb_is_enabled() || tcc_is_enabled(0);
}

static void _battery_life_record(battery_life_state_t *state) {
    battery_life_log_t *log = &state->log;
    uint32_t now = watch_rtc_get_unix_time();
    uint16_t stored = _battery_life_stored(log);

    if (_battery_life_supply_is_disturbed()) return;

    if (stored) {
        uint32_t last = log->timestamps[_battery_life_slot(log, stored - 1)];

        // Time moving backwards, not moving, or jumping far forward all mean the
        // clock was changed, not that a day passed. Skip the sample instead of
        // resetting: a bad sample corrupts the fit for weeks, and the cold boot
        // check in _battery_life_load is what detects a swapped cell.
        if (now <= last || now - last > BATTERY_LIFE_MAX_GAP) return;
    }

    uint16_t voltage = _battery_life_read_battery();
    int8_t temperature = _battery_life_read_temperature();

    if (stored) {
        uint16_t last_voltage = log->voltages[_battery_life_slot(log, stored - 1)];

        if (voltage > last_voltage + BATTERY_LIFE_NEW_CELL_JUMP) {
            // Require a second high reading. A single one is more often a sample
            // taken while something was powering the rail than a new cell.
            if (log->pending_high < 1) {
                log->pending_high++;
                return;
            }
            _battery_life_reset(log);
            stored = 0;
        } else {
            log->pending_high = 0;
        }
    }

    if (stored == 0) {
        // Fix the daily sample to the hour we started on, so every reading is
        // taken at a similar point in the day's temperature cycle.
        log->sample_hour = watch_rtc_get_date_time().unit.hour;
    }

    uint16_t slot = log->count % BATTERY_LIFE_NUM_SAMPLES;
    log->timestamps[slot] = now;
    log->voltages[slot] = voltage;
    log->temperatures[slot] = temperature;
    log->count++;

    state->save_failed = !_battery_life_save(log);
}

/* Least squares fit of charge against time. Fitting charge rather than voltage
 * matters: the voltage plateau makes a voltage trend meaningless for most of the
 * cell's life, while charge drains at a steady rate under a steady load. */
static battery_life_estimate_t _battery_life_estimate(const battery_life_log_t *log) {
    battery_life_estimate_t estimate = {0};
    uint16_t n = _battery_life_stored(log);

    if (n < BATTERY_LIFE_MIN_SAMPLES) return estimate;

    uint32_t first = log->timestamps[_battery_life_slot(log, 0)];
    uint32_t last = log->timestamps[_battery_life_slot(log, n - 1)];
    if (last - first < BATTERY_LIFE_MIN_SPAN) return estimate;

    float sum_days = 0, sum_charge = 0;
    for (uint16_t i = 0; i < n; i++) {
        uint16_t slot = _battery_life_slot(log, i);
        sum_days += (log->timestamps[slot] - first) / 86400.0f;
        sum_charge += _battery_life_charge(log->voltages[slot], log->temperatures[slot]);
    }
    float mean_days = sum_days / n;
    float mean_charge = sum_charge / n;

    float covariance = 0, variance = 0;
    for (uint16_t i = 0; i < n; i++) {
        uint16_t slot = _battery_life_slot(log, i);
        float days = (log->timestamps[slot] - first) / 86400.0f - mean_days;
        covariance += days * (_battery_life_charge(log->voltages[slot], log->temperatures[slot]) - mean_charge);
        variance += days * days;
    }
    if (variance <= 0) return estimate;

    estimate.permille_per_day = covariance / variance;
    estimate.valid = true;

    // Take the current charge from the fitted line rather than the last sample.
    // This reduces noise and accounts for time passed since that sample.
    uint32_t now = watch_rtc_get_unix_time();
    float now_days = (now > last ? now - first : last - first) / 86400.0f;
    estimate.charge_permille = mean_charge + estimate.permille_per_day * (now_days - mean_days);
    if (estimate.charge_permille < 0) estimate.charge_permille = 0;
    if (estimate.charge_permille > 1000) estimate.charge_permille = 1000;

    if (estimate.charge_permille < 1.0f) {
        // The cell is empty. This case needs handling on its own: once every
        // sample in the window is at the bottom of the curve the fitted slope is
        // zero, and treating zero slope as "no measurable drain" would report a
        // very long life at the moment the cell dies.
        estimate.days_left = 0;
        estimate.projecting = true;
    } else if (estimate.permille_per_day < 0) {
        float days = estimate.charge_permille / -estimate.permille_per_day;
        if (days <= BATTERY_LIFE_MAX_DAYS) {
            estimate.days_left = (uint16_t)(days + 0.5f);
            estimate.projecting = true;
        }
    }

    return estimate;
}

/* The top right is two digits. On the classic LCD those digits share segments
 * and can only form numbers up to 39 (see watch_slcd.h). Show the true count
 * where it fits, and clamp on the classic display, because an unclamped value
 * there renders as a smaller number with no sign that it is wrong. */
static void _battery_life_display_count(battery_life_state_t *state) {
    char buf[8];
    char fallback[8];

    if (state->save_failed) {
        // In the left cell of the top right, '-' renders as three stacked bars
        // on the classic LCD. Put the dash in the right cell on both displays.
        watch_display_text(WATCH_POSITION_TOP_RIGHT, " -");
        return;
    }

    unsigned int stored = _battery_life_stored(&state->log);
    snprintf(buf, sizeof(buf), "%2u", stored);
    snprintf(fallback, sizeof(fallback), "%2u", stored > 39 ? 39 : stored);
    watch_display_text_with_fallback(WATCH_POSITION_TOP_RIGHT, buf, fallback);
}

static void _battery_life_update_display(battery_life_state_t *state) {
    char buf[8];

    // The voltage page lights the decimal point on the custom LCD. Clear it so
    // it does not stay lit on pages that show whole numbers.
    watch_clear_decimal_if_available();

    if (state->reset_ticks) {
        watch_display_text_with_fallback(WATCH_POSITION_TOP_LEFT, "LOG", "LO");
        watch_display_text(WATCH_POSITION_TOP_RIGHT, "  ");
        watch_display_text(WATCH_POSITION_BOTTOM, "CLEAr ");
        return;
    }

    // The sample count shows in the top right on every page. It tells the user
    // how much data is behind the estimate.
    _battery_life_display_count(state);

    switch (state->page) {
        case BATTERY_LIFE_PAGE_DAYS_LEFT: {
            battery_life_estimate_t estimate = _battery_life_estimate(&state->log);
            watch_display_text_with_fallback(WATCH_POSITION_TOP_LEFT, "DAY", "DA");
            if (!estimate.valid) {
                watch_display_text(WATCH_POSITION_BOTTOM, "  ----");
            } else if (!estimate.projecting) {
                // On the classic LCD, bottom positions 4 and 6 wire the top and
                // bottom segments together, which distorts '>'. Put it in
                // position 5 so the digits land in 6 through 8.
                watch_display_text(WATCH_POSITION_BOTTOM, " >999 ");
            } else {
                snprintf(buf, sizeof(buf), "%6u", (unsigned int)estimate.days_left);
                watch_display_text(WATCH_POSITION_BOTTOM, buf);
            }
            break;
        }

        case BATTERY_LIFE_PAGE_CHARGE: {
            battery_life_estimate_t estimate = _battery_life_estimate(&state->log);
            float permille;
            if (estimate.valid) {
                permille = estimate.charge_permille;
            } else {
                uint16_t stored = _battery_life_stored(&state->log);
                if (stored) {
                    // Use the newest logged sample rather than a new reading. It
                    // is the same data the fit uses and costs no ADC time.
                    uint16_t slot = _battery_life_slot(&state->log, stored - 1);
                    permille = _battery_life_charge(state->log.voltages[slot],
                                                    state->log.temperatures[slot]);
                } else {
                    // Nothing logged yet, so measure. Battery first, then the
                    // thermistor, which loads the rail when enabled. This must
                    // apply the same correction as the logged path above, or the
                    // value would jump when the first sample lands.
                    uint16_t live = _battery_life_read_battery();
                    permille = _battery_life_charge(live, _battery_life_read_temperature());
                }
            }
            watch_display_text_with_fallback(WATCH_POSITION_TOP_LEFT, "CHG", "CH");
            snprintf(buf, sizeof(buf), "%6d", (int)roundf(permille / 10.0f));
            watch_display_text(WATCH_POSITION_BOTTOM, buf);
            break;
        }

        case BATTERY_LIFE_PAGE_VOLTAGE:
            watch_display_text_with_fallback(WATCH_POSITION_TOP_LEFT, "BAT", "BA");
            watch_display_float_with_best_effort(
                _battery_life_millivolts(_battery_life_read_battery()) / 1000.0f, " V");
            break;

        case BATTERY_LIFE_PAGE_RATE: {
            battery_life_estimate_t estimate = _battery_life_estimate(&state->log);
            // 'R' only renders full height in position 1, so the two-character
            // fallback puts it there. In position 0 it renders as a stub.
            watch_display_text_with_fallback(WATCH_POSITION_TOP_LEFT, "RTE", "dR");
            if (!estimate.valid) {
                watch_display_text(WATCH_POSITION_BOTTOM, "  ----");
            } else {
                // Permille per day into percent per thirty days.
                int rate = (int)roundf(estimate.permille_per_day * 3.0f);
                if (rate < -999) rate = -999;
                if (rate > 999) rate = 999;
                snprintf(buf, sizeof(buf), "%6d", rate);
                watch_display_text(WATCH_POSITION_BOTTOM, buf);
            }
            break;
        }
    }
}

/* Low energy mode has the peripherals off and watch faces must not wake them,
 * so this shows only what the log can answer. It also keeps the digits out of
 * positions 8 and 9, which the classic LCD sleep animation clears. */
static void _battery_life_display_low_energy(battery_life_state_t *state) {
    battery_life_estimate_t estimate = _battery_life_estimate(&state->log);
    char buf[8];

    watch_clear_decimal_if_available();
    _battery_life_display_count(state);
    watch_display_text_with_fallback(WATCH_POSITION_TOP_LEFT, "DAY", "DA");

    if (!estimate.valid) {
        watch_display_text(WATCH_POSITION_BOTTOM, "----  ");
    } else if (!estimate.projecting) {
        watch_display_text(WATCH_POSITION_BOTTOM, ">999  ");
    } else {
        snprintf(buf, sizeof(buf), "%4u  ", (unsigned int)estimate.days_left);
        watch_display_text(WATCH_POSITION_BOTTOM, buf);
    }
}

void battery_life_face_setup(uint8_t watch_face_index, void ** context_ptr) {
    (void) watch_face_index;

    if (*context_ptr == NULL) {
        *context_ptr = malloc(sizeof(battery_life_state_t));
        memset(*context_ptr, 0, sizeof(battery_life_state_t));
        _battery_life_load(&((battery_life_state_t *)*context_ptr)->log);
    }
}

void battery_life_face_activate(void *context) {
    battery_life_state_t *state = (battery_life_state_t *)context;

    state->page = BATTERY_LIFE_PAGE_DAYS_LEFT;
    state->reset_ticks = 0;
    if (watch_sleep_animation_is_running()) watch_stop_sleep_animation();
}

bool battery_life_face_loop(movement_event_t event, void *context) {
    battery_life_state_t *state = (battery_life_state_t *)context;

    switch (event.event_type) {
        case EVENT_ACTIVATE:
            _battery_life_update_display(state);
            break;

        case EVENT_TICK:
            if (state->reset_ticks && --state->reset_ticks == 0) {
                _battery_life_update_display(state);
            } else if (state->page == BATTERY_LIFE_PAGE_VOLTAGE &&
                       movement_get_local_date_time().unit.second % 5 == 0) {
                // The only page backed by a live reading.
                _battery_life_update_display(state);
            }
            break;

        case EVENT_ALARM_LONG_UP:
            // Releasing the clear gesture also arrives here. Ignore that case,
            // or the log is wiped and the page turns straight afterwards.
            if (state->reset_ticks) break;
            // Otherwise the press was held between half a second and a second
            // and a half. Turn the page rather than ignoring it.
            // fall through
        case EVENT_ALARM_BUTTON_UP:
            state->page = (state->page + 1) % BATTERY_LIFE_PAGE_COUNT;
            state->reset_ticks = 0;
            _battery_life_update_display(state);
            break;

        case EVENT_ALARM_REALLY_LONG_PRESS:
            // Clearing the log needs a really long press. A short press turns
            // the page, and users often hold it slightly too long.
            _battery_life_reset(&state->log);
            state->save_failed = !_battery_life_save(&state->log);
            state->page = BATTERY_LIFE_PAGE_DAYS_LEFT;
            state->reset_ticks = 2;
            _battery_life_update_display(state);
            break;

        case EVENT_TIMEOUT:
            // Nothing here changes more than once a day, so there is no reason
            // to stay on screen. Resigning also keeps the face out of low energy
            // mode, where the display and peripheral rules are restrictive.
            movement_move_to_face(0);
            break;

        case EVENT_LOW_ENERGY_UPDATE:
            if (!watch_sleep_animation_is_running()) {
                _battery_life_display_low_energy(state);
                watch_start_sleep_animation(1000);
            } else if (movement_get_local_date_time().unit.minute == 0) {
                _battery_life_display_low_energy(state);
            }
            break;

        case EVENT_BACKGROUND_TASK:
            _battery_life_record(state);
            break;

        default:
            return movement_default_loop_handler(event);
    }

    return true;
}

void battery_life_face_resign(void *context) {
    (void) context;
}

movement_watch_face_advisory_t battery_life_face_advise(void *context) {
    battery_life_state_t *state = (battery_life_state_t *)context;
    movement_watch_face_advisory_t retval = { 0 };

    // Called at the top of every minute. Only one minute of the hour is ours.
    watch_date_time_t date_time = watch_rtc_get_date_time();
    if (date_time.unit.minute != BATTERY_LIFE_SAMPLE_MINUTE) return retval;

    uint16_t stored = _battery_life_stored(&state->log);
    if (stored == 0) {
        // Nothing logged yet. Take the first reading at the next opportunity;
        // that hour becomes the daily slot.
        retval.wants_background_task = true;
        return retval;
    }

    uint32_t last = state->log.timestamps[_battery_life_slot(&state->log, stored - 1)];
    uint32_t now = watch_rtc_get_unix_time();
    if (now < last) {
        // Clock moved backwards. Let _battery_life_record decide what to do.
        retval.wants_background_task = true;
        return retval;
    }

    uint32_t elapsed = now - last;
    if (elapsed < BATTERY_LIFE_MIN_INTERVAL) return retval;

    // Normally wait for the pinned hour. If a sample was skipped because USB
    // was attached or the buzzer was on, take the next clean hour instead of
    // losing a whole day.
    retval.wants_background_task = (date_time.unit.hour == state->log.sample_hour) ||
                                   (elapsed >= BATTERY_LIFE_RETRY_AFTER);

    return retval;
}
