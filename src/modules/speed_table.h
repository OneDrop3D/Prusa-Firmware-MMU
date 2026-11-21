/// @file speed_table.h
#pragma once
#include "../config/config.h"
#include "../hal/progmem.h"
#include "../hal/cpu.h"
#include "math.h"

namespace modules {

/// Speed  tables for acceleration calculations
namespace speed_table {

typedef uint16_t st_timer_t;

/// CPU timer frequency divider required for the speed tables
static_assert(F_CPU / config::stepTimerFrequencyDivider == 2000000,
    "speed tables not compatible for the requested frequency");

/// First used index of the original fast table (lower rates go to the slow table)
static constexpr uint8_t speedTableFastFirst = 8;

/// The rate is at most 10000 after step_loops, so the original fast table is used up to index 39 (+1 for the slope)
static_assert(config::maxStepFrequency <= 40000, "speed_table_fast is cut for rates <= 10000 after step_loops");
static constexpr uint8_t speedTableFastSize = (10000 >> 8) + 2 - speedTableFastFirst;

/// The slow table covers rates 0..2047 by steps of 8 (+1 entry for the slope of the last segment)
static constexpr uint16_t speedTableSlowSize = 257;

/// Lookup table for rates equal or higher than 8*256 (interval only, the slope is the difference with the next entry)
extern const st_timer_t speed_table_fast[speedTableFastSize] PROGMEM;

/// Lookup table for lower step rates (interval only, the slope is the difference with the next entry)
extern const st_timer_t speed_table_slow[speedTableSlowSize] PROGMEM;

/// Calculate the next timer interval and steps according to current step rate
static inline st_timer_t calc_timer(st_timer_t step_rate, uint8_t &step_loops) {
    if (step_rate > config::maxStepFrequency)
        step_rate = config::maxStepFrequency;
    if (step_rate > 20000) { // If steprate > 20kHz >> step 4 times
        step_rate = (step_rate >> 2) & 0x3fff;
        step_loops = 4;
    } else if (step_rate > 10000) { // If steprate > 10kHz >> step 2 times
        step_rate = (step_rate >> 1) & 0x7fff;
        step_loops = 2;
    } else {
        step_loops = 1;
    }

    using modules::math::mulU8X16toH16;
    namespace pm = hal::progmem;

    st_timer_t timer; // calculated interval

    if (step_rate < (F_CPU / 500000))
        step_rate = (F_CPU / 500000);
    step_rate -= (F_CPU / 500000); // Correct for minimal speed
    if (step_rate >= (8 * 256)) { // higher step rate
        const uint16_t *table_address = &speed_table_fast[(uint8_t)(step_rate >> 8) - speedTableFastFirst];
        uint8_t tmp_step_rate = (step_rate & 0x00ff);
        uint16_t base = pm::read_word(table_address);
        uint16_t gain = base - pm::read_word(table_address + 1);
        timer = mulU8X16toH16(tmp_step_rate, gain);
        timer = base - timer;
    } else { // lower step rates
        const uint16_t *table_address = &speed_table_slow[step_rate >> 3];
        timer = pm::read_word(table_address);
        uint16_t gain = timer - pm::read_word(table_address + 1);
        timer -= ((gain * (uint8_t)(step_rate & 0x0007)) >> 3);
    }
    if (timer < 100) {
        // 20kHz this should never happen
        timer = 100;
    }
    return timer;
}

} // namespace speed_table
} // namespace modules
