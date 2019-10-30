/**
 * Marlin 3D Printer Firmware
 *
 * Copyright (c) 2019 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
 * Copyright (c) 2016 Bob Cousins bobcousins42@googlemail.com
 * Copyright (c) 2015-2016 Nico Tonnhofer wurstnase.reprap@gmail.com
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

#if defined(ARDUINO_ARCH_STM32) && !defined(STM32GENERIC)

#include "HAL.h"

#include "timers.h"

// ------------------------
// Local defines
// ------------------------

#define NUM_HARDWARE_TIMERS 2

#define __TIMER_DEV(X) TIM##X
#define _TIMER_DEV(X) __TIMER_DEV(X)
#define STEP_TIMER_DEV _TIMER_DEV(STEP_TIMER)
#define TEMP_TIMER_DEV _TIMER_DEV(TEMP_TIMER)

// ------------------------
// Private Variables
// ------------------------

HardwareTimer *timer_instance[NUM_HARDWARE_TIMERS] = { NULL };
bool timer_enabled[NUM_HARDWARE_TIMERS] = { false };

// ------------------------
// Public functions
// ------------------------

// frequency is in Hertz
void HAL_timer_start(const uint8_t timer_num, const uint32_t frequency) {
  if (!HAL_timer_initialized(timer_num)) {
   // According to documentation the prescale factor is computed automatically if setOverflow is in HERTZ_FORMAT
   switch (timer_num) {
      case STEP_TIMER_NUM:
        // STEPPER TIMER - use a 32bit timer if possible
        timer_instance[timer_num] = new HardwareTimer(STEP_TIMER_DEV);
        timer_instance[timer_num]->setMode(1, TIMER_OUTPUT_COMPARE, NC);
        timer_instance[timer_num]->setPrescaleFactor(STEPPER_TIMER_PRESCALE);
        timer_instance[timer_num]->setOverflow(frequency, HERTZ_FORMAT);
        timer_instance[timer_num]->attachInterrupt(Step_Handler); // Called on rollover AND
        timer_instance[timer_num]->attachInterrupt(1, Step_Handler); // called on channel 1 compare
        break;
      case TEMP_TIMER_NUM:
        // TEMP TIMER - any available 16bit Timer
        timer_instance[timer_num] = new HardwareTimer(TEMP_TIMER_DEV);
        timer_instance[timer_num]->setMode(1, TIMER_OUTPUT_COMPARE, NC);
        timer_instance[timer_num]->setOverflow(frequency, HERTZ_FORMAT);
        timer_instance[timer_num]->attachInterrupt(Temp_Handler); //Just called on rollover
        break;
    }
  }
}

void HAL_timer_enable_interrupt(const uint8_t timer_num) {
  if (HAL_timer_initialized(timer_num) && !timer_enabled[timer_num]) {
    // Resume the timer to start receiving the interrupt
    timer_instance[timer_num]->resume();
    timer_enabled[timer_num] = true;
  }
}

void HAL_timer_disable_interrupt(const uint8_t timer_num) {
  if (HAL_timer_interrupt_enabled(timer_num)) {
    // Pause the timer instead of detaching the interrupt through detachInterrupt()
    timer_instance[timer_num]->pause();
    timer_enabled[timer_num] = false;
  }
}

bool HAL_timer_interrupt_enabled(const uint8_t timer_num) {
  return HAL_timer_initialized(timer_num) && timer_enabled[timer_num];
}

//warning: From great power comes great responsibility.
//it's here but please avoid using it if not absolutely necessary.
//accessing hardware registers from outside the HAL lowers compatibility with other platforms.
TIM_TypeDef * HAL_timer_device(const uint8_t timer_num) {
  switch (timer_num) {
      case STEP_TIMER_NUM: return STEP_TIMER_DEV;
      case TEMP_TIMER_NUM: return TEMP_TIMER_DEV;
  }
  return nullptr;
}
#endif // ARDUINO_ARCH_STM32 && !STM32GENERIC
