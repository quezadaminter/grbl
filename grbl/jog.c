/*
  jog.h - Jogging methods
  Part of Grbl

  Copyright (c) 2016 Sungeun K. Jeon for Gnea Research LLC

  Grbl is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  Grbl is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with Grbl.  If not, see <http://www.gnu.org/licenses/>.
*/

#include "grbl.h"

// Sets up valid jog motion received from g-code parser, checks for soft-limits, and executes the jog.
uint8_t jog_execute(plan_line_data_t *pl_data, parser_block_t *gc_block)
{
  // Initialize planner data struct for jogging motions.
  // NOTE: Spindle and coolant are allowed to fully function with overrides during a jog.
  pl_data->feed_rate = gc_block->values.f;
  pl_data->condition |= PL_COND_FLAG_NO_FEED_OVERRIDE;
  #ifdef USE_LINE_NUMBERS
    pl_data->line_number = gc_block->values.n;
  #endif

  if (bit_istrue(settings.flags,BITFLAG_SOFT_LIMIT_ENABLE)) {
    if (system_check_travel_limits(gc_block->values.xyz)) { return(STATUS_TRAVEL_EXCEEDED); }
  }

#ifdef JOG_WITH_PULSES
  if(sys.state == STATE_IDLE)
  {
    sys.state = STATE_JOG;
    //cli();
    //st_wake_up();
    //TIMSK1 &= ~(1<<OCIE1A);
    //sei();

    if(bit_istrue(settings.flags,BITFLAG_INVERT_ST_ENABLE))
    {
      STEPPERS_DISABLE_PORT |= (1<<STEPPERS_DISABLE_BIT);
    }
    else
    {
      STEPPERS_DISABLE_PORT &= ~(1<<STEPPERS_DISABLE_BIT);
    }

    uint8_t idx;
    uint8_t dir_mask;
    uint8_t step_mask;
    int32_t target_steps = 0;
    uint16_t delay = 100;

    for (idx = 0; idx < N_AXIS; idx++)
    {
      target_steps = gc_block->values.xyz[idx] - sys_position[idx];
      if(target_steps != 0)
      {
        delay = (1.0 / ((settings.steps_per_mm[idx] * settings.max_rate[idx]) / 60)) * 1000000;

        dir_mask = get_direction_pin_mask(idx);
        step_mask = get_step_pin_mask(idx);
        if(target_steps < 0)
        {
          DIRECTION_PORT |= dir_mask;
        }
        else
        {
          DIRECTION_PORT &= ~(dir_mask);
        }

        uint32_t s = abs(target_steps);
        while(s != 0)
        {
          //STEP_PORT ^= (STEP_PORT & ~STEP_MASK) | get_step_pin_mask(idx);
          STEP_PORT ^= step_mask;
          if(DIRECTION_PORT & (dir_mask))
          {
            sys_position[idx]--;
          }
          else
          {
            sys_position[idx]++;
          }
          --s;

          // Apply a delay to keep the movement to less than the maximum rate
          // configured for the machine. This protects against a stream of
          // pulses that can otherwise overwhelm the machine.
          delay_us(delay);
        }
      }
    }
    st_go_idle();
    sys.state = STATE_IDLE;
  }

#else
  //Valid jog command. Plan, set state, and execute.
  mc_line(gc_block->values.xyz,pl_data);
  if (sys.state == STATE_IDLE) {
    if (plan_get_current_block() != NULL) { // Check if there is a block to execute.
      sys.state = STATE_JOG;
      st_prep_buffer();
      st_wake_up();  // NOTE: Manual start. No state machine required.
    }
  }
#endif

  return(STATUS_OK);
}
