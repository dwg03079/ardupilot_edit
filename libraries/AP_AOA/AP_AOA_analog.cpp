/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
/*
 *   analog AOA driver
 */

#include "AP_AOA_analog.h"

#if AP_AOA_ANALOG_ENABLED

#include <AP_HAL/AP_HAL.h>
#include <AP_Common/AP_Common.h>

#include "AP_AOA.h"

extern const AP_HAL::HAL &hal;

// scaling for 3DR analog AOA sensor
#define VOLTS_TO_PASCAL 819

AP_AOA_Analog::AP_AOA_Analog(AP_AOA &_frontend, uint8_t _instance) :
    AP_AOA_Backend(_frontend, _instance)
{
    _source = hal.analogin->channel(get_pin());
}

bool AP_AOA_Analog::init()
{
    return _source != nullptr;
}

// read the AOA sensor
bool AP_AOA_Analog::get_differential_pressure(float &pressure)
{
    // allow pin to change
    if (_source == nullptr || !_source->set_pin(get_pin())) {
        return false;
    }
    pressure = _source->voltage_average_ratiometric() * VOLTS_TO_PASCAL / get_psi_range();
    return true;
}

#endif  // AP_AOA_ANALOG_ENABLED
