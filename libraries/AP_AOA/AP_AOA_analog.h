#pragma once

#include <AP_HAL/AP_HAL_Boards.h>

#ifndef AP_AOA_ANALOG_ENABLED
#define AP_AOA_ANALOG_ENABLED AP_AOA_BACKEND_DEFAULT_ENABLED
#endif

#if AP_AOA_ANALOG_ENABLED

#include <AP_HAL/AP_HAL.h>
#include <AP_Param/AP_Param.h>

#include "AP_AOA_Backend.h"

class AP_AOA_Analog : public AP_AOA_Backend
{
public:
    AP_AOA_Analog(AP_AOA &frontend, uint8_t _instance);

    // probe and initialise the sensor
    bool init(void) override;

    // return the current differential_pressure in Pascal
    bool get_differential_pressure(float &pressure) override;

    // temperature not available via analog backend
    bool get_temperature(float &temperature) override { return false; }

private:
    AP_HAL::AnalogSource *_source;
};

#endif  // AP_AOA_ANALOG_ENABLED
