/*
  SITL AOA backend - a perfect AOA sensor
 */
#pragma once

#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/AP_HAL_Boards.h>

#include <AP_Math/AP_Math.h>

#ifndef AP_AOA_SITL_ENABLED
#define AP_AOA_SITL_ENABLED AP_SIM_ENABLED
#endif

#if AP_AOA_SITL_ENABLED

#include "AP_AOA_Backend.h"

class AP_AOA_SITL : public AP_AOA_Backend
{
public:

    //using AP_AOA_Backend::AP_AOA_Backend;

    AP_AOA_SITL(AP_AOA& frontend, uint8_t instance);
    ~AP_AOA_SITL() {};

    bool init(void) override {
        return true;
    }

    // return the current differential_pressure in Pascal
    bool get_differential_pressure(float &pressure) override;

    // temperature not available via analog backend
    bool get_temperature(float &temperature) override;

private:
    //AP_AOA& frontend;
    //uint8_t instance;
};

#endif // AP_AOA_SITL_ENABLED
