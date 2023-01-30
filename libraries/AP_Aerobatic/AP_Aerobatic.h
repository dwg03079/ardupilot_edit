#pragma once

/// @file    AP_Aerobatic.h
/// @brief   Aerobatic algorithm. This is a instance of an
/// AP_Aerobatic class

/*
 * Written by SeungJin Lee
 */

#include <AP_Math/AP_Math.h>
#include <AP_AHRS/AP_AHRS.h>
#include <AP_Param/AP_Param.h>

#include <AP_Common/Location.h>

class AP_Aerobatic {
public:
    AP_Aerobatic(AP_AHRS& ahrs)
        : _ahrs(ahrs)
    {
        AP_Param::setup_object_defaults(this, var_info);
    }

    /* Do not allow copies */
    CLASS_NO_COPY(AP_Aerobatic);



private:
    // reference to the AHRS object
    AP_AHRS& _ahrs;


    AP_Float _L1_period;
    AP_Float _L1_damping;
    AP_Float _L1_xtrack_i_gain;
    AP_Float _loiter_bank_limit;
};
