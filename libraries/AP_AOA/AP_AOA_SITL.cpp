#include "AP_AOA_SITL.h"

#if AP_AOA_SITL_ENABLED

#include <AP_Baro/AP_Baro.h>
#include <SITL/SITL.h>

AP_AOA_SITL::AP_AOA_SITL(AP_AOA& _frontend, uint8_t _instance) :
    AP_AOA_Backend(_frontend, _instance)
{
}

// return the current differential_pressure in Pascal
bool AP_AOA_SITL::get_differential_pressure(float &pressure)
{
    const uint8_t _instance = get_instance();

    if (_instance >= AOA_MAX_SENSORS) {
        return false;
    }

    //pressure = AP::sitl()->state.airspeed_raw_pressure[_instance];

    float aoa_ahrs = ahrs_AOA();
    //float pressure_up = pressure * cosf(radians(aoa_ahrs) + (4 / M_PI)) * cosf(radians(aoa_ahrs) + (4 / M_PI));
    //float pressure_down = pressure * cosf(radians(aoa_ahrs) - (4 / M_PI)) * cosf(radians(aoa_ahrs) + (4 / M_PI));
    //pressure = pressure_up - pressure_down;
    pressure = -(AP::sitl()->state.airspeed_raw_pressure[_instance]) * 2.0f * tanf(radians(aoa_ahrs));

    return true;
}

// get last temperature
bool AP_AOA_SITL::get_temperature(float &temperature)
{
    const uint8_t _instance = get_instance();

    if (_instance >= AOA_MAX_SENSORS) {
        return false;
    }

    const auto *sitl = AP::sitl();

    // this was mostly swiped from SIM_AOA_DLVR:
    const float sim_alt = sitl->state.altitude;

    float sigma, delta, theta;
    AP_Baro::SimpleAtmosphere(sim_alt * 0.001f, sigma, delta, theta);

    // To Do: Add a sensor board temperature offset parameter
    temperature = (KELVIN_TO_C(SSL_AIR_TEMPERATURE * theta)) + 25.0;

    return true;
}

#endif // AP_AOA_SITL_ENABLED
