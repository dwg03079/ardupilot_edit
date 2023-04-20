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
 *   AP_AOA.cpp - AOA (pitot) driver
 */

#include "AP_AOA.h"

#include <AP_Vehicle/AP_Vehicle_Type.h>

// Dummy the AP_AOA class to allow building AOA only for plane, rover, sub, and copter & heli 2MB boards
// This could be removed once the build system allows for APM_BUILD_TYPE in header files
// Note that this is also defined in AP_AOA_Params.cpp
#ifndef AP_AOA_DUMMY_METHODS_ENABLED
#define AP_AOA_DUMMY_METHODS_ENABLED ((APM_BUILD_COPTER_OR_HELI && BOARD_FLASH_SIZE <= 1024) || \
                                            APM_BUILD_TYPE(APM_BUILD_AntennaTracker) || APM_BUILD_TYPE(APM_BUILD_Blimp))
#endif

#if !AP_AOA_DUMMY_METHODS_ENABLED


#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/I2CDevice.h>
#include <AP_Math/AP_Math.h>
#include <GCS_MAVLink/GCS.h>
#include <SRV_Channel/SRV_Channel.h>
#include <AP_Logger/AP_Logger.h>
#include <utility>
#include "AP_AOA_MS4525.h"
#include "AP_AOA_MS5525.h"
#include "AP_AOA_SDP3X.h"
#include "AP_AOA_DLVR.h"
#include "AP_AOA_analog.h"
#include "AP_AOA_ASP5033.h"
#include "AP_AOA_Backend.h"
#include "AP_AOA_UAVCAN.h"
#include "AP_AOA_NMEA.h"

#include "AP_AOA_SITL.h"
extern const AP_HAL::HAL &hal;

#include <AP_Vehicle/AP_FixedWing.h>


#ifdef HAL_AOA_TYPE_DEFAULT
 #define AOA_DEFAULT_TYPE HAL_AOA_TYPE_DEFAULT
 #ifndef AOA_DEFAULT_PIN
 #define AOA_DEFAULT_PIN 1
 #endif
#elif APM_BUILD_TYPE(APM_BUILD_ArduPlane)
 // The HAL_BOARD_SITL setting is required because of current probe process for MS4525 will
 // connect and find the SIM_DLVR sensors & fault as there is no way to tell them apart
 #if CONFIG_HAL_BOARD == HAL_BOARD_SITL
  #define AOA_DEFAULT_TYPE TYPE_ANALOG
  #define AOA_DEFAULT_PIN 1
 #else
  #define AOA_DEFAULT_TYPE TYPE_I2C_MS4525
  #ifdef HAL_DEFAULT_AOA_PIN
      #define AOA_DEFAULT_PIN HAL_DEFAULT_AOA_PIN
  #else
     #define AOA_DEFAULT_PIN 15
  #endif
 #endif //CONFIG_HAL_BOARD
#else   // All Other Vehicle Types
 #define AOA_DEFAULT_TYPE TYPE_NONE
 #define AOA_DEFAULT_PIN 15
#endif

#ifndef HAL_AOA_BUS_DEFAULT
#define HAL_AOA_BUS_DEFAULT 1
#endif

#define OPTIONS_DEFAULT AP_AOA::OptionsMask::ON_FAILURE_AHRS_WIND_MAX_DO_DISABLE | AP_AOA::OptionsMask::ON_FAILURE_AHRS_WIND_MAX_RECOVERY_DO_REENABLE | AP_AOA::OptionsMask::USE_EKF_CONSISTENCY

#define ENABLE_PARAMETER !(APM_BUILD_TYPE(APM_BUILD_ArduPlane) || defined(HAL_BUILD_AP_PERIPH))

// table of user settable parameters
const AP_Param::GroupInfo AP_AOA::var_info[] = {

#if ENABLE_PARAMETER
    // @Param: _ENABLE
    // @DisplayName: AOA Enable
    // @Description: Enable AOA sensor support
    // @Values: 0:Disable, 1:Enable
    // @User: Standard
    AP_GROUPINFO_FLAGS("_ENABLE", 30, AP_AOA, _enable, 0, AP_PARAM_FLAG_ENABLE),
#endif
    // slots 0-9 (and 63) were previously used by params before being refactored into AP_AOA_Params

    // @Param: _TUBE_ORDER
    // @DisplayName: Control pitot tube order
    // @Description: This parameter allows you to control whether the order in which the tubes are attached to your pitot tube matters. If you set this to 0 then the first (often the top) connector on the sensor needs to be the stagnation pressure (the pressure at the tip of the pitot tube). If set to 1 then the second (often the bottom) connector needs to be the stagnation pressure. If set to 2 (the default) then the AOA driver will accept either order. The reason you may wish to specify the order is it will allow your AOA sensor to detect if the aircraft is receiving excessive pressure on the static port compared to the stagnation port such as during a stall, which would otherwise be seen as a positive AOA.
    // @User: Advanced
    // @Values: 0:Normal,1:Swapped,2:Auto Detect

    // tube order param had to be shortened so is not preserved in per group descriptions 

#if AOA_MAX_SENSORS > 1
    // @Param: _PRIMARY
    // @DisplayName: Primary AOA sensor
    // @Description: This selects which AOA sensor will be the primary if multiple sensors are found
    // @Values: 0:FirstSensor,1:2ndSensor
    // @User: Advanced
    AP_GROUPINFO("_PRIMARY", 10, AP_AOA, primary_sensor, 0),
#endif

    // 11-20 were previously used by second sensor params before being refactored into AP_AOA_Params

#ifndef HAL_BUILD_AP_PERIPH
    // @Param: _OPTIONS
    // @DisplayName: AOA options bitmask
    // @Description: Bitmask of options to use with AOA. 0:Disable use based on AOA/groundspeed mismatch (see AOA_WIND_MAX), 1:Automatically reenable use based on AOA/groundspeed mismatch recovery (see AOA_WIND_MAX) 2:Disable voltage correction, 3:Check that the AOA is statistically consistent with the navigation EKF vehicle and wind velocity estimates using EKF3 (requires AHRS_EKF_TYPE = 3)
    // @Description{Copter, Blimp, Rover, Sub}: This parameter and function is not used by this vehicle. Always set to 0.
    // @Bitmask: 0:SpeedMismatchDisable, 1:AllowSpeedMismatchRecovery, 2:DisableVoltageCorrection, 3:UseEkf3Consistency
    // @User: Advanced
    AP_GROUPINFO("_OPTIONS", 21, AP_AOA, _options, OPTIONS_DEFAULT),

    // @Param: _WIND_MAX
    // @DisplayName: Maximum AOA and ground speed difference
    // @Description: If the difference between AOA and ground speed is greater than this value the sensor will be marked unhealthy. Using AOA_OPTION this health value can be used to disable the sensor.
    // @Description{Copter, Blimp, Rover, Sub}: This parameter and function is not used by this vehicle. Always set to 0.
    // @Units: m/s
    // @User: Advanced
    AP_GROUPINFO("_WIND_MAX", 22, AP_AOA, _wind_max, 0),

    // @Param: _WIND_WARN
    // @DisplayName: AOA and ground speed difference that gives a warning
    // @Description: If the difference between AOA and ground speed is greater than this value the sensor will issue a warning. If 0 AOA_WIND_MAX is used.
    // @Description{Copter, Blimp, Rover, Sub}: This parameter and function is not used by this vehicle. Always set to 0.
    // @Units: m/s
    // @User: Advanced
    AP_GROUPINFO("_WIND_WARN", 23, AP_AOA, _wind_warn, 0),

    // @Param: _WIND_GATE
    // @DisplayName: Re-enable Consistency Check Gate Size
    // @Description: Number of standard deviations applied to the re-enable EKF consistency check that is used when AOA_OPTIONS bit position 3 is set. Larger values will make the re-enabling of the AOA sensor faster, but increase the likelihood of re-enabling a degraded sensor. The value can be tuned by using the ARSP.TR log message by setting ARSP_WIND_GATE to a value that is higher than the value for ARSP.TR observed with a healthy AOA sensor. Occasional transients in ARSP.TR above the value set by ARSP_WIND_GATE can be tolerated provided they are less than 5 seconds in duration and less than 10% duty cycle.
    // @Description{Copter, Blimp, Rover, Sub}: This parameter and function is not used by this vehicle.
    // @Range: 0.0 10.0
    // @User: Advanced
    AP_GROUPINFO("_WIND_GATE", 26, AP_AOA, _wind_gate, 5.0f),
    
    // @Param: _OFF_PCNT
    // @DisplayName: Maximum offset cal speed error 
    // @Description: The maximum percentage speed change in AOA reports that is allowed due to offset changes between calibraions before a warning is issued. This potential speed error is in percent of ASPD_FBW_MIN. 0 disables. Helps warn of calibrations without pitot being covered.
    // @Range: 0.0 10.0
    // @Units: %
    // @User: Advanced
    AP_GROUPINFO_FRAME("_OFF_PCNT", 27, AP_AOA, max_speed_pcnt, 0, AP_PARAM_FRAME_PLANE),    

#endif

    // @Group: _
    // @Path: AP_AOA_Params.cpp
    AP_SUBGROUPINFO(param[0], "_", 28, AP_AOA, AP_AOA_Params),

#if AOA_MAX_SENSORS > 1
    // @Group: 2_
    // @Path: AP_AOA_Params.cpp
    AP_SUBGROUPINFO(param[1], "2_", 29, AP_AOA, AP_AOA_Params),
#endif

    // index 30 is used by enable at the top of the table

    AP_GROUPEND
};

/*
  this scaling factor converts from the old system where we used a
  0 to 4095 raw ADC value for 0-5V to the new system which gets the
  voltage in volts directly from the ADC driver
 */
#define SCALING_OLD_CALIBRATION 819 // 4095/5

AP_AOA::AP_AOA(AP_AHRS &ahrs, AP_Airspeed &airspeed)
    : _ahrs(ahrs), _airspeed(airspeed)
{
    AP_Param::setup_object_defaults(this, var_info);

    // Setup defaults that only apply to first sensor
    param[0].type.set_default(AOA_DEFAULT_TYPE);
#ifndef HAL_BUILD_AP_PERIPH
    param[0].bus.set_default(HAL_AOA_BUS_DEFAULT);
    param[0].pin.set_default(AOA_DEFAULT_PIN);
#endif

    if (_singleton != nullptr) {
        AP_HAL::panic("AP_AOA must be singleton");
    }
    _singleton = this;
}

void AP_AOA::set_fixedwing_parameters(const AP_FixedWing *_fixed_wing_parameters)
{
    fixed_wing_parameters = _fixed_wing_parameters;
}

// macro for use by HAL_INS_PROBE_LIST
#define GET_I2C_DEVICE(bus, address) hal.i2c_mgr->get_device(bus, address)

bool AP_AOA::add_backend(AP_AOA_Backend *backend)
{
    if (!backend) {
        return false;
    }
    if (num_sensors >= AOA_MAX_SENSORS) {
        AP_HAL::panic("Too many AOA drivers");
    }
    const uint8_t i = num_sensors;
    sensor[num_sensors++] = backend;
    if (!sensor[i]->init()) {
        GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "AOA %u init failed", i+1);
        delete sensor[i];
        sensor[i] = nullptr;
    }
    return true;
}

/*
  macro to add a backend with check for too many sensors
  We don't try to start more than the maximum allowed
 */
#define ADD_BACKEND(backend) \
    do { add_backend(backend);     \
    if (num_sensors == AOA_MAX_SENSORS) { return; } \
    } while (0)


// convet params to per instance param table
// PARAMETER_CONVERSION - Added: Dec-2022
void AP_AOA::convert_per_instance()
{
    AP_Param::ConversionInfo info;
#ifndef HAL_BUILD_AP_PERIPH
    // Vehicle conversion
    if (!AP_Param::find_key_by_pointer(this, info.old_key)) {
        return;
    }

    static const struct convert_table {
        uint32_t element[2];
        ap_var_type type;
        const char* name;
    }  conversion_table[] = {
        { {4042, 714}, AP_PARAM_INT8, "TYPE" },      // AOA_TYPE, AOA2_TYPE
        { {74, 778}, AP_PARAM_INT8, "USE" },        // AOA_USE, AOA2_USE
        { {138, 842}, AP_PARAM_FLOAT, "OFFSET" },    // AOA_OFFSET, AOA2_OFFSET
        { {202, 906}, AP_PARAM_FLOAT, "RATIO" },     // AOA_RATIO, AOA2_RATIO
        { {266, 970}, AP_PARAM_INT8, "PIN" },        // AOA_PIN, AOA2_PIN
#if AP_AOA_AUTOCAL_ENABLE
        { {330, 1034}, AP_PARAM_INT8, "AUTOCAL" },    // AOA_AUTOCAL, AOA2_AUTOCAL
#endif
        { {394, 1098}, AP_PARAM_INT8, "TUBE_ORDR" },  // AOA_TUBE_ORDER, AOA2_TUBE_ORDR
        { {458, 1162}, AP_PARAM_INT8, "SKIP_CAL" },   // AOA_SKIP_CAL, AOA2_SKIP_CAL
        { {522, 1226}, AP_PARAM_FLOAT, "PSI_RANGE" }, // AOA_PSI_RANGE, AOA2_PSI_RANGE
        { {586, 1290}, AP_PARAM_INT8, "BUS" },        // AOA_BUS, AOA2_BUS
        { {1546, 1610}, AP_PARAM_INT32, "DEVID" },    // AOA_DEVID, AOA2_DEVID
    };

#else
    // Periph conversion
    if (!AP_Param::find_top_level_key_by_pointer(this, info.old_key)) {
        return;
    }
    const struct convert_table {
        uint32_t element[2];
        ap_var_type type;
        const char* name;
    }  conversion_table[] = {
        { {0, 11}, AP_PARAM_INT8, "TYPE" },      // AOA_TYPE, AOA2_TYPE
#if AP_AOA_AUTOCAL_ENABLE
        { {5, 16}, AP_PARAM_INT8, "AUTOCAL" },    // AOA_AUTOCAL, AOA2_AUTOCAL
#endif
        { {8, 19}, AP_PARAM_FLOAT, "PSI_RANGE" }, // AOA_PSI_RANGE, AOA2_PSI_RANGE
        { {24, 25}, AP_PARAM_INT32, "DEVID" },    // AOA_DEVID, AOA2_DEVID
    };
#endif

    char param_name[17] {};
    info.new_name = param_name;

    for (const auto & elem : conversion_table) {
        info.type = elem.type;
        for (uint8_t i=0; i < MIN(AOA_MAX_SENSORS,2); i++) {
            info.old_group_element = elem.element[i];
            if (i == 0) {
                hal.util->snprintf(param_name, sizeof(param_name), "AOA_%s",  elem.name);
            } else {
                hal.util->snprintf(param_name, sizeof(param_name), "AOA%i_%s", i+1,  elem.name);
            }
            AP_Param::convert_old_parameter(&info, 1.0, 0);
        }
    }
}

void AP_AOA::init()
{

    convert_per_instance();

#if ENABLE_PARAMETER
    // if either type is set then enable if not manually set
    if (!_enable.configured() && ((param[0].type.get() != TYPE_NONE) || (param[1].type.get() != TYPE_NONE))) {
        _enable.set_and_save(1);
    }

    // Check if enabled
    if (!lib_enabled()) {
        return;
    }
#endif

    if (enabled(0)) {
        allocate();
    }
}

void AP_AOA::allocate()
{
    if (sensor[0] != nullptr) {
        // already initialised, periph may call allocate several times to allow CAN detection
        return;
    }

#ifdef HAL_AOA_PROBE_LIST
    // load sensors via a list from hwdef.dat
    HAL_AOA_PROBE_LIST;
#else
    // look for sensors based on type parameters
    for (uint8_t i=0; i<AOA_MAX_SENSORS; i++) {
#if AP_AOA_AUTOCAL_ENABLE
        state[i].calibration.init(param[i].ratio);
        state[i].last_saved_ratio = param[i].ratio;
#endif

        // Set the enable automatically to false and set the probability that the AOA is healhy to start with
        state[i].failures.health_probability = 1.0f;

        switch ((enum AOA_type)param[i].type.get()) {
        case TYPE_NONE:
            // nothing to do
            break;
        case TYPE_I2C_MS4525:
#if AP_AOA_MS4525_ENABLED
            sensor[i] = new AP_AOA_MS4525(*this, i);
#endif
            break;
        case TYPE_SITL:
#if AP_AOA_SITL_ENABLED
            sensor[i] = new AP_AOA_SITL(*this, i);
#endif
            break;
        case TYPE_ANALOG:
#if AP_AOA_ANALOG_ENABLED
            sensor[i] = new AP_AOA_Analog(*this, i);
#endif
            break;
        case TYPE_I2C_MS5525:
#if AP_AOA_MS5525_ENABLED
            sensor[i] = new AP_AOA_MS5525(*this, i, AP_AOA_MS5525::MS5525_ADDR_AUTO);
#endif
            break;
        case TYPE_I2C_MS5525_ADDRESS_1:
#if AP_AOA_MS5525_ENABLED
            sensor[i] = new AP_AOA_MS5525(*this, i, AP_AOA_MS5525::MS5525_ADDR_1);
#endif
            break;
        case TYPE_I2C_MS5525_ADDRESS_2:
#if AP_AOA_MS5525_ENABLED
            sensor[i] = new AP_AOA_MS5525(*this, i, AP_AOA_MS5525::MS5525_ADDR_2);
#endif
            break;
        case TYPE_I2C_SDP3X:
#if AP_AOA_SDP3X_ENABLED
            sensor[i] = new AP_AOA_SDP3X(*this, i);
#endif
            break;
        case TYPE_I2C_DLVR_5IN:
#if AP_AOA_DLVR_ENABLED
            sensor[i] = new AP_AOA_DLVR(*this, i, 5);
#endif
            break;
        case TYPE_I2C_DLVR_10IN:
#if AP_AOA_DLVR_ENABLED
            sensor[i] = new AP_AOA_DLVR(*this, i, 10);
#endif
            break;
        case TYPE_I2C_DLVR_20IN:
#if AP_AOA_DLVR_ENABLED
            sensor[i] = new AP_AOA_DLVR(*this, i, 20);
#endif
            break;
        case TYPE_I2C_DLVR_30IN:
#if AP_AOA_DLVR_ENABLED
            sensor[i] = new AP_AOA_DLVR(*this, i, 30);
#endif
            break;
        case TYPE_I2C_DLVR_60IN:
#if AP_AOA_DLVR_ENABLED
            sensor[i] = new AP_AOA_DLVR(*this, i, 60);
#endif  // AP_AOA_DLVR_ENABLED
            break;
        case TYPE_I2C_ASP5033:
#if AP_AOA_ASP5033_ENABLED
            sensor[i] = new AP_AOA_ASP5033(*this, i);
#endif
            break;
        case TYPE_UAVCAN:
#if AP_AOA_UAVCAN_ENABLED
            sensor[i] = AP_AOA_UAVCAN::probe(*this, i, uint32_t(param[i].bus_id.get()));
#endif
            break;
        case TYPE_NMEA_WATER:
#if AP_AOA_NMEA_ENABLED
#if APM_BUILD_TYPE(APM_BUILD_Rover) || APM_BUILD_TYPE(APM_BUILD_ArduSub) 
            sensor[i] = new AP_AOA_NMEA(*this, i);
#endif
#endif
            break;

        }
        if (sensor[i] && !sensor[i]->init()) {
            GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "AOA %u init failed", i + 1);
            delete sensor[i];
            sensor[i] = nullptr;
        }
        if (sensor[i] != nullptr) {
            num_sensors = i+1;
        }
    }

#if AP_AOA_UAVCAN_ENABLED
    // we need a 2nd pass for DroneCAN sensors so we can match order by DEVID
    // the 2nd pass accepts any devid
    for (uint8_t i=0; i<AOA_MAX_SENSORS; i++) {
        if (sensor[i] == nullptr && (enum AOA_type)param[i].type.get() == TYPE_UAVCAN) {
            sensor[i] = AP_AOA_UAVCAN::probe(*this, i, 0);
            if (sensor[i] != nullptr) {
                num_sensors = i+1;
            }
        }
    }
#endif // AP_AOA_UAVCAN_ENABLED
#endif // HAL_AOA_PROBE_LIST

    // set DEVID to zero for any sensors not found. This allows backends to order
    // based on previous value of DEVID. This allows for swapping out sensors
    for (uint8_t i=0; i<AOA_MAX_SENSORS; i++) {
        if (sensor[i] == nullptr) {
            // note we use set() not set_and_save() to allow a sensor to be temporarily
            // removed for one boot without losing its slot
            param[i].bus_id.set(0);
        }
    }
}

// read the AOA sensor
float AP_AOA::get_pressure(uint8_t i)
{
    if (!enabled(i)) {
        return 0;
    }
    float pressure = 0;
    if (sensor[i]) {
        state[i].healthy = sensor[i]->get_differential_pressure(pressure);
    }
    return pressure;
}

// get a temperature reading if possible
bool AP_AOA::get_temperature(uint8_t i, float &temperature)
{
    if (!enabled(i)) {
        return false;
    }
    if (sensor[i]) {
        return sensor[i]->get_temperature(temperature);
    }
    return false;
}

// calibrate the zero offset for the AOA. This must be called at
// least once before the get_AOA() interface can be used
void AP_AOA::calibrate(bool in_startup)
{
#ifndef HAL_BUILD_AP_PERIPH
    if (!lib_enabled()) {
        return;
    }
    if (hal.util->was_watchdog_reset()) {
        GCS_SEND_TEXT(MAV_SEVERITY_INFO,"AOA: skipping cal");
        return;
    }
    for (uint8_t i=0; i<AOA_MAX_SENSORS; i++) {
        if (!enabled(i)) {
            continue;
        }
        if (state[i].use_zero_offset) {
            param[i].offset.set(0);
            continue;
        }
        if (in_startup && param[i].skip_cal) {
            continue;
        }
        if (sensor[i] == nullptr) {
            GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "AOA %u not initalized, cannot cal", i+1);
            continue;
        }
        state[i].cal.start_ms = AP_HAL::millis();
        state[i].cal.count = 0;
        state[i].cal.sum = 0;
        state[i].cal.read_count = 0;
        calibration_state[i] = CalibrationState::IN_PROGRESS;
        GCS_SEND_TEXT(MAV_SEVERITY_INFO,"AOA %u calibration started", i+1);
    }
#endif // HAL_BUILD_AP_PERIPH
}

/*
  update async AOA zero offset calibration
*/
void AP_AOA::update_calibration(uint8_t i, float raw_pressure)
{
#ifndef HAL_BUILD_AP_PERIPH
    if (!enabled(i) || state[i].cal.start_ms == 0) {
        return;
    }
    
    // consider calibration complete when we have at least 15 samples
    // over at least 1 second
    if (AP_HAL::millis() - state[i].cal.start_ms >= 1000 &&
        state[i].cal.read_count > 15) {
        if (state[i].cal.count == 0) {
            GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "AOA %u unhealthy", i + 1);
            calibration_state[i] = CalibrationState::FAILED;
        } else {
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "AOA %u calibrated", i + 1);
            float calibrated_offset = state[i].cal.sum / state[i].cal.count;
            // check if new offset differs too greatly from last calibration, indicating pitot uncovered in wind
            if (fixed_wing_parameters != nullptr) {
                float AOA_min = fixed_wing_parameters->airspeed_min.get();
                // use percentage of AOA_FBW_MIN as criteria for max allowed change in offset
                float max_change = 0.5*(sq((1 + (max_speed_pcnt * 0.01))*AOA_min) - sq(AOA_min));
                if (max_speed_pcnt > 0 && (abs(calibrated_offset-param[i].offset) > max_change) && (abs(param[i].offset) > 0)) {
                    GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "AOA %d offset change large;cover and recal", i +1);
                }
            }
            param[i].offset.set_and_save(calibrated_offset);
            calibration_state[i] = CalibrationState::SUCCESS;
        }
        state[i].cal.start_ms = 0;
        return;
    }
    // we discard the first 5 samples
    if (state[i].healthy && state[i].cal.read_count > 5) {
        state[i].cal.sum += raw_pressure;
        state[i].cal.count++;
    }
    state[i].cal.read_count++;
#endif // HAL_BUILD_AP_PERIPH
}

// get aggregate calibration state for the AOA library:
AP_AOA::CalibrationState AP_AOA::get_calibration_state() const
{
    for (uint8_t i=0; i<AOA_MAX_SENSORS; i++) {
        switch (calibration_state[i]) {
        case CalibrationState::SUCCESS:
        case CalibrationState::NOT_STARTED:
            continue;
        case CalibrationState::IN_PROGRESS:
            return CalibrationState::IN_PROGRESS;
        case CalibrationState::FAILED:
            return CalibrationState::FAILED;
        }
    }
    return CalibrationState::SUCCESS;
}

// read one AOA sensor
void AP_AOA::read(uint8_t i)
{
    if (!enabled(i) || !sensor[i]) {
        return;
    }
    state[i].last_update_ms = AP_HAL::millis();

    // try and get a direct reading of AOA
    if (sensor[i]->has_AOA()) {
        state[i].healthy = sensor[i]->get_AOA(state[i].AOA);
        state[i].raw_AOA = state[i].AOA;  // for logging
        return;
    }

    float raw_pressure = get_pressure(i);
    float AOA_pressure = raw_pressure - get_offset(i); // AOA increase -> AOA_pressure decrease (first tube is on upside, second tube is on downside)

    // remember raw pressure for logging
    state[i].corrected_pressure = AOA_pressure;

#ifndef HAL_BUILD_AP_PERIPH
    bool prev_healthy = state[i].healthy;
    if (state[i].cal.start_ms != 0) {
        update_calibration(i, raw_pressure);
    }

    // filter before clamping positive
    if (!prev_healthy) {
        // if the previous state was not healthy then we should not
        // use an IIR filter, otherwise a bad reading will last for
        // some time after the sensor becomees healthy again
        state[i].filtered_pressure = AOA_pressure;
    } else {
        //state[i].filtered_pressure = AOA_pressure;
        state[i].filtered_pressure = 0.7f * state[i].filtered_pressure + 0.3f * AOA_pressure;
    }

    if (!_airspeed.healthy()) {
        return;
    } else {
        float pressure_as = MAX(fabsf(_airspeed.get_corrected_pressure()), 10); // avoid 0 division
        float pressure_ratio = constrain_float((AOA_pressure / pressure_as), -2.0f, 2.0f);
        float pressure_as_f = MAX(fabsf(_airspeed.get_filtered_pressure()), 10); // avoid 0 division
        float pressure_ratio_f = constrain_float((state[i].filtered_pressure / pressure_as_f), -2.0f, 2.0f);

        switch ((enum pitot_tube_order)param[i].tube_order.get()) {
        case PITOT_TUBE_ORDER_NEGATIVE:
            state[i].last_pressure = -AOA_pressure;
            state[i].raw_AOA = degrees(atanf(pressure_ratio * 0.5f));
            state[i].AOA = degrees(atanf(pressure_ratio_f * 0.5f));
            break;
        case PITOT_TUBE_ORDER_POSITIVE:
        default:
            state[i].last_pressure = AOA_pressure;
            state[i].raw_AOA = degrees(atanf(-pressure_ratio * 0.5f));
            state[i].AOA = degrees(atanf(-pressure_ratio_f * 0.5f));
            break;
        }
    }

    /*
      we support different pitot tube setups so user can choose if
      they want to be able to detect pressure on the static port
     */
    /*
    switch ((enum pitot_tube_order)param[i].tube_order.get()) {
    case PITOT_TUBE_ORDER_NEGATIVE:
        state[i].last_pressure  = -AOA_pressure;
        state[i].raw_AOA   = sqrtf(MAX(-AOA_pressure, 0) * param[i].ratio);
        state[i].AOA       = sqrtf(MAX(-state[i].filtered_pressure, 0) * param[i].ratio);
        break;
    case PITOT_TUBE_ORDER_POSITIVE:
        state[i].last_pressure  = AOA_pressure;
        state[i].raw_AOA   = sqrtf(MAX(AOA_pressure, 0) * param[i].ratio);
        state[i].AOA       = sqrtf(MAX(state[i].filtered_pressure, 0) * param[i].ratio);
        break;
    case PITOT_TUBE_ORDER_AUTO:
    default:
        state[i].last_pressure  = fabsf(AOA_pressure);
        state[i].raw_AOA   = sqrtf(fabsf(AOA_pressure) * param[i].ratio);
        state[i].AOA       = sqrtf(fabsf(state[i].filtered_pressure) * param[i].ratio);
        break;
    }
    */
#endif // HAL_BUILD_AP_PERIPH
}

// read all AOA sensors
void AP_AOA::update()
{
    if (!lib_enabled()) {
        return;
    }

    for (uint8_t i=0; i<AOA_MAX_SENSORS; i++) {
        read(i);
    }

#if HAL_GCS_ENABLED
    // debugging until we get MAVLink support for 2nd AOA sensor
    if (enabled(1)) {
        gcs().send_named_float("AS2", get_AOA(1));
    }
#endif

#if HAL_LOGGING_ENABLED
    const uint8_t old_primary = primary;
#endif

    // setup primary
    if (healthy(primary_sensor.get())) {
        primary = primary_sensor.get();
    } else {
        for (uint8_t i=0; i<AOA_MAX_SENSORS; i++) {
            if (healthy(i)) {
                primary = i;
                break;
            }
        }
    }

    check_sensor_failures();

#if HAL_LOGGING_ENABLED
    if (primary != old_primary) {
        AP::logger().Write_Event(LogEvent::AOA_PRIMARY_CHANGED);
    }
    //if (_log_bit != (uint32_t)-1 && AP::logger().should_log(_log_bit)) {
        Log_AOA();
    //}
#endif
}



// @LoggerMessage: HYGR
// @Description: Hygrometer data
// @Field: TimeUS: Time since system startup
// @Field: Id: sensor ID
// @Field: Humidity: percentage humidity
// @Field: Temp: temperature in degrees C

void AP_AOA::Log_AOA()
{
    const uint64_t now = AP_HAL::micros64();
    for (uint8_t i=0; i<AOA_MAX_SENSORS; i++) {
        if (!enabled(i) || sensor[i] == nullptr) {
            continue;
        }
        float temperature;
        if (!get_temperature(i, temperature)) {
            temperature = 0;
        }
        const struct log_AOA pkt{
            LOG_PACKET_HEADER_INIT(LOG_AOA_MSG),
            time_us       : now,
            instance      : i,
            AOA      : get_raw_AOA(i),
            diffpressure  : get_differential_pressure(i),
            temperature   : (int16_t)(temperature * 100.0f),
            rawpressure   : get_corrected_pressure(i),
            offset        : get_offset(i),
            use           : use(i),
            healthy       : healthy(i),
            health_prob   : get_health_probability(i),
            test_ratio    : get_test_ratio(i),
            primary       : get_primary()
        };
        AP::logger().WriteBlock(&pkt, sizeof(pkt));

#if AP_AOA_HYGROMETER_ENABLE
        struct {
            uint32_t sample_ms;
            float temperature;
            float humidity;
        } hygrometer;
        if (sensor[i]->get_hygrometer(hygrometer.sample_ms, hygrometer.temperature, hygrometer.humidity) &&
            hygrometer.sample_ms != state[i].last_hygrometer_log_ms) {
            AP::logger().WriteStreaming("HYGR",
                                        "TimeUS,Id,Humidity,Temp",
                                        "s#%O",
                                        "F---",
                                        "QBff",
                                        AP_HAL::micros64(),
                                        i,
                                        hygrometer.humidity,
                                        hygrometer.temperature);
            state[i].last_hygrometer_log_ms = hygrometer.sample_ms;
        }
#endif
    }
}

bool AP_AOA::use(uint8_t i) const
{
#ifndef HAL_BUILD_AP_PERIPH
    if (!lib_enabled()) {
        return false;
    }
    if (_force_disable_use) {
        return false;
    }
    if (!enabled(i) || !param[i].use) {
        return false;
    }
    if (param[i].use == 2 && !is_zero(SRV_Channels::get_output_scaled(SRV_Channel::k_throttle))) {
        // special case for gliders with AOA sensors behind the
        // propeller. Allow AOA to be disabled when throttle is
        // running
        return false;
    }
    return true;
#else
    return false;
#endif // HAL_BUILD_AP_PERIPH
}

/*
  return true if all enabled sensors are healthy
 */
bool AP_AOA::all_healthy(void) const
{
    for (uint8_t i=0; i<AOA_MAX_SENSORS; i++) {
        if (enabled(i) && !healthy(i)) {
            return false;
        }
    }
    return true;
}

bool AP_AOA::lib_enabled() const {
#if ENABLE_PARAMETER
    return _enable > 0;
#endif
    return true;
}

// return true if AOA is enabled
bool AP_AOA::enabled(uint8_t i) const {
    if (!lib_enabled()) {
        return false;
    }
    if (i < AOA_MAX_SENSORS) {
        return param[i].type.get() != TYPE_NONE;
    }
    return false;
}

// return health status of sensor
bool AP_AOA::healthy(uint8_t i) const {
    bool ok = state[i].healthy && enabled(i) && sensor[i] != nullptr;
#ifndef HAL_BUILD_AP_PERIPH
    // sanity check the offset parameter.  Zero is permitted if we are skipping calibration.
    ok &= (fabsf(param[i].offset) > 0 || state[i].use_zero_offset || param[i].skip_cal);
#endif
    return ok;
}

// return the current AOA in m/s
float AP_AOA::get_AOA(uint8_t i) const {
    if (!enabled(i)) {
        // we can't have negative AOA so sending an obviously invalid value
        return -1.0;
    }
    return state[i].AOA;
}

// return the unfiltered AOA in m/s
float AP_AOA::get_raw_AOA(uint8_t i) const {
    if (!enabled(i)) {
        // we can't have negative AOA so sending an obviously invalid value
        return -1.0;
    }
    return state[i].raw_AOA;
}

// return the differential pressure in Pascal for the last AOA reading
float AP_AOA::get_differential_pressure(uint8_t i) const {
    if (!enabled(i)) {
        return 0.0;
    }
    return state[i].last_pressure;
}

// return the current corrected pressure
float AP_AOA::get_corrected_pressure(uint8_t i) const {
    if (!enabled(i)) {
        return 0.0;
    }
    return state[i].corrected_pressure;
}

#if AP_AOA_HYGROMETER_ENABLE
bool AP_AOA::get_hygrometer(uint8_t i, uint32_t &last_sample_ms, float &temperature, float &humidity) const
{
    if (!enabled(i) || sensor[i] == nullptr) {
        return false;
    }
    return sensor[i]->get_hygrometer(last_sample_ms, temperature, humidity);
}
#endif // AP_AOA_HYGROMETER_ENABLE

#else  // build type is not appropriate; provide a dummy implementation:
const AP_Param::GroupInfo AP_AOA::var_info[] = { AP_GROUPEND };

void AP_AOA::update() {};
bool AP_AOA::get_temperature(uint8_t i, float &temperature) { return false; }
void AP_AOA::calibrate(bool in_startup) {}
AP_AOA::CalibrationState AP_AOA::get_calibration_state() const { return CalibrationState::NOT_STARTED; }
bool AP_AOA::use(uint8_t i) const { return false; }
bool AP_AOA::enabled(uint8_t i) const { return false; }
bool AP_AOA::healthy(uint8_t i) const { return false; }
float AP_AOA::get_AOA(uint8_t i) const { return 0.0; }
float AP_AOA::get_differential_pressure(uint8_t i) const { return 0.0; }



bool AP_AOA::all_healthy(void) const { return false; }
void AP_AOA::init(void) {};
AP_AOA::AP_AOA() {}

#endif // #if AP_AOA_DUMMY_METHODS_ENABLED

// singleton instance
AP_AOA *AP_AOA::_singleton;

namespace AP {

AP_AOA *AOA()
{
    return AP_AOA::get_singleton();
}

};
