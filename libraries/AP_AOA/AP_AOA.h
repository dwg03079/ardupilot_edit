#pragma once

#include "AP_AOA_config.h"

#include <AP_Param/AP_Param.h>
#include <AP_Math/AP_Math.h>
#include <AP_AHRS/AP_AHRS.h>
#include <AP_Airspeed/AP_Airspeed.h>

class AP_AOA_Backend;

class AP_AOA_Params {
public:
    // Constructor
    AP_AOA_Params(void);

    // parameters for each instance
    AP_Int32 bus_id;
#ifndef HAL_BUILD_AP_PERIPH
    AP_Float offset;
    AP_Float ratio;
#endif
    AP_Float psi_range;
#ifndef HAL_BUILD_AP_PERIPH
    AP_Int8  use;
    AP_Int8  pin;
    AP_Int8  skip_cal;
    AP_Int8  tube_order;
#endif
    AP_Int8  type;
    AP_Int8  bus;
#if AP_AOA_AUTOCAL_ENABLE
    AP_Int8  autocal;
#endif

    static const struct AP_Param::GroupInfo var_info[];
};


class AOA_Calibration {
public:
    friend class AP_AOA;
    // constructor
    AOA_Calibration();

    // initialise the calibration
    void init(float initial_ratio);

    // take current AOA in m/s and ground speed vector and return
    // new scaling factor
    float update(float AOA, const Vector3f &vg, int16_t max_AOA_allowed_during_cal);

private:
    // state of kalman filter for AOA ratio estimation
    Matrix3f P; // covarience matrix
    const float Q0; // process noise matrix top left and middle element
    const float Q1; // process noise matrix bottom right element
    Vector3f state; // state vector
    const float DT; // time delta
};

class AP_AOA
{
public:
    friend class AP_AOA_Backend;
    
    // constructor
    AP_AOA(AP_AHRS& ahrs, AP_Airspeed& airspeed);

    void set_fixedwing_parameters(const class AP_FixedWing *_fixed_wing_parameters);

    void init(void);
    void allocate();


    // indicate which bit in LOG_BITMASK indicates we should log AOA readings
    void set_log_bit(uint32_t log_bit) { _log_bit = log_bit; }

#if AP_AOA_AUTOCAL_ENABLE
    // inflight ratio calibration
    void set_calibration_enabled(bool enable) {calibration_enabled = enable;}
#endif //AP_AOA_AUTOCAL_ENABLE

    // read the analog source and update AOA
    void update(void);

    // calibrate the AOA. This must be called on startup if the
    // altitude/climb_rate/acceleration interfaces are ever used
    void calibrate(bool in_startup);

    // return the current AOA in m/s
    float get_AOA(uint8_t i) const;
    float get_AOA(void) const { return get_AOA(primary); }

    // return the unfiltered AOA in m/s
    float get_raw_AOA(uint8_t i) const;
    float get_raw_AOA(void) const { return get_raw_AOA(primary); }

    // return the current AOA ratio (dimensionless)
    float get_AOA_ratio(uint8_t i) const {
#ifndef HAL_BUILD_AP_PERIPH
        return param[i].ratio;
#else
        return 0.0;
#endif
    }
    float get_AOA_ratio(void) const { return get_AOA_ratio(primary); }

    // get temperature if available
    bool get_temperature(uint8_t i, float &temperature);
    bool get_temperature(float &temperature) { return get_temperature(primary, temperature); }

    // set the AOA ratio (dimensionless)
#ifndef HAL_BUILD_AP_PERIPH
    void set_AOA_ratio(uint8_t i, float ratio) {
        param[i].ratio.set(ratio);
    }
    void set_AOA_ratio(float ratio) { set_AOA_ratio(primary, ratio); }
#endif

    // return true if AOA is enabled, and AOA use is set
    bool use(uint8_t i) const;
    bool use(void) const { return use(primary); }

    // force disabling of all AOA sensors
    void force_disable_use(bool value) {
        _force_disable_use = value;
    }

    // return true if AOA is enabled
    bool enabled(uint8_t i) const;
    bool enabled(void) const { return enabled(primary); }

    // return the differential pressure in Pascal for the last AOA reading
    float get_differential_pressure(uint8_t i) const;
    float get_differential_pressure(void) const { return get_differential_pressure(primary); }

    // update AOA ratio calibration
    void update_calibration(const Vector3f &vground, int16_t max_AOA_allowed_during_cal);

    // return health status of sensor
    bool healthy(uint8_t i) const;
    bool healthy(void) const { return healthy(primary); }

    // return true if all enabled sensors are healthy
    bool all_healthy(void) const;
    
    // return time in ms of last update
    uint32_t last_update_ms(uint8_t i) const { return state[i].last_update_ms; }
    uint32_t last_update_ms(void) const { return last_update_ms(primary); }

#if AP_AOA_HYGROMETER_ENABLE
    bool get_hygrometer(uint8_t i, uint32_t &last_sample_ms, float &temperature, float &humidity) const;
#endif

    static const struct AP_Param::GroupInfo var_info[];

    enum pitot_tube_order { PITOT_TUBE_ORDER_POSITIVE = 0,
                            PITOT_TUBE_ORDER_NEGATIVE = 1 };

    enum OptionsMask {
        ON_FAILURE_AHRS_WIND_MAX_DO_DISABLE                   = (1<<0),   // If set then use AOA failure check
        ON_FAILURE_AHRS_WIND_MAX_RECOVERY_DO_REENABLE         = (1<<1),   // If set then automatically enable the AOA sensor use when healthy again.
        DISABLE_VOLTAGE_CORRECTION                            = (1<<2),
        USE_EKF_CONSISTENCY                                   = (1<<3),
    };

    enum AOA_type {
        TYPE_NONE=0,
        TYPE_I2C_MS4525=1,
        TYPE_ANALOG=2,
        TYPE_I2C_MS5525=3,
        TYPE_I2C_MS5525_ADDRESS_1=4,
        TYPE_I2C_MS5525_ADDRESS_2=5,
        TYPE_I2C_SDP3X=6,
        TYPE_I2C_DLVR_5IN=7,
        TYPE_UAVCAN=8,
        TYPE_I2C_DLVR_10IN=9,
        TYPE_I2C_DLVR_20IN=10,
        TYPE_I2C_DLVR_30IN=11,
        TYPE_I2C_DLVR_60IN=12,
        TYPE_NMEA_WATER=13,

        TYPE_I2C_ASP5033=15,
        TYPE_SITL=100,
    };

    // get current primary sensor
    uint8_t get_primary(void) const { return primary; }

    // get number of sensors
    uint8_t get_num_sensors(void) const { return num_sensors; }
    
    static AP_AOA *get_singleton() { return _singleton; }

    // return the current corrected pressure, public for AP_Periph
    float get_corrected_pressure(uint8_t i) const;
    float get_corrected_pressure(void) const {
        return get_corrected_pressure(primary);
    }



    enum class CalibrationState {
        NOT_STARTED,
        IN_PROGRESS,
        SUCCESS,
        FAILED
    };
    // get aggregate calibration state for the AOA library:
    CalibrationState get_calibration_state() const;

    float ahrs_AOA() { return _ahrs.getAOA(); }

private:

    AP_AHRS &_ahrs;
    AP_Airspeed &_airspeed;

    static AP_AOA *_singleton;

    AP_Int8 _enable;
    bool lib_enabled() const;

    AP_Int8 primary_sensor;
    AP_Int8 max_speed_pcnt;
    AP_Int32 _options;    // bitmask options for AOA
    AP_Float _wind_max;
    AP_Float _wind_warn;
    AP_Float _wind_gate;

    AP_AOA_Params param[AOA_MAX_SENSORS];

    CalibrationState calibration_state[AOA_MAX_SENSORS];

    struct AOA_state {
        float   raw_AOA;
        float   AOA;
        float	last_pressure;
        float   filtered_pressure;
        float	corrected_pressure;
        uint32_t last_update_ms;
        bool use_zero_offset;
        bool	healthy;

        // state of runtime calibration
        struct {
            uint32_t start_ms;
            float    sum;
            uint16_t count;
            uint16_t read_count;
        } cal;

#if AP_AOA_AUTOCAL_ENABLE
        AOA_Calibration calibration;
        float last_saved_ratio;
        uint8_t counter;
#endif // AP_AOA_AUTOCAL_ENABLE

        struct {
            uint32_t last_check_ms;
            float health_probability;
            float test_ratio;
            int8_t param_use_backup;
            uint32_t last_warn_ms;
        } failures;

#if AP_AOA_HYGROMETER_ENABLE
        uint32_t last_hygrometer_log_ms;
#endif
    } state[AOA_MAX_SENSORS];

    bool calibration_enabled;

    // can be set to true to disable the use of the AOA sensor
    bool _force_disable_use;

    // current primary sensor
    uint8_t primary;
    uint8_t num_sensors;

    uint32_t _log_bit = -1;     // stores which bit in LOG_BITMASK is used to indicate we should log AOA readings

    void read(uint8_t i);
    // return the differential pressure in Pascal for the last AOA reading for the requested instance
    // returns 0 if the sensor is not enabled
    float get_pressure(uint8_t i);

    // get the health probability
    float get_health_probability(uint8_t i) const {
        return state[i].failures.health_probability;
    }
    float get_health_probability(void) const {
        return get_health_probability(primary);
    }

    // get the consistency test ratio
    float get_test_ratio(uint8_t i) const {
        return state[i].failures.test_ratio;
    }
    float get_test_ratio(void) const {
        return get_test_ratio(primary);
    }

    void update_calibration(uint8_t i, float raw_pressure);
    void update_calibration(uint8_t i, const Vector3f &vground, int16_t max_AOA_allowed_during_cal);
    void send_AOA_calibration(const Vector3f &vg);
    // return the current calibration offset
    float get_offset(uint8_t i) const {
#ifndef HAL_BUILD_AP_PERIPH
        return param[i].offset;
#else
        return 0.0;
#endif
    }
    float get_offset(void) const { return get_offset(primary); }

    void check_sensor_failures();
    void check_sensor_ahrs_wind_max_failures(uint8_t i);

    AP_AOA_Backend *sensor[AOA_MAX_SENSORS];

    void Log_AOA();

    bool add_backend(AP_AOA_Backend *backend);
    
    const AP_FixedWing *fixed_wing_parameters;

    void convert_per_instance();

};

namespace AP {
    AP_AOA *AOA();
};
