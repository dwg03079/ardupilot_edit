#pragma once

#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL_Boards.h>
//#include <AP_MSP/msp.h>

#ifndef AP_AOA_ENABLED
#define AP_AOA_ENABLED 1
#endif



#ifndef AOA_MAX_SENSORS
#define AOA_MAX_SENSORS 2
#endif

#ifndef AP_AOA_AUTOCAL_ENABLE
#define AP_AOA_AUTOCAL_ENABLE AP_AOA_ENABLED
#endif

#ifndef AP_AOA_HYGROMETER_ENABLE
#define AP_AOA_HYGROMETER_ENABLE (AP_AOA_ENABLED && BOARD_FLASH_SIZE > 1024)
#endif
