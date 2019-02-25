#ifndef SUNS_APP_INC_TWIREGISTERINTERFACE_H_
#define SUNS_APP_INC_TWIREGISTERINTERFACE_H_

#include <stdint.h>
#include "TWISlaveRegisterAccess.h"

namespace TWIRegisterInterface {

enum StatusReg {
    ERROR = 1,
    NEW_DATA = 2
};

#pragma pack(push, 1)
union registerDesc {
    struct {
        uint8_t STATUS;
        uint8_t WHO_AM_I;
        uint16_t AZIMUTH_ANGLE;
        uint16_t ELEVATION_ANGLE;
        int16_t TEMPERATURE_A;
        int16_t TEMPERATURE_B;
        int16_t TEMPERATURE_C;
        int16_t TEMPERATURE_D;
        int16_t TEMPERATURE_STRUCT;
        uint16_t ALS_1A_VL_RAW;
        uint16_t ALS_1B_VL_RAW;
        uint16_t ALS_1C_VL_RAW;
        uint16_t ALS_1D_VL_RAW;
        uint16_t ALS_2A_VL_RAW;
        uint16_t ALS_2B_VL_RAW;
        uint16_t ALS_2C_VL_RAW;
        uint16_t ALS_2D_VL_RAW;
        uint16_t ALS_3A_VL_RAW;
        uint16_t ALS_3B_VL_RAW;
        uint16_t ALS_3C_VL_RAW;
        uint16_t ALS_3D_VL_RAW;
        uint16_t ALS_1A_IR_RAW;
        uint16_t ALS_1B_IR_RAW;
        uint16_t ALS_1C_IR_RAW;
        uint16_t ALS_1D_IR_RAW;
        uint16_t ALS_2A_IR_RAW;
        uint16_t ALS_2B_IR_RAW;
        uint16_t ALS_2C_IR_RAW;
        uint16_t ALS_2D_IR_RAW;
        uint16_t ALS_3A_IR_RAW;
        uint16_t ALS_3B_IR_RAW;
        uint16_t ALS_3C_IR_RAW;
        uint16_t ALS_3D_IR_RAW;
        uint16_t TEMPERATURE_A_RAW;
        uint16_t TEMPERATURE_B_RAW;
        uint16_t TEMPERATURE_C_RAW;
        uint16_t TEMPERATURE_D_RAW;
        uint16_t TEMPERATURE_STRUCT_RAW;
        uint16_t ALS_STATUS;
        uint8_t ALS_1A_ID;
        uint8_t ALS_1B_ID;
        uint8_t ALS_1C_ID;
        uint8_t ALS_1D_ID;
        uint8_t ALS_2A_ID;
        uint8_t ALS_2B_ID;
        uint8_t ALS_2C_ID;
        uint8_t ALS_2D_ID;
        uint8_t ALS_3A_ID;
        uint8_t ALS_3B_ID;
        uint8_t ALS_3C_ID;
        uint8_t ALS_3D_ID;
    } registerMap;
    volatile uint8_t registerMapArray[88];
};
#pragma pack(pop) 



volatile registerDesc registers = {
    {
        0x00,
        0xC2,
        0x00,
        0x00,
        0x00,
        0x00,
        0x00,
        0x00,
        0x00,
        0x00,
        0x00,
        0x00,
        0x00,
        0x00,
        0x00,
        0x00,
        0x00,
        0x00,
        0x00,
        0x00,
        0x00,
        0x00,
        0x00,
        0x00,
        0x00,
        0x00,
        0x00,
        0x00,
        0x00,
        0x00,
        0x00,
        0x00,
        0x00,
        0x00,
        0x00,
        0x00,
        0x00,
        0x00,
        0x00,
        0x00,
        0x00,
        0x00,
        0x00,
        0x00,
        0x00,
        0x00,
        0x00,
        0x00,
        0x00,
        0x00,
        0x00
    }
};

volatile uint8_t ALS_INTEGRATION_TIME = 38;
volatile uint8_t ALS_GAIN = 0;
volatile bool TRIGGER = false;

constexpr uint8_t REGISTERS_LEN = 88;

} // namespace TWIRegisterInterface

void hal::TWISlaveRegisterAccess::callbackRx() {
    if (2 == TWISlaveRegisterAccess::rx_buffer_cnt) {
            TWIRegisterInterface::ALS_INTEGRATION_TIME = TWISlaveRegisterAccess::rx_buffer[0];
            TWIRegisterInterface::ALS_GAIN = TWISlaveRegisterAccess::rx_buffer[1];
            TWIRegisterInterface::TRIGGER = true;
    } else {
        // error in communication
        TWIRegisterInterface::registers.registerMap.STATUS |= TWIRegisterInterface::StatusReg::ERROR;
    }
}

void hal::TWISlaveRegisterAccess::callbackTx() {
    TWIRegisterInterface::registers.registerMap.STATUS = 0;
}

#endif  // SUNS_APP_INC_TWIREGISTERINTERFACE_H_
