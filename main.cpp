#include "SunS_BH1730FVC.h"
#include "boards.h"
#include "Serial.h"
#include <util/delay.h>
#include "array.h"
#include "DigitalIO.h"
#include "LED.h"
#include "SunS_LM60.h"
#include "Analog.h"
#include "SunS_RTD.h"
#include "RTD.h"
#include "TWISlaveRegisterAccess.h"


using namespace hal;
using namespace bsp;

/* REQS for APP:

- ALS
- RTD
- LM60
- I2C communication
- UART debugg & CLI
- Watchdog
- */


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
    volatile uint8_t registerMapArray[89];
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
volatile uint8_t ALS_GAIN = 1;
volatile bool TRIGGER = false;

enum StatusReg {
    ERROR = 1,
    NEW_DATA = 2
};

constexpr uint8_t registers_len = 89;

void hal::TWISlaveRegisterAccess::callbackRx() {
    if (2 == TWISlaveRegisterAccess::rx_buffer_cnt) {
            ALS_INTEGRATION_TIME = TWISlaveRegisterAccess::rx_buffer[0];
            ALS_GAIN = TWISlaveRegisterAccess::rx_buffer[1];
            TRIGGER = true;
    } else {
        // error in communication
        registers.registerMap.STATUS |= StatusReg::ERROR;
    }
}

void hal::TWISlaveRegisterAccess::callbackTx() {
    registers.registerMap.STATUS = 0;
}


int main() {
    Serial0.init(9600);
    InternalADC::init(InternalADC::Prescaler::DIV_128, ADC_REFERENCE_TYPE);
    hal::TWISlaveRegisterAccess::init(0x1E);
    TWISlaveRegisterAccess::tx_buffer_start = registers.registerMapArray;
    TWISlaveRegisterAccess::tx_buffer_max = registers_len;
    sei();

    SunS_LM60 lm(TEMP_BOARD, 16, ADC_REFERENCE_VALUE);

    SunS_RTD RTD_A(RTD_AIN_A, 16, RTD_REFERENCE_RESISTANCE, ADC_REFERENCE_VALUE);
    SunS_RTD RTD_B(RTD_AIN_B, 16, RTD_REFERENCE_RESISTANCE, ADC_REFERENCE_VALUE);
    SunS_RTD RTD_C(RTD_AIN_C, 16, RTD_REFERENCE_RESISTANCE, ADC_REFERENCE_VALUE);
    SunS_RTD RTD_D(RTD_AIN_D, 16, RTD_REFERENCE_RESISTANCE, ADC_REFERENCE_VALUE);

    SunS_BH1730FVC <pins::SCL_1, pins::SDA_A1, pins::SDA_B1, pins::SDA_C1, pins::SDA_D1> ALS_1;
    SunS_BH1730FVC <pins::SCL_2, pins::SDA_A2, pins::SDA_B2, pins::SDA_C2, pins::SDA_D2> ALS_2;
    SunS_BH1730FVC <pins::SCL_3, pins::SDA_A3, pins::SDA_B3, pins::SDA_C3, pins::SDA_D3> ALS_3;


    uint16_t raw;
    libs::array<uint16_t, 4> dataVL, dataIR;
    libs::array<uint8_t, 4> ids;

    while (true) {
        if (true == TRIGGER) {
            TRIGGER = false;

            if ((ALS_INTEGRATION_TIME > 1) && (ALS_GAIN <= 3)) {

                // control
                registers.registerMap.AZIMUTH_ANGLE++;
            
                raw = RTD_A.measure();
                registers.registerMap.TEMPERATURE_A_RAW = raw;
                registers.registerMap.TEMPERATURE_A = RTD_A.temperature(raw);

                raw = RTD_B.measure();
                registers.registerMap.TEMPERATURE_B_RAW = raw;
                registers.registerMap.TEMPERATURE_B = RTD_B.temperature(raw);

                raw = RTD_C.measure();
                registers.registerMap.TEMPERATURE_C_RAW = raw;
                registers.registerMap.TEMPERATURE_C = RTD_C.temperature(raw);

                raw = RTD_D.measure();
                registers.registerMap.TEMPERATURE_D_RAW = raw;
                registers.registerMap.TEMPERATURE_D = RTD_D.temperature(raw);


                ids = ALS_1.readPartID();
                registers.registerMap.ALS_1A_ID = ids[0];
                registers.registerMap.ALS_1B_ID = ids[1];
                registers.registerMap.ALS_1C_ID = ids[2];
                registers.registerMap.ALS_1D_ID = ids[3];

                ALS_1.setGain(static_cast<SunS_BH1730FVC_Types::Gain>(ALS_GAIN));
                ALS_1.setIntegrationTime(ALS_INTEGRATION_TIME);       

                ids = ALS_2.readPartID();
                registers.registerMap.ALS_2A_ID = ids[0];
                registers.registerMap.ALS_2B_ID = ids[1];
                registers.registerMap.ALS_2C_ID = ids[2];
                registers.registerMap.ALS_2D_ID = ids[3];

                ALS_2.setGain(static_cast<SunS_BH1730FVC_Types::Gain>(ALS_GAIN));
                ALS_2.setIntegrationTime(ALS_INTEGRATION_TIME);

                ids = ALS_3.readPartID();
                registers.registerMap.ALS_3A_ID = ids[0];
                registers.registerMap.ALS_3B_ID = ids[1];
                registers.registerMap.ALS_3C_ID = ids[2];
                registers.registerMap.ALS_3D_ID = ids[3];

                ALS_3.setGain(static_cast<SunS_BH1730FVC_Types::Gain>(ALS_GAIN));
                ALS_3.setIntegrationTime(ALS_INTEGRATION_TIME);

                ALS_1.setMeasurement(SunS_BH1730FVC_Types::ONE_SHOT, SunS_BH1730FVC_Types::VL_IR);
                ALS_2.setMeasurement(SunS_BH1730FVC_Types::ONE_SHOT, SunS_BH1730FVC_Types::VL_IR);
                ALS_3.setMeasurement(SunS_BH1730FVC_Types::ONE_SHOT, SunS_BH1730FVC_Types::VL_IR);

                // wait for conversion
                for (uint8_t i = 0; i < ALS_INTEGRATION_TIME; i++) {
                    _delay_ms(3);
                }
                _delay_ms(2);

                ALS_1.ambientLightRAW(dataVL, dataIR);
                registers.registerMap.ALS_1A_VL_RAW = dataVL[0];
                registers.registerMap.ALS_1B_VL_RAW = dataVL[1];
                registers.registerMap.ALS_1C_VL_RAW = dataVL[2];
                registers.registerMap.ALS_1D_VL_RAW = dataVL[3];

                registers.registerMap.ALS_1A_IR_RAW = dataIR[0];
                registers.registerMap.ALS_1B_IR_RAW = dataIR[1];
                registers.registerMap.ALS_1C_IR_RAW = dataIR[2];
                registers.registerMap.ALS_1D_IR_RAW = dataIR[3];

                ALS_2.ambientLightRAW(dataVL, dataIR);
                registers.registerMap.ALS_2A_VL_RAW = dataVL[0];
                registers.registerMap.ALS_2B_VL_RAW = dataVL[1];
                registers.registerMap.ALS_2C_VL_RAW = dataVL[2];
                registers.registerMap.ALS_2D_VL_RAW = dataVL[3];

                registers.registerMap.ALS_2A_IR_RAW = dataIR[0];
                registers.registerMap.ALS_2B_IR_RAW = dataIR[1];
                registers.registerMap.ALS_2C_IR_RAW = dataIR[2];
                registers.registerMap.ALS_2D_IR_RAW = dataIR[3];

                ALS_3.ambientLightRAW(dataVL, dataIR);
                registers.registerMap.ALS_3A_VL_RAW = dataVL[0];
                registers.registerMap.ALS_3B_VL_RAW = dataVL[1];
                registers.registerMap.ALS_3C_VL_RAW = dataVL[2];
                registers.registerMap.ALS_3D_VL_RAW = dataVL[3];

                registers.registerMap.ALS_2A_IR_RAW = dataIR[0];
                registers.registerMap.ALS_2B_IR_RAW = dataIR[1];
                registers.registerMap.ALS_2C_IR_RAW = dataIR[2];
                registers.registerMap.ALS_2D_IR_RAW = dataIR[3];

                uint16_t lm60_raw = lm.measure();
                registers.registerMap.TEMPERATURE_STRUCT = lm.temperature(lm60_raw);
                registers.registerMap.TEMPERATURE_STRUCT_RAW = lm60_raw;
                // new data
                registers.registerMap.STATUS |= StatusReg::NEW_DATA;
            } else {
                registers.registerMap.STATUS |= StatusReg::ERROR;
            }
        }
    }
}
