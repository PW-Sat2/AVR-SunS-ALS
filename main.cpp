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
#include "TWIRegisterInterface.h"
#include "TWISlaveRegisterAccess.h"

using namespace hal;
using namespace bsp;


void waitForALS() {
    for (uint8_t i = 0; i < TWIRegisterInterface::ALS_INTEGRATION_TIME; i++) {
        _delay_ms(3);
    }
    _delay_ms(2);
}

int main() {
    // for debug only
    Serial0.init(9600);

    InternalADC::init(InternalADC::Prescaler::DIV_128, ADC_REFERENCE_TYPE);

    hal::TWISlaveRegisterAccess::init(0x1E);
    TWISlaveRegisterAccess::tx_buffer_start = TWIRegisterInterface::registers.registerMapArray;
    TWISlaveRegisterAccess::tx_buffer_max = TWIRegisterInterface::REGISTERS_LEN;
    sei();

    SunS_LM60 LM60(TEMP_BOARD, 16, ADC_REFERENCE_VALUE);

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
        if (true == TWIRegisterInterface::TRIGGER) {    
            TWIRegisterInterface::TRIGGER = false;
            if ((TWIRegisterInterface::ALS_INTEGRATION_TIME > 1) && (TWIRegisterInterface::ALS_GAIN <= 3)) {

                uint8_t ALS_1_status = 0;
                uint8_t ALS_2_status = 0;
                uint8_t ALS_3_status = 0;
                uint16_t ALS_collective_status = 0;

                // for debug purposes only, since azimuth angle is not updated in any way
                TWIRegisterInterface::registers.registerMap.AZIMUTH_ANGLE++;
            
                raw = RTD_A.measure();
                TWIRegisterInterface::registers.registerMap.TEMPERATURE_A_RAW = raw;
                TWIRegisterInterface::registers.registerMap.TEMPERATURE_A = RTD_A.temperature(raw);

                raw = RTD_B.measure();
                TWIRegisterInterface::registers.registerMap.TEMPERATURE_B_RAW = raw;
                TWIRegisterInterface::registers.registerMap.TEMPERATURE_B = RTD_B.temperature(raw);

                raw = RTD_C.measure();
                TWIRegisterInterface::registers.registerMap.TEMPERATURE_C_RAW = raw;
                TWIRegisterInterface::registers.registerMap.TEMPERATURE_C = RTD_C.temperature(raw);

                raw = RTD_D.measure();
                TWIRegisterInterface::registers.registerMap.TEMPERATURE_D_RAW = raw;
                TWIRegisterInterface::registers.registerMap.TEMPERATURE_D = RTD_D.temperature(raw);


                ids = ALS_1.readPartID();
                TWIRegisterInterface::registers.registerMap.ALS_1A_ID = ids[0];
                TWIRegisterInterface::registers.registerMap.ALS_1B_ID = ids[1];
                TWIRegisterInterface::registers.registerMap.ALS_1C_ID = ids[2];
                TWIRegisterInterface::registers.registerMap.ALS_1D_ID = ids[3];

                ALS_1_status |= ALS_1.setGain(static_cast<SunS_BH1730FVC_Types::Gain>(TWIRegisterInterface::ALS_GAIN));
                ALS_1_status |= ALS_1.setIntegrationTime(TWIRegisterInterface::ALS_INTEGRATION_TIME);       

                ids = ALS_2.readPartID();
                TWIRegisterInterface::registers.registerMap.ALS_2A_ID = ids[0];
                TWIRegisterInterface::registers.registerMap.ALS_2B_ID = ids[1];
                TWIRegisterInterface::registers.registerMap.ALS_2C_ID = ids[2];
                TWIRegisterInterface::registers.registerMap.ALS_2D_ID = ids[3];

                ALS_2_status |= ALS_2.setGain(static_cast<SunS_BH1730FVC_Types::Gain>(TWIRegisterInterface::ALS_GAIN));
                ALS_2_status |= ALS_2.setIntegrationTime(TWIRegisterInterface::ALS_INTEGRATION_TIME);

                ids = ALS_3.readPartID();
                TWIRegisterInterface::registers.registerMap.ALS_3A_ID = ids[0];
                TWIRegisterInterface::registers.registerMap.ALS_3B_ID = ids[1];
                TWIRegisterInterface::registers.registerMap.ALS_3C_ID = ids[2];
                TWIRegisterInterface::registers.registerMap.ALS_3D_ID = ids[3];

                ALS_3_status |= ALS_3.setGain(static_cast<SunS_BH1730FVC_Types::Gain>(TWIRegisterInterface::ALS_GAIN));
                ALS_3_status |= ALS_3.setIntegrationTime(TWIRegisterInterface::ALS_INTEGRATION_TIME);

                ALS_1_status |= ALS_1.setMeasurement(SunS_BH1730FVC_Types::ONE_SHOT, SunS_BH1730FVC_Types::VL_IR);
                ALS_2_status |= ALS_2.setMeasurement(SunS_BH1730FVC_Types::ONE_SHOT, SunS_BH1730FVC_Types::VL_IR);
                ALS_3_status |= ALS_3.setMeasurement(SunS_BH1730FVC_Types::ONE_SHOT, SunS_BH1730FVC_Types::VL_IR);

                // wait for conversion performed by the ALSes
                waitForALS();


                ALS_1_status |= ALS_1.ambientLightRAW(dataVL, dataIR);
                TWIRegisterInterface::registers.registerMap.ALS_1A_VL_RAW = dataVL[0];
                TWIRegisterInterface::registers.registerMap.ALS_1B_VL_RAW = dataVL[1];
                TWIRegisterInterface::registers.registerMap.ALS_1C_VL_RAW = dataVL[2];
                TWIRegisterInterface::registers.registerMap.ALS_1D_VL_RAW = dataVL[3];

                TWIRegisterInterface::registers.registerMap.ALS_1A_IR_RAW = dataIR[0];
                TWIRegisterInterface::registers.registerMap.ALS_1B_IR_RAW = dataIR[1];
                TWIRegisterInterface::registers.registerMap.ALS_1C_IR_RAW = dataIR[2];
                TWIRegisterInterface::registers.registerMap.ALS_1D_IR_RAW = dataIR[3];

                ALS_2_status |= ALS_2.ambientLightRAW(dataVL, dataIR);
                TWIRegisterInterface::registers.registerMap.ALS_2A_VL_RAW = dataVL[0];
                TWIRegisterInterface::registers.registerMap.ALS_2B_VL_RAW = dataVL[1];
                TWIRegisterInterface::registers.registerMap.ALS_2C_VL_RAW = dataVL[2];
                TWIRegisterInterface::registers.registerMap.ALS_2D_VL_RAW = dataVL[3];

                TWIRegisterInterface::registers.registerMap.ALS_2A_IR_RAW = dataIR[0];
                TWIRegisterInterface::registers.registerMap.ALS_2B_IR_RAW = dataIR[1];
                TWIRegisterInterface::registers.registerMap.ALS_2C_IR_RAW = dataIR[2];
                TWIRegisterInterface::registers.registerMap.ALS_2D_IR_RAW = dataIR[3];

                ALS_3_status |= ALS_3.ambientLightRAW(dataVL, dataIR);
                TWIRegisterInterface::registers.registerMap.ALS_3A_VL_RAW = dataVL[0];
                TWIRegisterInterface::registers.registerMap.ALS_3B_VL_RAW = dataVL[1];
                TWIRegisterInterface::registers.registerMap.ALS_3C_VL_RAW = dataVL[2];
                TWIRegisterInterface::registers.registerMap.ALS_3D_VL_RAW = dataVL[3];

                TWIRegisterInterface::registers.registerMap.ALS_3A_IR_RAW = dataIR[0];
                TWIRegisterInterface::registers.registerMap.ALS_3B_IR_RAW = dataIR[1];
                TWIRegisterInterface::registers.registerMap.ALS_3C_IR_RAW = dataIR[2];
                TWIRegisterInterface::registers.registerMap.ALS_3D_IR_RAW = dataIR[3];


                // ALS collective status
                ALS_collective_status = ALS_1_status | (ALS_2_status << 5) | (ALS_3_status << 10);
                TWIRegisterInterface::registers.registerMap.ALS_STATUS = ALS_collective_status;

                uint16_t lm60_raw = LM60.measure();
                TWIRegisterInterface::registers.registerMap.TEMPERATURE_STRUCT = LM60.temperature(lm60_raw);
                TWIRegisterInterface::registers.registerMap.TEMPERATURE_STRUCT_RAW = lm60_raw;
                // new data
                TWIRegisterInterface::registers.registerMap.STATUS |= TWIRegisterInterface::StatusReg::NEW_DATA;
            } else {
                TWIRegisterInterface::registers.registerMap.STATUS |= TWIRegisterInterface::StatusReg::ERROR;
            }
        }
    }
}
