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
#include "TWISlave.h"


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
        uint8_t CTRL_STATUS;
        uint8_t ALS_INTEGRATION_TIME;
        uint8_t ALS_GAIN;
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
    volatile uint8_t registerMapArray[90];
};
#pragma pack(pop) 

//uint8_t register_counter = 0;

volatile registerDesc registers = {
    {
        0x00,
        0x26,
        0x01,
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

volatile bool statusReset = false; 
constexpr uint8_t registers_len = 90;

void hal::TWISlave::callbackRx() {
    volatile uint8_t register_counter = TWISlave::rx_buffer[0];
    volatile uint8_t end_address = TWISlave::rx_buffer[0] + TWISlave::rx_buffer_cnt - 2;

    if ((1 < TWISlave::rx_buffer_cnt) && (end_address < 3)) {

        
        if (0x00 == TWISlave::rx_buffer[0]) {
            registers.registerMap.CTRL_STATUS |= (1 << 1);
        } else {
            volatile uint8_t xx = TWISlave::rx_buffer[1];
            registers.registerMapArray[register_counter] = xx;
        }
        
        for (uint8_t i = 1; i < (rx_buffer_cnt - 1); i++) {
            registers.registerMapArray[register_counter+i] = TWISlave::rx_buffer[i+1];
        }

    } else if ((1 == TWISlave::rx_buffer_cnt) && (90 > TWISlave::rx_buffer[0])) {
        for (uint8_t i = 0; i < (registers_len - register_counter); i++) {
            hal::TWISlave::tx_buffer[i] = registers.registerMapArray[register_counter+i];
        }

        // if register 0x00 is scheduled to be read
        if (0x00 == register_counter) {
            statusReset = true;
        } else {
            statusReset = false;
        }

    } else {
        // error in register (at address 0x00, bitfield, bit no. 0)
        registers.registerMap.CTRL_STATUS |= (1 << 0);
    }
}

void hal::TWISlave::callbackTx() {
    if (true == statusReset) {
        registers.registerMap.CTRL_STATUS &= ~(101 << 0);
        statusReset = false;
    }
}

using ALS_1_Type = SunS_BH1730FVC<pins::SCL_1, pins::SDA_A1, pins::SDA_B1, pins::SDA_C1, pins::SDA_D1>;
using ALS_2_Type = SunS_BH1730FVC<pins::SCL_2, pins::SDA_A2, pins::SDA_B2, pins::SDA_C2, pins::SDA_D2>;
using ALS_3_Type = SunS_BH1730FVC<pins::SCL_3, pins::SDA_A3, pins::SDA_B3, pins::SDA_C3, pins::SDA_D3>;


int main() {
    Serial0.init(9600);
    InternalADC::init(InternalADC::Prescaler::DIV_128, ADC_REFERENCE_TYPE);
    hal::TWISlave::init(0x1E);
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
        if (registers.registerMap.CTRL_STATUS & 0b00000010) {

            registers.registerMap.CTRL_STATUS &= ~(1 << 1);

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

            ALS_1.setGain(ALS_1_Type::GAIN_1);
            ALS_1.setIntegrationTime(registers.registerMap.ALS_INTEGRATION_TIME);       

            ids = ALS_2.readPartID();
            registers.registerMap.ALS_2A_ID = ids[0];
            registers.registerMap.ALS_2B_ID = ids[1];
            registers.registerMap.ALS_2C_ID = ids[2];
            registers.registerMap.ALS_2D_ID = ids[3];

            ALS_2.setGain(ALS_2_Type::GAIN_1);
            ALS_2.setIntegrationTime(registers.registerMap.ALS_INTEGRATION_TIME);

            ids = ALS_3.readPartID();
            registers.registerMap.ALS_3A_ID = ids[0];
            registers.registerMap.ALS_3B_ID = ids[1];
            registers.registerMap.ALS_3C_ID = ids[2];
            registers.registerMap.ALS_3D_ID = ids[3];

            ALS_3.setGain(ALS_3_Type::GAIN_1);
            ALS_3.setIntegrationTime(registers.registerMap.ALS_INTEGRATION_TIME);

            ALS_1.setMeasurement(ALS_1_Type::ONE_SHOT, ALS_1_Type::VL_IR);
            ALS_2.setMeasurement(ALS_2_Type::ONE_SHOT, ALS_2_Type::VL_IR);
            ALS_3.setMeasurement(ALS_3_Type::ONE_SHOT, ALS_3_Type::VL_IR);

            _delay_ms(900);

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
            registers.registerMap.CTRL_STATUS |= (1 << 2);
        }
    }
}
