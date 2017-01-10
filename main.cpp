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


using ALS_1_Type = SunS_BH1730FVC<pins::SCL_1, pins::SDA_A1, pins::SDA_B1, pins::SDA_C1, pins::SDA_D1>;
using ALS_2_Type = SunS_BH1730FVC<pins::SCL_2, pins::SDA_A2, pins::SDA_B2, pins::SDA_C2, pins::SDA_D2>;
using ALS_3_Type = SunS_BH1730FVC<pins::SCL_3, pins::SDA_A3, pins::SDA_B3, pins::SDA_C3, pins::SDA_D3>;


int main() {
    Serial0.init(9600);
    InternalADC::init(InternalADC::Prescaler::DIV_128, ADC_REFERENCE_TYPE);
    
    SunS_LM60 lm(TEMP_BOARD, 16, ADC_REFERENCE_VALUE);

    SunS_RTD RTD_A(RTD_AIN_A, 16, RTD_REFERENCE_RESISTANCE, ADC_REFERENCE_VALUE);
    SunS_RTD RTD_B(RTD_AIN_B, 16, RTD_REFERENCE_RESISTANCE, ADC_REFERENCE_VALUE);
    SunS_RTD RTD_C(RTD_AIN_C, 16, RTD_REFERENCE_RESISTANCE, ADC_REFERENCE_VALUE);
    SunS_RTD RTD_D(RTD_AIN_D, 16, RTD_REFERENCE_RESISTANCE, ADC_REFERENCE_VALUE);

    SunS_BH1730FVC <pins::SCL_1, pins::SDA_A1, pins::SDA_B1, pins::SDA_C1, pins::SDA_D1> ALS_1;
    SunS_BH1730FVC <pins::SCL_2, pins::SDA_A2, pins::SDA_B2, pins::SDA_C2, pins::SDA_D2> ALS_2;
    SunS_BH1730FVC <pins::SCL_3, pins::SDA_A3, pins::SDA_B3, pins::SDA_C3, pins::SDA_D3> ALS_3;


    uint16_t raw_a, raw_b, raw_c, raw_d;
    libs::array<uint16_t, 4> dataVL_1, dataIR_1, dataVL_2, dataIR_2, dataVL_3, dataIR_3;
    libs::array<uint8_t, 4> ids;

    while (true) {
        raw_a = RTD_A.measure();
        raw_b = RTD_B.measure();
        raw_c = RTD_C.measure();
        raw_d = RTD_D.measure();

        ids = ALS_1.readPartID();
        ALS_1.setGain(ALS_1_Type::GAIN_1);
        ALS_1.setIntegrationTime(38);

        ids = ALS_2.readPartID();
        ALS_2.setGain(ALS_2_Type::GAIN_1);
        ALS_2.setIntegrationTime(38);

        ids = ALS_3.readPartID();
        ALS_3.setGain(ALS_3_Type::GAIN_1);
        ALS_3.setIntegrationTime(38);

        ALS_1.setMeasurement(ALS_1_Type::ONE_SHOT, ALS_1_Type::VL_IR);
        ALS_2.setMeasurement(ALS_2_Type::ONE_SHOT, ALS_2_Type::VL_IR);
        ALS_3.setMeasurement(ALS_3_Type::ONE_SHOT, ALS_3_Type::VL_IR);

        _delay_ms(900);

        ALS_1.ambientLightRAW(dataVL_1, dataIR_1);
        ALS_2.ambientLightRAW(dataVL_2, dataIR_2);
        ALS_3.ambientLightRAW(dataVL_3, dataIR_3);
        float lm60_temp = lm.temperature(lm.measure());

        
        Serial0.printf("%f %u %f %f %u %f %f %u %f %f %u %f %f ", lm60_temp, raw_a, RTD_A.resistance(raw_a), RTD_A.temperature(raw_a), raw_b, RTD_B.resistance(raw_b), RTD_B.temperature(raw_b), raw_c, RTD_C.resistance(raw_c), RTD_C.temperature(raw_c), raw_d, RTD_C.resistance(raw_d), RTD_C.temperature(raw_d));

        Serial0.printf("%u %u %u %u %u %u %u %u %u %u %u %u %u %u %u %u %u %u %u %u %u %u %u %u\r\n", dataVL_1[0], dataIR_1[0], dataVL_1[1], dataIR_1[1], dataVL_1[2], dataIR_1[2], dataVL_1[3], dataIR_1[3], dataVL_2[0], dataIR_2[0], dataVL_2[1], dataIR_2[1], dataVL_2[2], dataIR_2[2], dataVL_2[3], dataIR_2[3], dataVL_3[0], dataIR_3[0], dataVL_3[1], dataIR_3[1], dataVL_3[2], dataIR_3[2], dataVL_3[3], dataIR_3[3]);

        _delay_ms(1);
    }
}
