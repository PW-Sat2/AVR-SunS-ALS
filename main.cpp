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

int main() {

    InternalADC::init(InternalADC::Prescaler::DIV_128, InternalADC::Reference::AVcc, 5);
    SunS_LM60 lm(InternalADC::Input::ADC15, 16);

    RTD myRTD(500);
    SunS_RTD rtd(InternalADC::Input::ADC15, 16, 2200, myRTD);

    Serial0.init(115200);

    libs::array<uint16_t, 4> dataVL{0}, dataVL2{0};
    BH1730FVC ALS(pins::A0, pins::A1, pins::A2, pins::A3, pins::A4);
    BH1730FVC ALS2(pins::A5, pins::A1, pins::A2, pins::A3, pins::A4);
    

    ALS.init();
    ALS2.init();

    ALS.setMeasurement(BH1730FVC::CONTINUOUS, BH1730FVC::VL_ONLY);
    ALS.setGain(BH1730FVC::GAIN_1);
    ALS.setIntegrationTime(38);

    ALS2.setMeasurement(BH1730FVC::CONTINUOUS, BH1730FVC::VL_ONLY);
    ALS2.setGain(BH1730FVC::GAIN_1);
    ALS2.setIntegrationTime(38);
    
    
    _delay_ms(1000);
    
    while (true) {
        libs::array<softI2Cmulti::Status, 5> status = ALS.ambientLightRAW(dataVL);
        libs::array<softI2Cmulti::Status, 5> status2 = ALS2.ambientLightRAW(dataVL2);


        libs::array<uint8_t, 4> ids = ALS.readPartID();
        libs::array<uint8_t, 4> ids2 = ALS2.readPartID();


        for (uint8_t i = 0; i < 4; i++) {
            Serial0.printf("%u \t %u \t Status: %d \t VL: %u\r\n", i, ids[i], status[i], dataVL[i]);
        }
        Serial0.printf("%u\r\n", status[4]);

        for (uint8_t i = 0; i < 4; i++) {
            Serial0.printf("%u \t %u \t Status: %d \t VL: %u\r\n", i, ids2[i], status2[i], dataVL2[i]);
        }
        Serial0.printf("%u\r\n", status2[4]);

        Serial0.printf("---------------------\r\n");
        Serial0.printf("LM60: %f\r\n", lm.temperature(lm.measure()));
        Serial0.printf("---------------------\r\n");

        _delay_ms(500);
    }
}
