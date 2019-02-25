#ifndef SUNS_APP_INC_SUNS_RTD_H_
#define SUNS_APP_INC_SUNS_RTD_H_

#include "Analog.h"
#include "RTD.h"

namespace hal {

class SunS_RTD {
 public:
    constexpr SunS_RTD(const InternalADC::Input input, const uint8_t resolution, const float ref_resistance, const float reference_voltage, const RTD rtd = RTD(1000)) :
            analog_channel{input}, oversample((resolution < 10) ? 1 : ((resolution > 16) ? 4096 : (1 << 2*(resolution - 10)))), ref_resistance(ref_resistance), rtd{rtd}, reference_voltage(reference_voltage) {
    }

    uint16_t measure() {
        mcu::InternalADCMux::select(analog_channel);
        
        uint16_t result = 0;
        for (uint8_t i = 0; i < oversample; i++) {
            result += InternalADC::read();
        }
        return result;
    }

    float resistance(uint16_t raw) {
        float mv = bits_to_mV(raw);
        return ref_resistance/(reference_voltage*1000/mv - 1); 
    }

    int16_t temperature(uint16_t raw) {
        float res = resistance(raw);
        int16_t temperature = static_cast<int16_t>(10*rtd.temperature(res));
        return temperature;
    }

 private:
    const InternalADC::Input analog_channel;
    const uint8_t oversample;
    const float ref_resistance, reference_voltage;
    RTD rtd;

    float bits_to_mV(uint16_t raw) {
        return static_cast<float>(raw)/(oversample)/1024.0*reference_voltage*1000;
    }
  
};

}  // namespace hal

#endif  // SUNS_APP_INC_SUNS_RTD_H_
