#ifndef SUNS_APP_INC_SUNS_LM60_H_
#define SUNS_APP_INC_SUNS_LM60_H_

#include "Analog.h"
#include "LM60.h"

namespace hal {

class SunS_LM60 {
 public:
    constexpr SunS_LM60(const InternalADC::Input input, const uint8_t resolution) :
            analog_channel{input}, oversample((resolution < 10) ? 1 : ((resolution > 16) ? 64 : (1 << (resolution - 10)))) {
    }

    uint16_t measure() {
        mcu::InternalADCMux::select(analog_channel);
        
        uint16_t result = 0;
        for (uint8_t i = 0; i < oversample; i++) {
            result += InternalADC::read();
        }
        return result;
    }

    float temperature(uint16_t raw) {
        float mv = bits_to_mV(raw);
        float temperature = LM60::temperature(mv);
        return temperature;
    }

 private:
    const InternalADC::Input analog_channel;
    const uint8_t oversample;

    float bits_to_mV(uint16_t raw) {
        return static_cast<float>(raw)/(oversample)/1024.0*InternalADC::read_reference()*1000;
    }
   
};

}  // namespace hal

#endif  // SUNS_APP_INC_SUNS_LM60_H_
