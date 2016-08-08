#ifndef HAL_PERIPH_SPI_H_
#define HAL_PERIPH_SPI_H_

#include <avr/io.h>
#include "mcu.h"
#include "DigitalIO.h"
#include "array.h"

namespace hal {

class SPI {
 public:
    enum class Polarity : int {
        idle_low = 0,
        idle_high = 1
    };

    enum class Phase : int {
        leading_sample = 0,
        trailing_sample = 1
    };

    enum class DataOrder : int {
        MSB_first = 0,
        LSB_first = 1
    };

    enum class ClockDivisor : int {
        DIV_4 = 0,
        DIV_16 = 1,
        DIV_64 = 2,
        DIV_128 = 3,
    };

    static void init(const Polarity polarity, const Phase phase,
            const DataOrder data_order, const ClockDivisor clock_divisor) {
        pin_mosi.init(DigitalIO::OUTPUT);
        pin_sck.init(DigitalIO::OUTPUT);

        SPCR = (1 << SPE) | (1 << MSTR) | (static_cast<uint8_t>(clock_divisor))
                | (static_cast<uint8_t>(phase) << CPHA)
                | (static_cast<uint8_t>(polarity) << CPOL)
                | (static_cast<uint8_t>(data_order) << DORD);
    }

    static uint8_t shift(const uint8_t data) {
        SPDR = data;
        while (!(SPSR & (1 << SPIF))) {
        }
        return SPDR;
    }

 private:
    static constexpr DigitalIO pin_mosi { mcu::pin_mosi },
                               pin_sck  { mcu::pin_sck };
};

class SPI_Device {
 public:
    constexpr explicit SPI_Device(const DigitalIO::Pin pin_cs) :
            pin_cs(pin_cs) {
    }

    void init() const {
        this->pin_cs.init(DigitalIO::OUTPUT);
        this->disable();
    }

    void enable() const {
        this->pin_cs.reset();
    }

    void disable() const {
        this->pin_cs.set();
    }

    uint8_t shift(const uint8_t data) const {
        return SPI::shift(data);
    }

    uint8_t data_transfer(const uint8_t data) const {
        this->enable();
        uint8_t x = SPI::shift(data);
        this->disable();
        return x;
    }

    template<typename T, typename T2>
    void data_transfer(T&& output, T2&& input) const {
        this->enable();
        const uint8_t * out_ptr = output.data();
        uint8_t * in_ptr = input.data();
        int len = input.size();
        while (len--) {
            (*in_ptr) = SPI::shift(*out_ptr);
            in_ptr++;
            out_ptr++;
        }
        this->disable();
    }

    template<typename T>
    void data_transmit(T&& data) const {
        this->enable();
        for (auto& x : data) {
            SPI::shift(x);
        }
        this->disable();
    }

    template<typename T>
    void data_receive(T&& data) const {
        this->enable();
        for (auto& x : data) {
            x = SPI::shift(0);
        }
        this->disable();
    }

 private:
     const DigitalIO pin_cs;
};

}  // namespace hal

#endif  // HAL_PERIPH_SPI_H_
