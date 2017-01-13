#ifndef HAL_DEVICES_SUNS_BH1730FVC_H_
#define HAL_DEVICES_SUNS_BH1730FVC_H_

#include "array.h"
#include "DigitalIO.h"
#include <util/delay.h>


namespace hal {

namespace softI2Cmulti_Types {
enum Action {
    START_WRITE = 0,
    START_READ = 1
};

} // namespace softI2Cmulti_Types

template<DigitalIO::Pin scl, DigitalIO::Pin SDA_A, DigitalIO::Pin SDA_B, DigitalIO::Pin SDA_C, DigitalIO::Pin SDA_D>
class softI2Cmulti {
 public:
    softI2Cmulti() {
    }

    void init() const {
        this->SCL.pinmode(DigitalIO::INPUT);
        this->SCL.reset();

        for (uint8_t i = 0; i < this->len; i++) {
            SDAs[i].pinmode(DigitalIO::INPUT);
            SDAs[i].read();
        }
    }


    uint8_t start(uint8_t address, const softI2Cmulti_Types::Action start_action) {
        this->SCL.pinmode(DigitalIO::INPUT);
        _delay_loop_1(this->hDelay);
        this->SDAs_LOW();
        _delay_loop_1(this->hDelay);

        return this->write((address << 1) | start_action);
    }

    void stop() const {
        this->SDAs_LOW();
        _delay_loop_1(this->hDelay);
        this->SCL.pinmode(DigitalIO::INPUT);
        _delay_loop_1(this->qDelay);
        this->SDAs_HIGH();
        _delay_loop_1(this->hDelay);
    }

    uint8_t write(uint8_t data) {
        uint8_t return_status = 0;

        for (uint8_t i = 0; i < 8; i++) {
            this->SCL.pinmode(DigitalIO::OUTPUT);
            _delay_loop_1(this->qDelay);

            if (data & 0x80) {
                this->SDAs_HIGH();
            } else {
                this->SDAs_LOW();
            }

            _delay_loop_1(this->hDelay);
            this->SCL.pinmode(DigitalIO::INPUT);
            _delay_loop_1(this->hDelay);

            volatile uint16_t timeout = 0;
            while (0 == this->SCL.read()) {
                timeout++;
                if (timeout > 5000) {
                    return_status |= (1 << 4);
                    break;
                }
            }

            data = data << 1;
        }

        this->SCL.pinmode(DigitalIO::OUTPUT);
        _delay_loop_1(this->qDelay);

        this->SDAs_HIGH();
        _delay_loop_1(this->hDelay);

        this->SCL.pinmode(DigitalIO::INPUT);
        _delay_loop_1(this->hDelay);


        for (uint8_t i = 0; i < 4; i++) {
            return_status |= (SDAs[i].read() << i);
        }

        this->SCL.pinmode(DigitalIO::OUTPUT);
        _delay_loop_1(this->hDelay);

        return return_status;
    }

    libs::array<uint8_t, 4> read(bool ACK) {
        libs::array<uint8_t, 4> SDA_read_data;

        for (uint8_t i = 0; i < 8; i++) {
            this->SCL.pinmode(DigitalIO::OUTPUT);
            _delay_loop_1(this->hDelay);
            this->SCL.pinmode(DigitalIO::INPUT);
            _delay_loop_1(this->hDelay);

            volatile uint16_t timeout = 0;
            while (0 == this->SCL.read()) {
                timeout++;
                if (timeout > 5000) {
                    break;
                }
            }


            for (uint8_t j = 0; j < 4; j++) {
                if (SDAs[j].read()) {
                    SDA_read_data[j] |=(0x80 >> i);
                } else {
                    SDA_read_data[j] &=~(0x80 >> i);
                }
            }
        }

        this->SCL.pinmode(DigitalIO::OUTPUT);
        _delay_loop_1(this->qDelay);

        if (ACK) {
            this->SDAs_LOW();
        } else {
            this->SDAs_HIGH();
        }

        _delay_loop_1(this->hDelay);

        this->SCL.pinmode(DigitalIO::INPUT);
        _delay_loop_1(this->hDelay);

        this->SCL.pinmode(DigitalIO::OUTPUT);

        this->SDAs_HIGH();
        _delay_loop_1(this->hDelay);

        return SDA_read_data;
    }

 private:
    const DigitalIO SCL{scl};
    libs::array<const DigitalIO, 4> SDAs = {{DigitalIO(SDA_A), DigitalIO(SDA_B), DigitalIO(SDA_C), DigitalIO(SDA_D)}};

    uint8_t qDelay{3};
    uint8_t hDelay{5};
    uint8_t len{4};

    void SDAs_LOW() const {
        for (uint8_t i = 0; i < this->len; i++) {
            SDAs[i].pinmode(DigitalIO::OUTPUT);
        }
    }
    void SDAs_HIGH() const {
        for (uint8_t i = 0; i < this->len; i++) {
            SDAs[i].pinmode(DigitalIO::INPUT);
        }
    }
};

namespace SunS_BH1730FVC_Types {
enum Gain {
    GAIN_1 = 0b00,
    GAIN_2 = 0b01,
    GAIN_64 = 0b10,
    GAIN_128 = 0b11,
};

enum MeasurementType {
    CONTINUOUS = 0b00000011,
    ONE_SHOT = 0b00001011,
};

enum DataSel {
    VL_IR = 0b00000000,
    VL_ONLY = 0b00000100,
};
} // namespace SunS_BH1730FVC_Types

template<DigitalIO::Pin scl, DigitalIO::Pin SDA_A, DigitalIO::Pin SDA_B, DigitalIO::Pin SDA_C, DigitalIO::Pin SDA_D>
class SunS_BH1730FVC {
 public:
     SunS_BH1730FVC() {}

     void init() const {
         this->i2c.init();
     }

    uint8_t setIntegrationTime(uint8_t cycle) {
        uint8_t comm_status = 0;

        uint8_t return_status = this->i2c.start(this->address, softI2Cmulti_Types::START_WRITE);

        comm_status = this->processStatus(comm_status, return_status);

        return_status = this->i2c.write(0b10000000 | TIMING);

        comm_status = this->processStatus(comm_status, return_status);

        // ITIME
        return_status = this->i2c.write(static_cast<uint8_t>(256-cycle));

        comm_status =  this->processStatus(comm_status, return_status);

        this->i2c.stop();
        return comm_status;
    }


    uint8_t setGain(SunS_BH1730FVC_Types::Gain gain) {
        uint8_t comm_status = 0;

        uint8_t return_status = this->i2c.start(this->address, softI2Cmulti_Types::START_WRITE);

        comm_status =  this->processStatus(comm_status, return_status);

        return_status = this->i2c.write(0b10000000 | GAIN);

        comm_status = this->processStatus(comm_status, return_status);

        // GAIN
        return_status = this->i2c.write(static_cast<uint8_t>(gain));

        comm_status = this->processStatus(comm_status, return_status);

        this->i2c.stop();

        return comm_status;
    }

    libs::array<uint8_t, 4> readPartID() {
        this->i2c.start(this->address, softI2Cmulti_Types::START_WRITE);


        this->i2c.write(0b10000000 | ID);


        this->i2c.stop();

        this->i2c.start(this->address, softI2Cmulti_Types::START_READ);

        libs::array<uint8_t, 4> ids =  this->i2c.read(false);

        this->i2c.stop();

        return ids;
    }

    uint8_t ambientLightRAW(libs::array<uint16_t, 4>& VL, libs::array<uint16_t, 4>& IR) {
        libs::array<uint8_t, 4> DATA0MSB;
        libs::array<uint8_t, 4> DATA0LSB;
        libs::array<uint8_t, 4> DATA1MSB;
        libs::array<uint8_t, 4> DATA1LSB;

        uint8_t comm_status = 0;

        uint8_t return_status = this->i2c.start(this->address, softI2Cmulti_Types::START_WRITE);

        comm_status = this->processStatus(comm_status, return_status);

        return_status = this->i2c.write(0b10010100);

        comm_status = this->processStatus(comm_status, return_status);

        this->i2c.stop();

        return_status = this->i2c.start(this->address, softI2Cmulti_Types::START_READ);

        comm_status = this->processStatus(comm_status, return_status);

        // LSB first, then MSB!
        DATA0LSB = this->i2c.read(true);
        DATA0MSB = this->i2c.read(true);
        DATA1LSB = this->i2c.read(true);
        DATA1MSB = this->i2c.read(false);

        this->i2c.stop();

        for (uint8_t i = 0; i < 4; i++) {
            VL[i] = (static_cast<uint16_t>(DATA0MSB[i]) << 8) | DATA0LSB[i];
            IR[i] = (static_cast<uint16_t>(DATA1MSB[i]) << 8) | DATA1LSB[i];
        }

        return comm_status;
    }


    uint8_t ambientLightRAW(libs::array<uint16_t, 4>& VL) {
        libs::array<uint8_t, 4> DATA0MSB;
        libs::array<uint8_t, 4> DATA0LSB;

        uint8_t comm_status = 0;

        uint8_t return_status = this->i2c.start(this->address, softI2Cmulti_Types::START_WRITE);
        comm_status = this->processStatus(comm_status, return_status);

        return_status = this->i2c.write(0b10010100);
        comm_status = this->processStatus(comm_status, return_status);

        this->i2c.stop();

        return_status = this->i2c.start(this->address, softI2Cmulti_Types::START_READ);
        comm_status = this->processStatus(comm_status, return_status);

        // LSB first, then MSB!
        DATA0LSB = this->i2c.read(true);
        DATA0MSB = this->i2c.read(false);

        this->i2c.stop();

        for (uint8_t i = 0; i < 4; i++) {
            VL[i] = (static_cast<uint16_t>(DATA0MSB[i]) << 8) | DATA0LSB[i];
        }

        return comm_status;
    }

    uint8_t setMeasurement(SunS_BH1730FVC_Types::MeasurementType type, SunS_BH1730FVC_Types::DataSel data_sel) {
        uint8_t comm_status = 0;

        uint8_t return_status = this->i2c.start(this->address, softI2Cmulti_Types::START_WRITE);
        comm_status = this->processStatus(comm_status, return_status);

        return_status = this->i2c.write(0b10000000 | CONTROL);
        comm_status = this->processStatus(comm_status, return_status);

        // CONTROL
        return_status = this->i2c.write(type | data_sel);
        comm_status = this->processStatus(comm_status, return_status);

        this->i2c.stop();
        return comm_status;
    }


    bool adcValid() {
        this->i2c.start(this->address, softI2Cmulti_Types::START_WRITE);

        this->i2c.write(0b10000000 | CONTROL);

        this->i2c.stop();

        this->i2c.start(this->address, softI2Cmulti_Types::START_READ);

        // CONTROL
        uint8_t reg_val = this->i2c.read(false)[0];

        this->i2c.stop();
        return static_cast<bool>(reg_val & (1 << 4));
    }

 private:
    softI2Cmulti<scl, SDA_A, SDA_B, SDA_C, SDA_D> i2c;
    const uint8_t address = 0b0101001;

    enum RegisterAddr {
        CONTROL = 0x00,
        TIMING = 0x01,
        INTERRUPT = 0x02,
        THLLOW = 0x03,
        THLHIGH = 0x04,
        THHLOW = 0x05,
        THHHIGH = 0x06,
        GAIN = 0x07,
        ID = 0x12,
        DATA0LOW = 0x14,
        DATA0HIGH = 0x15,
        DATA1LOW = 0x16,
        DATA1HIGH = 0x17,
    };

    uint8_t processStatus(uint8_t collective_status,  uint8_t returned_status) {
        collective_status |= returned_status;
        return collective_status;
    }
};

}  // namespace hal

#endif  // HAL_DEVICES_SUNS_BH1730FVC_H_
