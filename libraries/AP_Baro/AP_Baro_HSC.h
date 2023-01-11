#pragma once

#include "AP_Baro_Backend.h"

#if AP_BARO_HSC_ENABLED

#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/Semaphores.h>
#include <AP_HAL/Device.h>

#ifndef HAL_BARO_HSC_I2C_ADDR
#define HAL_BARO_HSC_I2C_ADDR 0x28
#endif

#ifndef HAL_BARO_HSC_I2C_ADDR2
#define HAL_BARO_HSC_I2C_ADDR2 0x38
#endif

class AP_Baro_HSC : public AP_Baro_Backend
{
public:
    void update() override;

    enum HSC_TRANSFER_FUNCTION_TYPE {
        ANALOG_A,
        ANALOG_B,
        ANALOG_C,
        ANALOG_F,
    };

    enum HSC_PRESSURE_RANGE_TYPE {
        ABS_100KA,
    };

    static AP_Baro_Backend *probe(AP_Baro& baro, AP_HAL::OwnPtr<AP_HAL::Device> dev, enum HSC_TRANSFER_FUNCTION_TYPE hsc_tf_type=ANALOG_A, enum HSC_PRESSURE_RANGE_TYPE hsc_pr_type=ABS_100KA);

private:
    AP_Baro_HSC(AP_Baro& baro, AP_HAL::OwnPtr<AP_HAL::Device> dev, enum HSC_TRANSFER_FUNCTION_TYPE hsc_tf_type, enum HSC_PRESSURE_RANGE_TYPE hsc_pr_type);

    bool _init();

    void _read();

    void _calculate();

    AP_HAL::OwnPtr<AP_HAL::Device> _dev;
    uint8_t _instance;

    uint8_t _status;
    uint16_t _bridge_data;
    uint16_t _temperature_data;

    uint16_t _output_min;
    uint16_t _output_max;
    float _pressure_min_pa;
    float _pressure_max_pa;

    float _pressure_pa;
    float _temperature_c;

    enum HSC_TRANSFER_FUNCTION_TYPE _hsc_tf_type;
    enum HSC_PRESSURE_RANGE_TYPE _hsc_pr_type;
};


#endif // AP_BARO_HSC_ENABLED