#pragma once

#include "AP_Baro_Backend.h"
#include "AP_Baro_HSC_Types.h"

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

    static AP_Baro_Backend *probe(AP_Baro& baro, AP_HAL::OwnPtr<AP_HAL::Device> dev, enum AP_Baro_HSC_Types::TRANSFER_FUNCTION hsc_tf_type=AP_Baro_HSC_Types::TRANSFER_FUNCTION::ANALOG_A, enum AP_Baro_HSC_Types::PRESSURE_RANGE hsc_pr_type=AP_Baro_HSC_Types::PRESSURE_RANGE::ABS_100KA);

private:
    AP_Baro_HSC(AP_Baro& baro, AP_HAL::OwnPtr<AP_HAL::Device> dev, enum AP_Baro_HSC_Types::TRANSFER_FUNCTION hsc_tf_type=AP_Baro_HSC_Types::TRANSFER_FUNCTION::ANALOG_A, enum AP_Baro_HSC_Types::PRESSURE_RANGE hsc_pr_type=AP_Baro_HSC_Types::PRESSURE_RANGE::ABS_100KA);

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

    enum AP_Baro_HSC_Types::TRANSFER_FUNCTION _hsc_tf_type;
    enum AP_Baro_HSC_Types::PRESSURE_RANGE _hsc_pr_type;
};


#endif // AP_BARO_HSC_ENABLED