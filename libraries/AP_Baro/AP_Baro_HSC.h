#pragma once

#include "AP_Baro_Backend.h"

#if AP_BARO_HSC_ENABLED

#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/Semaphores.h>
#include <AP_HAL/Device.h>

class AP_Baro_HSC : public AP_Baro_Backend
{
public:
    void update() override;

    static AP_Baro_Backend* probe(AP_Baro& baro, AP_HAL::OwnPtr<AP_HAL::Device> dev,
                                  uint16_t output_min, uint16_t output_max,
                                  float pressure_min, float pressure_max);

private:
    AP_Baro_HSC(AP_Baro& baro, AP_HAL::OwnPtr<AP_HAL::Device> dev,
                uint16_t output_min, uint16_t output_max,
                float pressure_min, float pressure_max);

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
    float _pressure_min;
    float _pressure_max;

    float _pressure_pa;
    float _temperature_c;
};


#endif // AP_BARO_HSC_ENABLED