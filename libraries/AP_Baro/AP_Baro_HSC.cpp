/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.
   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.
   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#include "AP_Baro_HSC.h"

#if AP_BARO_HSC_ENABLED

#include <utility>
#include <stdio.h>

#include <AP_Math/AP_Math.h>
#include <AP_Math/crc.h>
#include <AP_BoardConfig/AP_BoardConfig.h>

extern const AP_HAL::HAL &hal;

// constructor
AP_Baro_HSC::AP_Baro_HSC(AP_Baro& baro, AP_HAL::OwnPtr<AP_HAL::Device> dev,
                         uint16_t output_min, uint16_t output_max,
                         float pressure_min, float pressure_max)

    : AP_Baro_Backend(baro)
    , _dev(std::move(dev))
    , _output_min(output_min)
    , _output_max(output_max)
    , _pressure_min(pressure_min)
    , _pressure_max(pressure_max)
{
}

// probe if device is connected and return an instance of it
AP_Baro_Backend* AP_Baro_HSC::probe(AP_Baro& baro, AP_HAL::OwnPtr<AP_HAL::Device> dev,
                                    uint16_t output_min, uint16_t output_max,
                                    float pressure_min, float pressure_max)
{
    if (!dev) {
        return nullptr;
    }
    AP_Baro_HSC *sensor = new AP_Baro_HSC(baro, std::move(dev), output_min, output_max, pressure_min, pressure_max);
    if (!sensor || !sensor->_init()) {
        delete sensor;
        return nullptr;
    }
    return sensor;
}

// initialise sensor
bool AP_Baro_HSC::_init()
{
    if (!_dev) {
        return false;
    }

    // prevent all other devices from accessing i2c during init
    _dev->get_semaphore()->take_blocking();

    // high retries for init
    _dev->set_retries(10);

    const char* name = "HSC Pressure Sensor";

    // attempt to read from the device to see if it is connected
    _read();
    if (_status != 0xff) {
        printf("%s found on bus %u address 0x%02x\n", name, _dev->bus_num(), _dev->get_bus_address());
    } else {
        // give back control of the i2c bus
        _dev->get_semaphore()->give();
        return false;
    }

    // register the sensor and set device type and bus id
    _instance = _frontend.register_sensor();
    _dev->set_device_type(DEVTYPE_BARO_HSC);
    set_bus_id(_instance, _dev->get_bus_id());

    // lower retries for run
    _dev->set_retries(3);

    // give back control of the i2c bus
    _dev->get_semaphore()->give();

    // request 100 Hz update
    _dev->register_periodic_callback(10 * AP_USEC_PER_MSEC, FUNCTOR_BIND_MEMBER(&AP_Baro_HSC::_read, void));

    return true;
}

void AP_Baro_HSC::_read()
{
    uint8_t buf[4];
    if (!_dev->transfer(nullptr, 0, buf, sizeof(buf))) {
        _status = 0xff;
        return;
    }

    _status = (buf[0] & 0xc0) >> 6;
    _bridge_data = ((buf[0] & 0x3f) << 8) + buf[1];
    _temperature_data = ((buf[2] << 8) + (buf[3] & 0xe0)) >> 5;
}

void AP_Baro_HSC::_calculate()
{
    // TODO: temperature-compensated pressure
    _pressure_pa = 1.0 * (_bridge_data - _output_min) * (_pressure_max - _pressure_min) / (_output_max - _output_min) + _pressure_min;
    _temperature_c = (_temperature_data * 0.09770395701) - 50.0;
}

void AP_Baro_HSC::update()
{
    _calculate();
    _copy_to_frontend(_instance, _pressure_pa, _temperature_c);
}

#endif // AP_BARO_HSC_ENABLED