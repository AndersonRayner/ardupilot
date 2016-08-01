// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
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

#include "AP_Wingtip.h"
#include <AP_HAL/I2CDevice.h>

extern const AP_HAL::HAL& hal;

// Define reset pins
#if CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_BBBMINI
AP_HAL::DigitalSource *_cs;
#define _TYPE_default 1
#else
#define _TYPE_default 0
#endif

// Define I2C bus
#define WINGTIP_BOARD_RESET_LEVEL 0
#define WINGTIP_I2C_BUS 1
#define WINGTIP_I2C_ADDR0 32
#define WINGTIP_I2C_ADDR1 33

// table of user settable parameters
const AP_Param::GroupInfo AP_Wingtip::var_info[] = {
        // @Param: _TYPE
        // @DisplayName: Wingtip sensor type
        // @Description: What type of wingtip sensor is connected
        // @Values: 0:Faked,1:I2C_Wingtip,2:I2C_Wingtip x4
        AP_GROUPINFO("_TYPE",    0, AP_Wingtip, _type, _TYPE_default),

        AP_GROUPEND
};

AP_Wingtip::AP_Wingtip(void)
{
    AP_Param::setup_object_defaults(this, var_info);

}

//  initialise the AP_Wingtip class.
void AP_Wingtip::init(void)
{
    memset(_healthy,0,sizeof(_healthy));  // Haven't received a reading yet -> assume not healthy
    memset(_enabled,1,sizeof(_enabled));
    memset(_RPM,0,sizeof(_RPM));
    memset(_de,0,sizeof(_de));


    // Reset the external boards
#if CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_BBBMINI
        _cs = hal.gpio->channel(BBB_P9_15);
        if (_cs == NULL) {
            AP_HAL::panic("Unable to reset wingtip boards");
        }

        _cs->mode(HAL_GPIO_OUTPUT);
        _cs->write(WINGTIP_BOARD_RESET_LEVEL);       // high resets the board
        hal.scheduler->delay(5);
        _cs->write(!WINGTIP_BOARD_RESET_LEVEL);       // go low to let it do it's thing
        hal.scheduler->delay(5);
#endif


     _dev = hal.i2c_mgr->get_device(WINGTIP_I2C_BUS, WINGTIP_I2C_ADDR0);

}

// Update all the wingtip data
void AP_Wingtip::update(void)
{
    switch (_type) {
    {
    case 0 : // Fake the data
        _RPM[0]++;
        _RPM[1]++;
        _RPM[2]++;
        _RPM[3]++;

        _de_raw[0]++;
        _de_raw[1]++;

        _healthy[0] = 1;  // rpm1, rpm2, de1
        _healthy[1] = 1; // rpm3, rpm4, de2

        break;
    }
    case 1 :  // From the wingtip boards
    {

       /*
        *
        *
        *     _dev = hal.i2c_mgr->get_device(WINGTIP_I2C_BUS, WINGTIP_I2C_ADDR0);
    _dev = hal.i2c_mgr->get_device(WINGTIP_I2C_BUS, WINGTIP_I2C_ADDR1);
        *
        * union wingtip_data data1;
        union wingtip_data data2;

        uint8_t CRC;

        // Take semaphore
        if (!i2c1_sem->take(5)) {
            return;
        }

        // Read the first wingtip board
        hal.i2c1->read(0x32, 7, data1.rxBuffer);

        // Calculate checksum
        CRC = 0;
        for (uint8_t ii = 0; ii<6; ii++) {
            CRC = CRC ^ data1.rxBuffer[ii];
        }

        if (data1.rxBuffer[6] == CRC) {
            _RPM[0]    = data1.data[0];
            _RPM[1]    = data1.data[1];
            _de_raw[0] = data1.data[2];

            _healthy[0] = 1; // rpm1, rpm2, de1

        } else {
            // mark sensor unhealthy
            _healthy[0] = 0; // rpm1, rpm2, de1
        }

        // Read the second wingtip board
        hal.i2c1->read(0x33, 7, data2.rxBuffer);

        // Return semaphore
        i2c1_sem->give();

        // Calculate checksum
        CRC = 0;
        for (uint8_t ii = 0; ii<6; ii++) {
            CRC = CRC ^ data2.rxBuffer[ii];
        }

        if (data2.rxBuffer[6] == CRC) {
            _RPM[2]    = data2.data[0];
            _RPM[3]    = data2.data[1];
            _de_raw[1] = data1.data[2];

            _healthy[1] = 1; // rpm3, rpm4, de2

        } else {
            // mark sensor unhealthy
            _healthy[1] = 0; // rpm3, rpm4, de2
        }*/

        break;
    }
    case 2 :    // Wingtip board that supplied all four RPMs
    {
        union wingtip_data data1;
        uint8_t CRC;

        // take i2c bus sempahore
        if (!_dev || !_dev->get_semaphore()->take(15)) {
            return;
        }

        // Read data from board
        if (!_dev->transfer(nullptr, 0, data1.rxBuffer, sizeof(data1.rxBuffer))) {
            return;
        }

        // Return semaphore
        _dev->get_semaphore()->give();

        // Calculate checksum
        CRC = 0;
        for (uint8_t ii = 0; ii<8; ii++) {
            CRC = CRC ^ data1.rxBuffer[ii];
        }

        if (data1.rxBuffer[8] == CRC) {
            _RPM[0] = data1.data[0];
            _RPM[1] = data1.data[1];
            _RPM[2] = data1.data[2];
            _RPM[3] = data1.data[3];

            _healthy[0] = 1; // rpm1, rpm2, de1
            _healthy[1] = 1; // rpm3, rpm4, de2

        } else {
            // mark sensor unhealthy
            _healthy[0] = 0; // rpm1, rpm2, de1
            _healthy[1] = 0; // rpm3, rpm4, de2
        }

        break;
    }
    default :
    {
        // do nothing at the moment
        // hal.console->printf("No type recognised!!! AP_Wingtip._type");
        break;
    }
    }

    // Convert de_raw values to a calibrated de
    _de[0]  = (float)_de_raw[0];
    _de[1]  = (float)_de_raw[1];
}


//  Check if an instance is healthy
bool AP_Wingtip::healthy(uint8_t instance) const
{
    if (instance >= 1) {
        return false;
    } else {
        return _healthy[instance];
    }
}


// Check if an instance is activated
bool AP_Wingtip::enabled(uint8_t instance) const
{
    if (instance >= 1) {
        return false;
    } else {
        return _enabled[instance];
    }
}

// Return the RPM for an instance.  Return 0 if not healthy
uint16_t AP_Wingtip::get_rpm(uint8_t instance) const {
    if (instance >= 6) {
        return false;
//    } else if (healthy(instance)) {
 //       return _RPM[instance];
    } else {
     //   return 0;
        return _RPM[instance];
    }
}

// return raw de for a sensor.  Return 0 if not healthy
uint16_t AP_Wingtip::get_de_raw(uint8_t instance) const {
   // if (healthy(instance+4)) {
        return _de_raw[instance];
  //  } else {
    //    return 0.0f;
  //  }
}


// return de for a sensor.  Return 0 if not healthy
float AP_Wingtip::get_de(uint8_t instance) const {
 //   if (healthy(instance+4)) {
        return _de[instance];
  //  } else {
 //       return 0.0f;
 //   }
}
