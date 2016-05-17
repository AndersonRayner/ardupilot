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
extern const AP_HAL::HAL& hal;
#if CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_BBBMINI
AP_HAL::DigitalSource *_cs;
#endif

// table of user settable parameters
const AP_Param::GroupInfo AP_Wingtip::var_info[] = {
        // @Param: _TYPE
        // @DisplayName: Wingtip sensor type
        // @Description: What type of wingtip sensor is connected
        // @Values: 0:Faked,1:I2C_Wingtip
        AP_GROUPINFO("_TYPE",    0, AP_Wingtip, _type, 0),


        AP_GROUPEND
};

AP_Wingtip::AP_Wingtip(void)
{
    AP_Param::setup_object_defaults(this, var_info);

}

//  initialise the AP_Wingtip class.
void AP_Wingtip::init(void)
{
    memset(_healthy,1,sizeof(_healthy));
    memset(_enabled,1,sizeof(_enabled));

    if (_type == 0) {
        _RPM[0] =   0;
        _RPM[1] = 100;
        _RPM[2] = 200;
        _RPM[3] = 300;

        _de[0] =   0.0f;
        _de[1] = 100.0f;

    } else {
        memset(_RPM,0,sizeof(_RPM));
        memset(_de,0,sizeof(_de));

#if CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_BBBMINI
        // Reset the external boards
        _cs = hal.gpio->channel(BBB_P9_15);
        if (_cs == NULL) {
            AP_HAL::panic("Unable to reset wingtip boards");
        }

        _cs->mode(HAL_GPIO_OUTPUT); // This high low thing might have to change as I think the logic went inverted on the new boards
        _cs->write(0);       // low resets the board
        hal.scheduler->delay(5);
        _cs->write(1);       // go high to let it do it's thing
#endif

    }
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

        _de[0]++;
        _de[1]++;

        for (uint8_t ii = 0; ii<6; ii++) {
            _healthy[ii] = 1;
        }

        break;
    }
    case 1 :  // From the wingtip boards
    {
#if CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_BBBMINI

        union wingtip_data data1;
        union wingtip_data data2;

        uint64_t time_us1 = AP_HAL::micros64();
        uint64_t time_us2 = AP_HAL::micros64();

        uint8_t CRC;

        // Read the first wingtip board
        hal.i2c1->read(0x32, 7, data1.rxBuffer);

        // Calculate checksum
        CRC = 0;
        for (uint8_t ii = 0; ii<6; ii++) {
            CRC = CRC ^ data1.rxBuffer[ii];
        }

        if (data1.rxBuffer[6] == CRC) {
            _RPM[0] = data1.data[0];
            _RPM[1] = data1.data[1];
            _de[0]  = (float)data1.data[2];
        } else {
            // mark sensor unhealthy
            _healthy[0] = 0; // rpm1
            _healthy[1] = 0; // rpm2
            _healthy[4] = 0; //  de1
        }

        //   time_us2 = AP_HAL::micros64();
        //    hal.console->printf("t1 = %6llu csum: 0x%02x  ", (time_us2-time_us1), data1.rxBuffer[6]);

        // Read the second wingtip board
        hal.i2c1->read(0x33, 7, data2.rxBuffer);

        // Calculate checksum
        CRC = 0;
        for (uint8_t ii = 0; ii<6; ii++) {
            CRC = CRC ^ data2.rxBuffer[ii];
        }

        if (data2.rxBuffer[6] == CRC) {
            _RPM[2] = data2.data[0];
            _RPM[3] = data2.data[1];
            _de[1]  = (float)data2.data[2];
        } else {
            // mark sensor unhealthy
            _healthy[2] = 0; // rpm3
            _healthy[3] = 0; // rpm4
            _healthy[5] = 0; //  de2
        }

        //   time_us1 = AP_HAL::micros64();
        //   hal.console->printf("t2 = %6llu csum: 0x%02x  ", (time_us1-time_us2), data2.rxBuffer[6]);
#endif

        break;
    }
    default :
    {
        // do nothing at the moment
        // hal.console->printf("No type recognised!!! AP_Wingtip._type");
    }
    }
}


//  Check if an instance is healthy
bool AP_Wingtip::healthy(uint8_t instance) const
{
    if (instance >= 6) {
        return false;
    } else {
        return _healthy[instance];
    }
}


// Check if an instance is activated
bool AP_Wingtip::enabled(uint8_t instance) const
{
    if (instance >= 6) {
        return false;
    } else {
        return _enabled[instance];
    }
}

// Return the RPM for an instance.  Return -1 if not healthy
uint16_t AP_Wingtip::get_rpm(uint8_t instance) const {
    if (instance >= 6) {
        return false;
    } else if (healthy(instance)) {
        return _RPM[instance];
    } else {
        return -1;
    }
}

// return de for a sensor.  Return -1 if not healthy
float AP_Wingtip::get_de(uint8_t instance) const {
    if (healthy(instance+4)) {
        return _de[instance];
    } else {
        return -1.0f;
    }
}
