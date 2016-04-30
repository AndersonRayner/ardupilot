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
AP_HAL::DigitalSource *_cs;

// table of user settable parameters
/*const AP_Param::GroupInfo AP_Wingtip::var_info[] = {
    // @Param: _TYPE
    // @DisplayName: RPM type
    // @Description: What type of RPM sensor is connected
    // @Values: 0:None,1:PX4-PWM
    AP_GROUPINFO("_TYP0",    0, AP_Wingtip, _type[0], 0),

    AP_GROUPINFO("_TYP1",    1, AP_Wingtip, _type[1], 0),

    AP_GROUPEND
};*/

AP_Wingtip::AP_Wingtip(void) 
{
   // hal.console->printf("AP_Wingtip:constructing...\n");  // Was causing crashes
   // AP_Param::setup_object_defaults(this, var_info);
   
    // init state and drivers
  //  memset(state,0,sizeof(state));
  //  memset(drivers,0,sizeof(drivers));
}

/*
  initialise the AP_Wingtip class. 
 */
void AP_Wingtip::init(void)
{
    // Want to think about resetting the boards here.
    _cs = hal.gpio->channel(BBB_P9_15);
    if (_cs == NULL) {
        AP_HAL::panic("Unable to reset wingtip boards");
    }

    _cs->mode(HAL_GPIO_OUTPUT);
    _cs->write(0);       // low resets the board
    hal.scheduler->delay(5);
    _cs->write(1);       // go high to let it do it's thing

}

/*
  update RPM state for all instances. This should be called by main loop
 */
void AP_Wingtip::update(void)
{
    union wingtip_data data1;
    union wingtip_data data2;

    uint64_t time_us1 = AP_HAL::micros64();
    uint64_t time_us2 = AP_HAL::micros64();

    hal.i2c1->read(0x32, 7, data1.rxBuffer);
	if (data1.rxBuffer[6] == 0x50) {
        _RPM[0] = data1.data[0];
        _RPM[1] = data1.data[1];
        _de[0]  = (float)data1.data[2];
	} else {
		// sensor not healthy, what to do?
		_RPM[0] = 0;
		_RPM[1] = 0;
		_de[0]  = 0.0f;
	}

    time_us2 = AP_HAL::micros64();
    hal.console->printf("t1 = %6llu csum: 0x%02x  ", (time_us2-time_us1), data1.rxBuffer[6]);

    hal.i2c1->read(0x35, 7, data2.rxBuffer);
	if (data2.rxBuffer[6] == 0x50) {
        _RPM[2] = data2.data[0];
        _RPM[3] = data2.data[1];
        _de[1]  = (float)data2.data[2];
	} else {
		// sensor not healthy, what to do?
		_RPM[2] = 0;
		_RPM[3] = 0;
		_de[1]  = 0.0f;
	}

    time_us1 = AP_HAL::micros64();
    hal.console->printf("t2 = %6llu csum: 0x%02x  ", (time_us1-time_us2), data2.rxBuffer[6]);
}

/*
  check if an instance is healthy
 */
bool AP_Wingtip::healthy(uint8_t instance) const
{
    hal.console->printf("AP_Wingtip:healthy - testing...\n");
    return true;
}

/*
  check if an instance is activated
 */
bool AP_Wingtip::enabled(uint8_t instance) const
{
    hal.console->printf("AP_Wingtip:enabled - testing...\n");
    return true;
}
