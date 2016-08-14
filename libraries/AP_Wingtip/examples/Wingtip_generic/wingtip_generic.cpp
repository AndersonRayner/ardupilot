/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
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

/*
 *   wingtip_generic.cpp - RPM library example sketch
 *
 */

#include <AP_HAL/AP_HAL.h>
#include <AP_Wingtip/AP_Wingtip.h>

#define WINGTIP_TYPE 2

const AP_HAL::HAL& hal = AP_HAL::get_HAL();
static AP_Wingtip Wingtip;

void setup()
{
    hal.console->println("APM Wingtip Sensor library test\n\n");
    AP_Param::set_object_value(&Wingtip, Wingtip.var_info, "_TYPE", WINGTIP_TYPE);
    Wingtip.init();

}

void loop(void)
{
    uint64_t time1_us = AP_HAL::micros64();
    uint64_t time2_us = AP_HAL::micros64();

    Wingtip.update();

    time2_us = AP_HAL::micros64();

    hal.console->printf("t1 = %6llu us  | ", (time2_us-time1_us));

    hal.console->printf("RPM : ");
    for (uint8_t ii = 0; ii<4; ii++) {
        hal.console->printf("%6u ",Wingtip.get_rpm(0,ii));
    }

    hal.console->printf("   de : ");
    for (uint8_t ii = 0; ii<4; ii++) {
        hal.console->printf("%6u ",Wingtip.get_de_raw(0,ii));
    }

    hal.console->printf("   i2c_err : %3u",Wingtip.get_i2c_lockups(0));

    hal.scheduler->delay(20);

    hal.console->printf("\n");
}

AP_HAL_MAIN();
