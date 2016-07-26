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
 *   calibration_generic.cpp - Calibration library example sketch
 *
 */

#include <AP_HAL/AP_HAL.h>
#include <AP_Calibration/AP_Calibration.h>

const AP_HAL::HAL& hal = AP_HAL::get_HAL();
static AP_Calibration calib;

void setup()
{
    hal.console->println("APM Calibration library\n\n");

}

void loop(void)
{

    int16_t user_input;

    hal.console->println();
    hal.console->println(
    "Menu:\n"
    "    i) calibrate IMU\n"
    "    c) calibrate compass\n"
    "    a) calibrate control surface\n"
    "    r) calibrate RPM\n"
    "    m) calibrate motcompass\n");

    // wait for user input
    while (!hal.console->available()) {
        hal.scheduler->delay(20);
    }

    // read in user input
    while (hal.console->available()) {
        user_input = hal.console->read();

        if (user_input == 'i' || user_input == 'I') {
        	hal.console->print("Starting IMU calibration\n");
            calib.calibrate_IMU();
        }

        if (user_input == 'c' || user_input == 'C') {
        	hal.console->print("Starting COMPASS calibration\n");
            calib.calibrate_compass();
        }

        if (user_input == 'a' || user_input == 'A') {
        	hal.console->print("Starting AILERON calibration\n");
            calib.calibrate_controls();
        }

        if (user_input == 'r' || user_input == 'R') {
        	hal.console->print("Starting RPM calibration\n");
            calib.calibrate_RPM();
        }

        if (user_input == 'm' || user_input == 'M') {
            hal.console->print("Starting motor / compass calibration\n");
            calib.calibrate_motcompass();
        }
    }
}

AP_HAL_MAIN();
