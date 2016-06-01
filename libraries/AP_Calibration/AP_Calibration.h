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
#pragma once

#define CALIBRATION_DIR "/root/APM/Calibration/"

#include <fcntl.h>
#include <stdlib.h>
#include <stdio.h>
#include <iostream>
#include <sys/stat.h>

#include <AP_HAL/AP_HAL.h>

#include <AP_Common/AP_Common.h>
#include <AP_HAL/I2CDevice.h>
#include <AP_HAL/SPIDevice.h>
#include <AP_Math/AP_Math.h>

#include <AP_InertialSensor/AP_InertialSensor.h>
#include <AP_Compass/AP_Compass.h>
#include <AP_Wingtip/AP_Wingtip.h>

class AP_Calibration
{
public:
    AP_Calibration(void);

    void calibrate_IMU(void);
    void calibrate_compass(void);
    void calibrate_controls(void);
    void calibrate_RPM(void);

private:

};
