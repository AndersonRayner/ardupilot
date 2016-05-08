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

#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_Param/AP_Param.h>
#include <AP_Math/AP_Math.h>
 
class AP_Wingtip
{
public:

    AP_Wingtip(void);

    // RPM driver types
    enum RPM_Type {
        RPM_TYPE_NONE    = 0,
        RPM_TYPE_PX4_PWM = 1
    };

    // The RPM_State structure is filled in by the backend driver
    struct RPM_State {
        uint8_t                instance;        // the instance number of this RPM
        float                  rate_rpm;        // measured rate in revs per minute
        uint32_t               last_reading_ms; // time of last reading
        float                  signal_quality;  // synthetic quality metric 
    };

    // parameters for each instance
    AP_Int8  _type[4];
    AP_Float _scaling[4];
    AP_Float _maximum[4];
    AP_Float _minimum[4];
    AP_Float _quality_min[4];

    static const struct AP_Param::GroupInfo var_info[];
    
    // Return the number of rpm sensor instances
    uint8_t num_sensors(void) const {
        return 1;
    }

    // detect and initialise any available rpm sensors
    void init(void);

    // update state of all rpm sensors. Should be called from main loop
    void update(void);

    /*
      return RPM for a sensor. Return -1 if not healthy
     */
    float get_rpm(uint8_t instance) const {
        return 1.0f;
    }

    /*
      return signal quality for a sensor.
     */
    float get_signal_quality(uint8_t instance) const {
        return 1.0f;
    }

    bool healthy(uint8_t instance) const;

    bool enabled(uint8_t instance) const;

private:

};
