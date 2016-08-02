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
#if CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_BBBMINI
#include <AP_HAL/GPIO.h>
#include <AP_HAL_Linux/GPIO_BBB.h>
#endif

// Number of backends allowed
#define WINGTIP_MAX_BACKENDS 2

// Define I2C bus
#define WINGTIP_BOARD_RESET_LEVEL 0
#define WINGTIP_I2C_BUS 1
#define WINGTIP_I2C_ADDR0 32
#define WINGTIP_I2C_ADDR1 33

class AP_Wingtip_Backend;

class AP_Wingtip
{
public:
    friend class AP_Wingtip_Backend;

    AP_Wingtip(void);

    // RPM driver types
    enum Wingtip_Type {
        WINGTIP_TYPE_NONE    = 0,
        WINGTIP_TYPE_X2      = 1,
        WINGTIP_TYPE_X4      = 2,
    };


    // The RPM_State structure is filled in by the backend driver
    struct Wingtip_State {
        uint8_t                instance;        // the instance number of this wingtip board
        float                  rate_rpm;        // measured rate in revs per minute
        uint32_t               last_reading_ms; // time of last reading
        float                  signal_quality;  // synthetic quality metric
    };

    // parameters for each instance
    AP_Int8  _type;

    static const struct AP_Param::GroupInfo var_info[];

    // detect and initialise any available rpm sensors
    void init(void);

    // update state of all rpm sensors. Should be called from main loop
    void update(void);

    // returns the number of sensors
    uint8_t num_sensors(void) const {
        return num_instances;
    }

    //  return RPM for a sensor and channel.
    uint16_t get_rpm(uint8_t board, uint8_t rpm_channel) const;
    uint16_t get_rpm(uint8_t rpm_channel) const {
        // default to board 0 if only channel is given
        return get_rpm(0, rpm_channel);
    }

    // return raw de for a sensor and channel.
    uint16_t get_de_raw(uint8_t board, uint8_t de_channel) const;
    uint16_t get_de_raw(uint8_t de_channel) const {
        // default to board 0 if only channel is given
        return get_de_raw(0, de_channel);
    }

    // return de for a sensor and channel.
    float    get_de(uint8_t board, uint8_t rpm_channel) const;
    float    get_de(uint8_t de_channel) const {
        // default to board 0 if only channel is given
        return get_de(0, de_channel);
    }

    // return if an instance is healthy
    bool healthy(uint8_t instance) const;

    // return if an instance is enabled
    bool enabled(uint8_t instance) const;

private:
    Wingtip_State state[WINGTIP_MAX_BACKENDS];
    AP_Wingtip_Backend *drivers[WINGTIP_MAX_BACKENDS];
    uint8_t num_instances:2;

    void detect_instance(uint8_t instance);
    void update_instance(uint8_t instance);
};
