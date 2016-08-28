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

// Number of backends allowed
#define WINGTIP_MAX_BACKENDS 2

// Define I2C bus
#define WINGTIP_BOARD_RESET_LEVEL 1
#define WINGTIP_I2C_BUS 1
#define WINGTIP_I2C_ADDR0 0x32

class AP_Wingtip_Backend;

class AP_Wingtip
{
public:
    friend class AP_Wingtip_Backend;

    AP_Wingtip(void);

    // RPM driver types
    enum Wingtip_Type {
        WINGTIP_TYPE_DISABLED = 0,
        WINGTIP_TYPE_SIM      = 1,
        WINGTIP_TYPE_X2       = 2,
        WINGTIP_TYPE_X4       = 3,
    };


    // The RPM_State structure is filled in by the backend driver
    struct Wingtip_State {
        bool                   enabled;
        bool                   healthy;
        uint8_t                instance;        // the instance number of this wingtip board
        uint8_t                i2c_lockups;     // number of i2c lockups

        uint64_t               last_reading_us; // time of last reading
        uint16_t               rpm[4];          // up to four RPMs per board
        uint16_t               de_raw[4];       // up to four de (raw) readings per board
        float                  de[4];           // up to four de readings per board
    };

    // parameters for each instance
    AP_Int8  _type[WINGTIP_MAX_BACKENDS];

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
    uint16_t get_rpm(uint8_t board, uint8_t channel) const;
    uint16_t get_rpm(uint8_t channel) const {
        // default to board 0 if only channel is given
        return get_rpm(0, channel);
    }

    // return raw de for a sensor and channel.
    uint16_t get_de_raw(uint8_t board, uint8_t channel) const;
    uint16_t get_de_raw(uint8_t channel) const {
        // default to board 0 if only channel is given
        return get_de_raw(0, channel);
    }

    // return de for a sensor and channel.
    float    get_de(uint8_t board, uint8_t channel) const;
    float    get_de(uint8_t channel) const {
        // default to board 0 if only channel is given
        return get_de(0, channel);
    }

    // return if an instance is healthy
    bool healthy(uint8_t board) const;

    // return if an instance is enabled
    bool enabled(uint8_t board) const;

    // return the time the last reading was taken
    // need to do this with a sync pin I think
    uint64_t get_last_reading_us(uint8_t board) const;

    uint8_t get_i2c_lockups(uint8_t board) const {
        return state[board].i2c_lockups;
    }

private:
    Wingtip_State state[WINGTIP_MAX_BACKENDS];
    AP_Wingtip_Backend *drivers[WINGTIP_MAX_BACKENDS];
    uint8_t num_instances:2;

    bool _sync_level;

};
