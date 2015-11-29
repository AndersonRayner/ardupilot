/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include "Copter.h"

/*
 * control_quatstab.cpp - init and run calls for stabilize flight mode
 *  Set using mode 18
 */

// quatstab_init - initialise quaternion-based stabilize controller
bool Copter::quatstab_init(bool ignore_checks)
{
    // set target altitude to zero for reporting
    // To-Do: make pos controller aware when it's active/inactive so it can always report the altitude error?
    pos_control.set_alt_target(0);

    // stabilize should never be made to fail
    return true;
}

// quatstab_run - runs the main quaternion-based stabilize controller
// should be called at 100hz or more
void Copter::quatstab_run()
{
    float target_roll, target_pitch;
    float target_yaw_rate;
    int16_t pilot_throttle_scaled;

    // if not armed or throttle at zero, set throttle to zero and exit immediately
    if(!motors.armed() || ap.throttle_zero) {
        attitude_control.set_throttle_out_unstabilized(0,true,g.throttle_filt);
        // slow start if landed
        if (ap.land_complete) {
            motors.slow_start(true);
        }
        return;
    }

    // apply SIMPLE mode transform to pilot inputs
    // update_simple_mode(); // I don't think I want to be using this

    // convert pilot input to lean angles
    // To-Do: convert get_pilot_desired_lean_angles to return angles as floats
    get_pilot_desired_lean_angles(channel_roll->control_in, channel_pitch->control_in, target_roll, target_pitch, aparm.angle_max);

    // get pilot's desired yaw rate
    target_yaw_rate = get_pilot_desired_yaw_rate(channel_yaw->control_in);

    // get pilot's desired throttle
    pilot_throttle_scaled = get_pilot_desired_throttle(channel_throttle->control_in);

    // call attitude controller
    // attitude_control.angle_ef_roll_pitch_rate_ef_yaw_smooth(target_roll, target_pitch, target_yaw_rate, get_smoothing_gain());
    attitude_control.angle_ef_roll_pitch_rate_ef_yaw_quat(target_roll, target_pitch, target_yaw_rate);

    // body-frame rate controller is run directly from 100hz loop

    // output pilot's throttle
    attitude_control.set_throttle_out(pilot_throttle_scaled, true, g.throttle_filt);

    // Debugging stuff
   /* static int counter;
    counter++;
    if (counter == 200)
    {
        hal.console->printf("Running Quaternion-based control\n");
        counter = 0;
    } */


}
