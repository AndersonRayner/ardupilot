/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
#include "Copter.h"
#include <dirent.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

/*
 * control_manoeuvre.pde - init and run calls for manoeuvre, flight mode
 */

#define MANOEUVRE_LEVEL_RATE_RP_CD 1000
#define MANOEUVRE_LEVEL_RATE_RP_CD 1000
#define MANOEUVRE_LEVEL_RATE_Y_CD   750

#define MANOEUVRE_PILOT_OVERRIDE_TIMEOUT_MS 500

#define MANOEUVRE_DIRECTORY "/home/matt/sf_ardupilot/manoeuvres/"

// SysID state enum
enum SysID_state {
    SYSID_NOT_ACTIVE = 0,
    SYSID_PILOT_OVERRIDE = 1,
    SYSID_WAIT_STEADY = 2,
    SYSID_TWITCHING = 3,
    SYSID_FINISHED = 4,
};

// SysID axis being twitched
enum SysID_axis {
    SYSID_ROLL = 0,
    SYSID_PITCH = 1,
    SYSID_YAW = 2,
    SYSID_THROTTLE = 3,
};


// autotune_state_struct - hold state flags
struct manoeuvre_state_struct {
    SysID_state         state      ;    // 1 = pilot is overriding controls so we suspend tuning temporarily
    bool                reverse_dir;    // 0 = twitching in negative direction (i.e. left for roll), 1 = positive direction (i.e. right for roll)
    uint8_t             step       ;    // Sets current step through manoeuvres process
    uint8_t             axis       ;
    uint8_t             file_number;
} manoeuvre_state;

struct manoeuvre_sequence_struct {
    uint8_t       ID        ;    // Manoeuvre ID number                           [ - ]
    uint32_t      t_step    ;    // Time to keep doing this stem                  [ ms ]
    float         trim_pitch;    // Trim pitch angle to do manoeuvre (offsets de) [ deg ]
    float         dt        ;    // Climb rate of current step                    [ m/s?? ]
    float         de        ;    // Target pitch angle of current step            [ deg ]
    float         da        ;    // Target roll angle of current step             [ deg ]
    float         dr        ;    // Target yaw rate of current step               [ deg/s ]
    float         ch5       ;    // unused
    float         ch6       ;    // unused
} manoeuvre_sequence;

static uint32_t manoeuvre_override_time;                         // the last time the pilot overrode the controls
static uint32_t manoeuvre_start_time;                            // the start time of the manoeuvre sequence

float manoeuvre_target_angle;

// Manoeuvre file handling
DIR *dir;
struct dirent *ent;
FILE *manoeuvre_fid;


// manoeuvre_init - initialise manoeuvre controller
bool Copter::manoeuvre_init(bool ignore_checks)
{
    // only allow SysID manoeuvres from loiter or alt_hold
    if (control_mode != LOITER && control_mode != ALT_HOLD) {
        return false;
    }

    // ensure throttle is above zero
    if (ap.throttle_zero) {
        return false;
    }

    // ensure we are flying
    if (!motors.armed() || !ap.auto_armed || ap.land_complete) {
        return false;
    }

    // initialise vertical speeds and leash lengths
    pos_control.set_speed_z(-g.pilot_velocity_z_max, g.pilot_velocity_z_max);
    pos_control.set_accel_z(g.pilot_accel_z);

    // initialise position and desired velocity
    pos_control.set_alt_target(inertial_nav.get_altitude());
    pos_control.set_desired_velocity_z(inertial_nav.get_velocity_z());

    // stop takeoff if running
    takeoff_stop();

    gcs_send_text(MAV_SEVERITY_INFO,"SysID Manoeuvures Mode Engaged");

    manoeuvre_target_angle = 500.0f;

    manoeuvre_state.state = SYSID_NOT_ACTIVE;
    manoeuvre_state.axis = 1;
    manoeuvre_state.step = 0;
    manoeuvre_state.reverse_dir = 0;

    Log_Write_Manoeuvre(manoeuvre_state.state, manoeuvre_sequence.ID, manoeuvre_state.axis);

    // Print a list of the maneuvoure files in a directory
    if ((dir = opendir(MANOEUVRE_DIRECTORY)) != NULL) {
        hal.console->printf("successfullly opened manoeuvre directory\n");
    } else {
        hal.console->printf("failed to open manoeuvre directory\n");
        // failed to enter the mode if it can't find any manoeuvres to run.
        // is this the correct behaviour?
        return false;
    }

    // Load the first manoeuvre file
    manoeuvre_get_file();

    // Switch to pilot override mode to allow system to start
    manoeuvre_state.state = SYSID_PILOT_OVERRIDE;

    return true;
}

// manoeuvre_run - runs the manoeuvre controller
// should be called at 100hz or more
void Copter::manoeuvre_run()
{
    //const float direction_sign = manoeuvre_state.reverse_dir ? 1.0f : -1.0f;

    // initialize vertical speeds and acceleration
    pos_control.set_speed_z(-g.pilot_velocity_z_max, g.pilot_velocity_z_max);
    pos_control.set_accel_z(g.pilot_accel_z);

    // get pilot desired lean angles
    float target_roll, target_pitch;
    get_pilot_desired_lean_angles(channel_roll->get_control_in(), channel_pitch->get_control_in(), target_roll, target_pitch, attitude_control.get_althold_lean_angle_max());

    // get pilot's desired yaw rate
    float target_yaw_rate = get_pilot_desired_yaw_rate(channel_yaw->get_control_in());

    // get pilot desired climb rate
    float target_climb_rate = get_pilot_desired_climb_rate(channel_throttle->get_control_in());
    target_climb_rate = constrain_float(target_climb_rate, -g.pilot_velocity_z_max, g.pilot_velocity_z_max);

    // Check pilot controls and stop twitching if there is an input
    if (!is_zero(target_roll) || !is_zero(target_pitch) || !is_zero(target_yaw_rate) || target_climb_rate != 0) {

        // Pilot override, stop twitching...
        if (manoeuvre_state.state != SYSID_PILOT_OVERRIDE) {
            gcs_send_text(MAV_SEVERITY_INFO,"Pilot overriding controls");
            manoeuvre_state.state = SYSID_PILOT_OVERRIDE;
            Log_Write_Manoeuvre(manoeuvre_state.state, manoeuvre_sequence.ID, manoeuvre_state.axis);

            // set gains to their original (flyable) values
            // autotune_load_orig_gains();
            attitude_control.use_ff_and_input_shaping(true);

            // Rewind the file back to the start.  We'll need to try it again
            rewind(manoeuvre_fid);
            // read the first two line (header)
            char buf[100];
            fgets(buf,100, manoeuvre_fid);
            fgets(buf,100, manoeuvre_fid);

            // File is open and ready to be read
            // Read the first line to get everything ready
            manoeuvre_read_line();

        }

        // reset pilot override time
        manoeuvre_override_time = millis();

    } else if (manoeuvre_state.state == SYSID_PILOT_OVERRIDE) {
        // check if we should resume twitching after pilot's override
        if (millis() - manoeuvre_override_time > MANOEUVRE_PILOT_OVERRIDE_TIMEOUT_MS) {
            gcs_send_text(MAV_SEVERITY_INFO,"Waiting for steady vehicle");
            manoeuvre_state.state = SYSID_WAIT_STEADY;
            Log_Write_Manoeuvre(manoeuvre_state.state, manoeuvre_sequence.ID, manoeuvre_state.axis);

            // set gains to their intra-test values (which are very close to the original gains)
            // autotune_load_intra_test_gains(); //I think we should be keeping the originals here to let the I term settle quickly
            // autotune_state.step = AUTOTUNE_STEP_WAITING_FOR_LEVEL; // set tuning step back from beginning
            // autotune_desired_yaw = ahrs.yaw_sensor;
        }
    }

    // check for zero rates to start manoeuvre
    if (     manoeuvre_state.state == SYSID_WAIT_STEADY                        &&
            ((ToDeg(ahrs.get_gyro().x) * 100.0f) < MANOEUVRE_LEVEL_RATE_RP_CD) &&
            ((ToDeg(ahrs.get_gyro().y) * 100.0f) < MANOEUVRE_LEVEL_RATE_RP_CD) &&
            ((ToDeg(ahrs.get_gyro().z) * 100.0f) < MANOEUVRE_LEVEL_RATE_Y_CD )     ) {

        // Can start twitching if not rotating and pilot not inputting controls
        gcs_send_text(MAV_SEVERITY_INFO,"Beginning twitches");
        // set gains to twitching values
        //manoeuvre_load_twitch_gains();
        manoeuvre_state.state = SYSID_TWITCHING;
        Log_Write_Manoeuvre(manoeuvre_state.state, manoeuvre_sequence.ID, manoeuvre_state.axis);
        manoeuvre_start_time = millis();
    }

    // Twitch if allowed, overriding all other target controls
    if (manoeuvre_state.state == SYSID_TWITCHING) {
        // disable rate and throttle limits
        attitude_control.use_ff_and_input_shaping(false);
        motors.set_desired_spool_state(AP_Motors::DESIRED_THROTTLE_UNLIMITED);


        // Update control inputs if the time for the step has passed
        if (millis()-manoeuvre_start_time>manoeuvre_sequence.t_step) {
            // Read a new line and checks it exists
            if (manoeuvre_read_line()) {
                // Reset manoeuvre_start_time
                manoeuvre_start_time = millis();
            } else {
                // End of that file, increment to the next one
                if (manoeuvre_get_file()) {
                    // Reset back to waiting for steady to wait for next test to start
                    manoeuvre_state.state = SYSID_WAIT_STEADY;
                    Log_Write_Manoeuvre(manoeuvre_state.state, manoeuvre_sequence.ID, manoeuvre_state.axis);
                } else {
                    // no more files, reverts back to pilot control inputs
                }
            }
        } else {
            // Override controls with values from manoeuvre file
            target_climb_rate = manoeuvre_sequence.dt;
            target_pitch = manoeuvre_sequence.de*100.0f;//+manoeuvre_sequence.trim_pitch*100.0f; // Adding this stops it working...
            target_roll = manoeuvre_sequence.da*100.0f;
            target_yaw_rate = manoeuvre_sequence.dr;
        }
    }

    // call attitude controller
    attitude_control.input_euler_angle_roll_pitch_euler_rate_yaw(target_roll, target_pitch, target_yaw_rate, get_smoothing_gain());

    // call position controller
    pos_control.set_alt_target_from_climb_rate_ff(target_climb_rate, G_Dt, false);
    pos_control.update_z_controller();

}

void Copter::manoeuvre_stop()
{
    // close the manoeuvres directory.  This will cause the manoeuvre index to reset
    // upon re-entering manoeuvre mode
    closedir (dir);

    // set gains to their original values
    //manoeuvre_load_orig_gains();

    // re-enable angle-to-rate request limits
    attitude_control.use_ff_and_input_shaping(true);

    manoeuvre_state.state = SYSID_NOT_ACTIVE;
    Log_Write_Manoeuvre(manoeuvre_state.state, manoeuvre_sequence.ID, manoeuvre_state.axis);
}

bool Copter::manoeuvre_get_file()
{
    // Gets the next manouvre file and stores it in the global struct << ent >>
    // returns 1 if successful, 0 if no more files left
    //
    // ent->d_type == 4      -> folder
    //             == 8      -> file

    const char *ext;
    char manoeuvre_file[60];
    char buf[100];

    while ((ent = readdir (dir)) != NULL) {
        hal.console->printf("  name %10s | type %u\n", ent->d_name,ent->d_type);
        if (ent->d_type == 8) {
            // is a file, check the extension
            ext = strrchr(ent->d_name, '.');

            if (strcmp(ext,".man") == 0) {
                // Appears to be a manoeuvre file
                hal.console->printf("      correct extension -> %s\n", ext);

                // Try opening and reading the file
                sprintf(manoeuvre_file, "%s%s",MANOEUVRE_DIRECTORY,ent->d_name);
                hal.console->printf("      trying to open %s\n",manoeuvre_file);

                if ((manoeuvre_fid = fopen(manoeuvre_file,"r"))) {
                    hal.console->printf("         file successfully opened\n\n");

                    // read the first two line (header)
                    fgets(buf,100, manoeuvre_fid);
                    fgets(buf,100, manoeuvre_fid);

                    // File is open and ready to be read
                    // Read the first line
                    manoeuvre_read_line();
                    return 1;

                } else {
                    hal.console->printf("         failed file open\n");
                }
            } else {
                hal.console->printf("      incorrect extension -> %s\n", ext);
                // file has no or incorrect extension, keep searching
            }
        }
    }

    // If we get to here, there are no more manoeuvres to run
    hal.console->printf("No more manoeuvres to run\n");

    // Set SysID state to finished and log
    manoeuvre_state.state = SYSID_FINISHED;
    Log_Write_Manoeuvre(manoeuvre_state.state, manoeuvre_sequence.ID, manoeuvre_state.axis);

    // switch flight mode back to altitude hold
    set_mode(ALT_HOLD, MODE_REASON_MISSION_END);

    // Inform the GCS
    // gcs_send_text(MAV_SEVERITY_INFO,"SysID Finished"); // it may not like this

    // Set t_step so that it indefinately runs the previous input until the mode changes
    manoeuvre_sequence.t_step = -1;

    // Return 0 to indicate we're finished
    return 0;
}

bool Copter::manoeuvre_read_line()
{
    // Returns 1 on successful line read
    //         0 on a failed line read (also causes the file to be closed)

    char buf[100];

    if ((fgets(buf,100, manoeuvre_fid)!=NULL)) {
        sscanf(buf, "%u %u %f %f %f %f %f %f %f",
                &manoeuvre_sequence.ID,
                &manoeuvre_sequence.t_step,
                &manoeuvre_sequence.trim_pitch,
                &manoeuvre_sequence.dt,
                &manoeuvre_sequence.de,
                &manoeuvre_sequence.da,
                &manoeuvre_sequence.dr,
                &manoeuvre_sequence.ch5,
                &manoeuvre_sequence.ch6  );

        // need some way to check if this was vaild...
        hal.console->printf("Running: %3u %6u %6.1f %6.1f %6.1f %6.1f %6.1f %6.1f %6.1f\n",
                manoeuvre_sequence.ID,
                manoeuvre_sequence.t_step,
                manoeuvre_sequence.trim_pitch,
                manoeuvre_sequence.dt,
                manoeuvre_sequence.de,
                manoeuvre_sequence.da,
                manoeuvre_sequence.dr,
                manoeuvre_sequence.ch5,
                manoeuvre_sequence.ch6  );

        // Return and indicate we have something to run
        return 1;

    } else {
        // That's the end of the file.  Time to close it
        fclose(manoeuvre_fid);
    }

    // If we get to here, then there is nothing left to run in this file
    return 0;
}


// Additional code for use later
/*
CHANGING THE GAINS
attitude_control.get_rate_pitch_pid().kP(tune_pitch_rp);
attitude_control.get_rate_pitch_pid().kI(tune_pitch_rp*AUTOTUNE_PI_RATIO_FINAL);
attitude_control.get_rate_pitch_pid().kD(tune_pitch_rd);
attitude_control.get_rate_pitch_pid().save_gains();

DOING THE TWITCH
attitude_control.input_euler_angle_roll_pitch_euler_rate_yaw( 0.0f, 0.0f, 0.0f);
switch (autotune_state.axis) {
case AUTOTUNE_AXIS_ROLL:
// override body-frame roll rate
attitude_control.rate_bf_roll_target(direction_sign * autotune_target_rate + autotune_start_rate);
break;
case AUTOTUNE_AXIS_PITCH:
// override body-frame pitch rate
attitude_control.rate_bf_pitch_target(direction_sign * autotune_target_rate + autotune_start_rate);
break;
case AUTOTUNE_AXIS_YAW:
// override body-frame yaw rate
attitude_control.rate_bf_yaw_target(direction_sign * autotune_target_rate + autotune_start_rate);
break;

NEED TO SWITCH EVERYTHING BACK AFTER EXIT FROM FLIGHT MODE
 */
