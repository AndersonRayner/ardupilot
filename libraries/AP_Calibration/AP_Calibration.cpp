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

#include "AP_Calibration.h"

extern const AP_HAL::HAL& hal;

static AP_InertialSensor ins;
static Compass compass;
static AP_Wingtip wingtip;

AP_Calibration::AP_Calibration(void)
{
  // do nothing
}

void AP_Calibration::calibrate_IMU(void) {
    Vector3f accel;
    Vector3f gyro;

    hal.console->println("Calibrating inertial sensors...");
    ins.init(100);

    // display number of detected accels/gyros
    hal.console->printf("    Number of detected accels : %u\n", ins.get_accel_count());
    hal.console->printf("    Number of detected gyros  : %u\n\n", ins.get_gyro_count());

    // Reset all of the IMU parameters
    for (int ii = 0; ii<ins.get_accel_count(); ii++)
    {
        // Accelerometers
        //ins._accel_cal_x[ii].set_and_save(Vector3f(1.0f,0,0));
        //ins._accel_cal_y[ii].set_and_save(Vector3f(0,1.0f,0));
        //ins._accel_cal_z[ii].set_and_save(Vector3f(0,0,1.0f));
        //ins._accel_offset[ii].set_and_save(Vector3f(0,0,0));
        // Gyros
        //ins._gyro_cal_x[ii].set_and_save(Vector3f(1.0f,0,0));
        //ins._gyro_cal_y[ii].set_and_save(Vector3f(0,1.0f,0));
        //ins._gyro_cal_z[ii].set_and_save(Vector3f(0,0,1.0f));
    }


    // Create a calibration file
    hal.console->printf("Making calibration file\n");
    mkdir(CALIBRATION_DIR, 0777);     // Create the directory for things to be stored into

    // Need six different known positions for the calibration to work
    for (int jj = 0; jj<6; jj++)
    {
        // Open the file
        char str_base[20];

        switch(jj) {
        case 0 :
            hal.console->printf("Z_down\n");
            sprintf(str_base, "%s", "Z_down.txt");
            break;

        case 1 :
            hal.console->printf("Z_up\n");
            sprintf(str_base, "%s", "Z_up.txt");
            break;

        case 2 :
            hal.console->printf("Y_down\n");
            sprintf(str_base, "%s", "Y_down.txt");
            break;

        case 3 :
            hal.console->printf("Y_up\n");
            sprintf(str_base, "%s", "Y_up.txt");
            break;

        case 4 :
            hal.console->printf("X_down\n");
            sprintf(str_base, "%s", "X_down.txt");
            break;

        case 5 :
            hal.console->printf("X_up\n");
            sprintf(str_base, "%s", "X_up.txt");
            break;

        default :
            hal.console->printf("Iteration %d!\n",jj);
            sprintf(str_base, "%d.txt", jj);
        }

        // Clear any user input buffer
        while( hal.console->available() ) {
            hal.console->read();
        }

        // Wait for user to confirm to take reading
        hal.console->printf("Press < return > to continue\n");
        while( !hal.console->available() ) {
            hal.scheduler->delay(20);
        }

        hal.console->printf("Starting recording\n");

        // Loop for each sensor
        for (int kk = 0; kk<ins.get_accel_count(); kk++)
        {

            // Start the data collection
            char str[20];
            sprintf(str, "%s%d-%s",CALIBRATION_DIR,kk,str_base);
            FILE *f = fopen(str,"w");

            if (f == NULL)
            {
                printf("Error opening file!\n");
                exit(1);
            }

            fprintf(f,"Accelerometer Calibration File\n");
            fprintf(f,"=======================\n");

            // Write data points
            for (int ii = 0; ii<500; ii++)
            {
                // wait until we have a sample
                ins.wait_for_sample();

                // read samples from ins
                ins.update();

                accel = ins.get_accel(kk);

                fprintf(f,"%f,%f,%f\n",accel.x, accel.y, accel.z);
            }

            fclose(f);
        }

        // Close the file
        hal.console->printf("Done file %d!\n\n",jj);
    }

    // Return
    hal.console->printf("Collected all the data.  Returning...");
    return;
}

void AP_Calibration::calibrate_compass(void) {
    hal.console->println("Calibrating compasses...");
    compass.init();

    // display number of detected accels/gyros
    hal.console->printf("\n");
    hal.console->printf("Number of detected magnetometers : %u\n", compass.get_count());

    // Reset all of the compass parameters


    // Create a calibration file
    hal.console->printf("Making calibration file\n");
    mkdir(CALIBRATION_DIR, 0777);     // Create the directory for things to be stored into

    // Clear any user input buffer
    while( hal.console->available() ) {
        hal.console->read();
    }

    // Open the required number of files
    FILE *f[3];
    for (uint8_t kk=0; kk<compass.get_count(); kk++)
    {
        char str[20];

        sprintf(str, "%s%d-%s",CALIBRATION_DIR,kk,"compass.txt");

        f[kk] = fopen(str,"w");

        if (f[kk] == NULL)
        {
           // hal.console>printf("Error opening file for compass!\n");
           // exit(1);
        }

        fprintf(f[kk],"Magnetometer Calibration File\n");
        fprintf(f[kk],"=============================\n");
    }

    // Wait for user to confirm to take reading
    hal.console->printf("Rotate the vehicle around at least three axes\n\n");
    hal.console->printf("Press < return > to start calibration\n");
    while( !hal.console->available() ) {
        hal.scheduler->delay(20);
    }


    // Collect the data
    for (int ii = 0; ii<500; ii++)
    {
        compass.accumulate();
        compass.read();

        for (uint8_t kk=0; kk<compass.get_count(); kk++)
        {
            const Vector3f &mag = compass.get_field(kk);
            fprintf(f[kk],"%f,%f,%f\n",mag.x, mag.y, mag.z);
        }
    }


    // Close the file
    for (uint8_t kk=0; kk<compass.get_count(); kk++)
    {
        fclose(f[kk]);
    }

    // Return
    hal.console->printf("Data collected!\n\n");
    return;

}

void AP_Calibration::calibrate_controls(void) {
    hal.console->println("Calibrating ailerons...");

    uint8_t ch_aileron1 = 5;
    uint8_t ch_aileron2 = 6;

    // Will I need to calibrate the accelerometer beforehand?  Probably yes...

    // Set servo update rate
    hal.rcout->set_freq(0xFF, 50);

    // Enable servos
    hal.rcout->enable_ch(ch_aileron1);
    hal.rcout->enable_ch(ch_aileron2);

    // Cycle servos and record results
    for (uint16_t pwm=1100; pwm<1700; pwm++)  // Improve this with detected min and max pwm for each servo
    {
        hal.rcout->write(ch_aileron1, pwm);
        //record PWM, accelerometer, and wingtip board data
    }

}
