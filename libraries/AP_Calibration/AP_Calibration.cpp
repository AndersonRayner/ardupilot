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

#define AP_COMPASS_TYPE_UNKNOWN         0x00
#define AP_COMPASS_TYPE_HIL             0x01
#define AP_COMPASS_TYPE_HMC5843         0x02
#define AP_COMPASS_TYPE_HMC5883L        0x03
#define AP_COMPASS_TYPE_PX4             0x04
#define AP_COMPASS_TYPE_VRBRAIN         0x05
#define AP_COMPASS_TYPE_AK8963_MPU9250  0x06
#define AP_COMPASS_TYPE_AK8963_I2C      0x07
#define AP_COMPASS_TYPE_LSM303D         0x08
#define AP_COMPASS_TYPE_LSM9DS1         0x09

extern const AP_HAL::HAL& hal;

static AP_InertialSensor ins;
static Compass compass;
static AP_Wingtip wingtip;
static AP_BattMonitor battery_mon;

AP_Calibration::AP_Calibration(void)
{
	// do nothing
}

void AP_Calibration::calibrate_IMU(void) {
	Vector3f accel;
	Vector3f gyro;

	hal.console->println("Calibrating inertial sensors...");
	ins.init(500);

	// display number of detected accels/gyros
	hal.console->printf("    Number of detected accels : %u\n", ins.get_accel_count());
	hal.console->printf("    Number of detected gyros  : %u\n\n", ins.get_gyro_count());

	// Reset all of the IMU parameters
	ins.reset_IMU_calibration();

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
			sprintf(str_base, "%s", "Z_down.acc");
			break;

		case 1 :
			hal.console->printf("Z_up\n");
			sprintf(str_base, "%s", "Z_up.acc");
			break;

		case 2 :
			hal.console->printf("Y_down\n");
			sprintf(str_base, "%s", "Y_down.acc");
			break;

		case 3 :
			hal.console->printf("Y_up\n");
			sprintf(str_base, "%s", "Y_up.acc");
			break;

		case 4 :
			hal.console->printf("X_down\n");
			sprintf(str_base, "%s", "X_down.acc");
			break;

		case 5 :
			hal.console->printf("X_up\n");
			sprintf(str_base, "%s", "X_up.acc");
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

		// Open a file for each sensor
		// Open the required number of files
		FILE *f[3];
		for (int kk = 0; kk<ins.get_accel_count(); kk++)
		{
			char str[40];
			sprintf(str, "%s%d-%s",CALIBRATION_DIR,kk,str_base);

			f[kk] = fopen(str,"w");

			fprintf(f[kk],"Accelerometer Calibration File\n");
			fprintf(f[kk],"=======================\n");
		}


		// Write data points
		for (uint16_t ii = 0; ii<1000; ii++)
		{
			// wait until we have a sample
			ins.wait_for_sample();

			// read samples from ins
			ins.update();
			for (uint8_t kk = 0; kk<ins.get_accel_count(); kk++)
			{
				accel = ins.get_accel(kk);
				fprintf(f[kk],"%f,%f,%f\n",accel.x, accel.y, accel.z);
			}
		}

		// Close the file
		for (uint8_t kk=0; kk<ins.get_accel_count(); kk++)
		{
			fclose(f[kk]);
			hal.console->printf("Done file %d-%d!\n",jj,kk);
		}
		hal.console->printf("\n");
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
	for (uint8_t ii=0; ii<compass.get_count(); ii++) {
	    hal.console->printf("  %u - Compass ID %d, ",ii,compass.get_compass_id(ii));
	    // Print the name of the compass
	    switch (compass.get_compass_id(ii)) {
	    case AP_COMPASS_TYPE_UNKNOWN :
	        hal.console->printf("Unknown");
	        break;
	    case AP_COMPASS_TYPE_HIL :
	        hal.console->printf("HIL");
            break;
	    case AP_COMPASS_TYPE_HMC5843 :
	        hal.console->printf("HMC5843");
            break;
	    case AP_COMPASS_TYPE_HMC5883L :
	        hal.console->printf("HMC5883L");
            break;
	    case AP_COMPASS_TYPE_PX4 :
	        hal.console->printf("PX4");
            break;
	    case AP_COMPASS_TYPE_VRBRAIN :
	        hal.console->printf("VRBRAIN");
            break;
	    case AP_COMPASS_TYPE_AK8963_MPU9250 :
	        hal.console->printf("MPU9250");
            break;
	    case AP_COMPASS_TYPE_AK8963_I2C :
	        hal.console->printf("AK8963_I2C");
            break;
	    case AP_COMPASS_TYPE_LSM303D :
	        hal.console->printf("LSM303D");
            break;
	    case AP_COMPASS_TYPE_LSM9DS1 :
	        hal.console->printf("LSM9DS1");
            break;
	    default :
	        hal.console->printf("Unknown ID...");
	        break;
	    }
	    hal.console->printf("\n");
	}

	// Reset all of the compass parameters
	compass.reset_compass_calibration();

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
		char str[40];

		sprintf(str, "%s%d-%s",CALIBRATION_DIR,kk,"compass.mag");

		f[kk] = fopen(str,"w");

		fprintf(f[kk],"Magnetometer Calibration File - devID %d\n",compass.get_compass_id(kk));
		fprintf(f[kk],"========================================\n");
	}

	// Wait for user to confirm to take reading
	hal.console->printf("Rotate the vehicle around at least three axes\n\n");
	hal.console->printf("Press < return > to start calibration\n");

	while( !hal.console->available() ) {
		hal.scheduler->delay(20);
	}

	// Clear input buffer so can stop collecting on key press
    while (hal.console->available()) {
        hal.console->read();
    }

    hal.console->printf("Press < return > to stop calibration\n");

	//uint32_t last_update = 0;
	bool continue_collecting = 1;

	// Collect the data
	while (continue_collecting)
	{
		compass.accumulate();
		compass.read();

		    // Record data
		    for (uint8_t kk=0; kk<compass.get_count(); kk++)
		    {
		        const Vector3f &mag = compass.get_field(kk);
		        fprintf(f[kk],"%f,%f,%f\n",mag.x, mag.y, mag.z);
		    }

		hal.scheduler->delay(20);

		if (hal.console->available())
		{
		    continue_collecting = 0;
		}

	}

	// Clear input buffer
    while (hal.console->available()) {
        hal.console->read();
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
    // Make sure to be using WingTip/wingtip_calibration script on the ardu
	uint8_t ch[2];
	Vector3f accel;

	ch[0] = 4;  // right aileron channel
	ch[1] = 5;  // left  aileron channel

	hal.console->print("Calibrating ailerons...\n");
	wingtip.init();

	hal.console->print("\n");

	// Force wingtip board type 1, won't work for a type 2 wingtip board
	AP_Param::set_object_value(&wingtip, wingtip.var_info, "_TYPE", 1);

	// Loop through each control surface
	uint8_t ii;
	for (ii=0; ii<2; ii++)
	{
		if (ii == 0)
		{
			hal.console->printf("\n=== RIGHT AILERON ===\n");
		} else {
			hal.console->printf("\n=== LEFT AILERON ===\n");
		}

		// Clear any user input buffer
		while( hal.console->available() ) {
			hal.console->read();
		}

		// Wait for user to confirm to start reading
		hal.console->printf("Attach accelerometer to control surface\n\n");
		hal.console->printf("Press < return > to start\n");
		while( !hal.console->available() ) {
			hal.scheduler->delay(20);
		}
		hal.console->printf("Starting recording\n");

		// Set servo update rate
		hal.rcout->set_freq(0xFF, 50);

		// Enable servo
		hal.rcout->enable_ch(ch[ii]);

		// Open file
		char str[40];
		if (ii == 0)
		{
			sprintf(str, "%s%s",CALIBRATION_DIR,"right.ail");
		} else {
			sprintf(str, "%s%s",CALIBRATION_DIR,"left.ail");
		}

		FILE *f = fopen(str,"w");

		fprintf(f,"Aileron Calibration File\n");
		fprintf(f,"=============================\n");

		// Cycle servos and record results
		for (uint16_t pwm=1100; pwm<1700; pwm=pwm+25)  // Improve this with detected min and max pwm for each servo
		{
			// Write new servo value
			hal.rcout->write(ch[ii], pwm);

			// Delay so that the control surface can get to where it needs to be
			hal.scheduler->delay(50);  // This will need some tweaking

			// Update wingtip board
			wingtip.update();

			//record PWM, accelerometer, and wingtip board data
			// accelerometer data is provided from the wingtip board on RPM1 and RPM2
			fprintf(f,"%u,%u,%u,%u\n",pwm,wingtip.get_rpm(0),wingtip.get_rpm(1),wingtip.get_de_raw(ii));  // will have to make sure this is the right way around
		}

		// Close file
		hal.console->printf("Data collected!\n\n");
		fclose(f);

		// Disable servo
		hal.rcout->disable_ch(ch[ii]);
	}

	// Return
	return;


}

void AP_Calibration::calibrate_RPM(void) {
	hal.console->print("Calibrating PWM to RPM...\n");

	// Force wingtip board type 2 (for now)
	AP_Param::set_object_value(&wingtip, wingtip.var_info, "_TYPE", 2);

	// Initialise wingtip sensor
	wingtip.init();

	hal.console->print("\n");

	// Wait for wingtip sensor to become healthy


	// Set servo update rate
	hal.rcout->set_freq(0xFF, 50);

	// Enable motors
	hal.rcout->enable_ch(0);
	hal.rcout->enable_ch(1);
	hal.rcout->enable_ch(2);
	hal.rcout->enable_ch(3);

	// Write idle PWM
	hal.rcout->write(0, 900);
	hal.rcout->write(1, 900);
	hal.rcout->write(2, 900);
	hal.rcout->write(3, 900);

	// Open file
	char str[40];
    sprintf(str, "%s%s",CALIBRATION_DIR,"motors.mot");
    FILE *f = fopen(str,"w");

    fprintf(f,"Motor Calibration File\n");
    fprintf(f,"=============================\n");

    // Clear any user input buffer
    while( hal.console->available() ) {
    	hal.console->read();
    }

    // Wait for user to confirm to start reading
    hal.console->printf("Press < return > to start\n");
    while( !hal.console->available() ) {
    	hal.scheduler->delay(20);
    }
    hal.console->printf("Starting recording\n");

    // Cycle motors and record results
    for (uint16_t pwm=1050; pwm<2001; pwm=pwm+50)  // Improve this with detected min and max pwm for each servo
    {
    	// Inform of PWM being tested
    	hal.console->printf("Testing PWM %4u\n",pwm);

    	// Write new servo value
    	hal.rcout->write(0, pwm);
    	hal.rcout->write(1, pwm);
    	hal.rcout->write(2, pwm);
    	hal.rcout->write(3, pwm);

    	for (uint16_t counter=0; counter<200; counter++)
    	{
    		// delay a bit
    	    uint64_t time_us_prev = AP_HAL::micros64();
    	    while((AP_HAL::micros64()-time_us_prev)<10000)
    	    {
    	    	// wait
    	    }

    		// Update wingtip boards
    		wingtip.update();

    		//record PWM, accelerometer, and wingtip board data
    		fprintf(f,"%llu,%u,%u,%u,%u,%u\n",
    				AP_HAL::micros64(),pwm,wingtip.get_rpm(0),wingtip.get_rpm(1),wingtip.get_rpm(2),wingtip.get_rpm(3));
    	}
    }

    // Record turning off the motors
    hal.rcout->write(0, 900);
    hal.rcout->write(1, 900);
    hal.rcout->write(2, 900);
    hal.rcout->write(3, 900);

    for (uint16_t counter=0; counter<600; counter++)
    {
        // delay a bit
        uint64_t time_us_prev = AP_HAL::micros64();
        while((AP_HAL::micros64()-time_us_prev)<10000)
        {
            // wait
        }

        // Update wingtip boards
        wingtip.update();

        //record PWM, accelerometer, and wingtip board data
        fprintf(f,"%llu,%u,%u,%u,%u,%u\n",
                AP_HAL::micros64(),900,wingtip.get_rpm(0),wingtip.get_rpm(1),wingtip.get_rpm(2),wingtip.get_rpm(3));
    }


    // Close file
    hal.console->printf("Data collected!\n\n");
    fclose(f);

    // Disable motors
    hal.rcout->disable_ch(0);
    hal.rcout->disable_ch(1);
    hal.rcout->disable_ch(2);
    hal.rcout->disable_ch(3);

}

void AP_Calibration::calibrate_motcompass(void) {
    hal.console->println("Calibrating compass offset due to motors...");

    // Initalise the compasses
    compass.init();
    hal.console->printf("Number of detected magnetometers : %u\n", compass.get_count());
    compass.reset_compass_calibration();

    // Initialise wingtip sensor
    wingtip.init();

    // Initialise battery monitor
    battery_mon.set_monitoring(0, AP_BattMonitor::BattMonitor_TYPE_ANALOG_VOLTAGE_AND_CURRENT);
    AP_Param::set_object_value(&battery_mon, battery_mon.var_info, "_VOLT_PIN", 2);
    AP_Param::set_object_value(&battery_mon, battery_mon.var_info, "_CURR_PIN", 0);
    AP_Param::set_object_value(&battery_mon, battery_mon.var_info, "_VOLT_MULT", 1);
    AP_Param::set_object_value(&battery_mon, battery_mon.var_info, "_AMP_PERVOLT", 33);
    AP_Param::set_object_value(&battery_mon, battery_mon.var_info, "_AMP_OFFSET",0);
    battery_mon.init();

    // Initalise motors
    // Set servo update rate
    hal.rcout->set_freq(0xFF, 50);

    // Enable motors
    hal.rcout->enable_ch(0);
    hal.rcout->enable_ch(1);
    hal.rcout->enable_ch(2);
    hal.rcout->enable_ch(3);

    // Write idle PWM
    hal.rcout->write(0, 900);
    hal.rcout->write(1, 900);
    hal.rcout->write(2, 900);
    hal.rcout->write(3, 900);

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
        char str[40];

        sprintf(str, "%s%d-%s",CALIBRATION_DIR,kk,"compass.magoffs");

        f[kk] = fopen(str,"w");

        fprintf(f[kk],"Magnetometer/Motor Current Calibration File\n");
        fprintf(f[kk],"===========================================\n");
    }

    // Wait for user to confirm to take reading
    hal.console->printf("The motors will spin up automatically.  Be careful!\n\n");
    hal.console->printf("Press < return > to start calibration\n");

    while( !hal.console->available() ) {
        hal.scheduler->delay(20);
    }

    // Clear input buffer so can stop collecting on key press
    while (hal.console->available()) {
        hal.console->read();
    }

    hal.console->printf("Press < return > to stop calibration early\n");

    bool continue_collecting = 1;

    uint16_t pwm = 1000;
    uint8_t  counter = 0;

    // Collect the data
    while (continue_collecting)
    {
        counter++;

        // Read battery monitor
        battery_mon.read();

        // Read wingtip sensor
        wingtip.update();

        // Read compass
        compass.accumulate();
        compass.read();

        // Record data
        for (uint8_t kk=0; kk<compass.get_count(); kk++)
        {
            const Vector3f &mag = compass.get_field(kk);
            fprintf(f[kk],"%f,%f,%f,%f\n",mag.x, mag.y, mag.z, battery_mon.current_amps());
        }

        // Set new PWM (if required)
        if (counter>50) {
            pwm = pwm + 25;
            counter = 0;

            hal.console->printf("Testing PWM %d\n",pwm);
            hal.rcout->write(0, pwm);
            hal.rcout->write(1, pwm);
            hal.rcout->write(2, pwm);
            hal.rcout->write(3, pwm);
        }

        hal.scheduler->delay(20);

        // Stop collecting if key pressed or PWM gets high enough
        if (hal.console->available() || pwm>1300)
        {
            continue_collecting = 0;
        }
    }

    // Write idle PWM
    hal.rcout->write(0, 900);
    hal.rcout->write(1, 900);
    hal.rcout->write(2, 900);
    hal.rcout->write(3, 900);

    // Disable motors
    hal.rcout->disable_ch(0);
    hal.rcout->disable_ch(1);
    hal.rcout->disable_ch(2);
    hal.rcout->disable_ch(3);


    // Clear input buffer
    while (hal.console->available()) {
        hal.console->read();
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
