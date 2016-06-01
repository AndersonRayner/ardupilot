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
	// IMU 1
	AP_Param::set_object_value(&ins, ins.var_info, "_ACC_CAL1_X_X", 1);
	AP_Param::set_object_value(&ins, ins.var_info, "_ACC_CAL1_X_Y", 0);
	AP_Param::set_object_value(&ins, ins.var_info, "_ACC_CAL1_X_Z", 0);

	AP_Param::set_object_value(&ins, ins.var_info, "_ACC_CAL1_Y_X", 0);
	AP_Param::set_object_value(&ins, ins.var_info, "_ACC_CAL1_Y_Y", 1);
	AP_Param::set_object_value(&ins, ins.var_info, "_ACC_CAL1_Y_Z", 0);

	AP_Param::set_object_value(&ins, ins.var_info, "_ACC_CAL1_Z_X", 0);
	AP_Param::set_object_value(&ins, ins.var_info, "_ACC_CAL1_Z_Y", 0);
	AP_Param::set_object_value(&ins, ins.var_info, "_ACC_CAL1_Z_Z", 1);

	AP_Param::set_object_value(&ins, ins.var_info, "_ACC_ACCOFFS_X", 0);
	AP_Param::set_object_value(&ins, ins.var_info, "_ACC_ACCOFFS_Y", 0);
	AP_Param::set_object_value(&ins, ins.var_info, "_ACC_ACCOFFS_Z", 0);

	// IMU 2
	AP_Param::set_object_value(&ins, ins.var_info, "_ACC_CAL2_X_X", 1);
	AP_Param::set_object_value(&ins, ins.var_info, "_ACC_CAL2_X_Y", 0);
	AP_Param::set_object_value(&ins, ins.var_info, "_ACC_CAL2_X_Z", 0);

	AP_Param::set_object_value(&ins, ins.var_info, "_ACC_CAL2_Y_X", 0);
	AP_Param::set_object_value(&ins, ins.var_info, "_ACC_CAL2_Y_Y", 1);
	AP_Param::set_object_value(&ins, ins.var_info, "_ACC_CAL2_Y_Z", 0);

	AP_Param::set_object_value(&ins, ins.var_info, "_ACC_CAL2_Z_X", 0);
	AP_Param::set_object_value(&ins, ins.var_info, "_ACC_CAL2_Z_Y", 0);
	AP_Param::set_object_value(&ins, ins.var_info, "_ACC_CAL2_Z_Z", 1);

	AP_Param::set_object_value(&ins, ins.var_info, "_ACC_ACC2OFFS_X", 0);
	AP_Param::set_object_value(&ins, ins.var_info, "_ACC_ACC2OFFS_Y", 0);
	AP_Param::set_object_value(&ins, ins.var_info, "_ACC_ACC2OFFS_Z", 0);

	// IMU 3
	AP_Param::set_object_value(&ins, ins.var_info, "_ACC_CAL3_X_X", 1);
	AP_Param::set_object_value(&ins, ins.var_info, "_ACC_CAL3_X_Y", 0);
	AP_Param::set_object_value(&ins, ins.var_info, "_ACC_CAL3_X_Z", 0);

	AP_Param::set_object_value(&ins, ins.var_info, "_ACC_CAL3_Y_X", 0);
	AP_Param::set_object_value(&ins, ins.var_info, "_ACC_CAL3_Y_Y", 1);
	AP_Param::set_object_value(&ins, ins.var_info, "_ACC_CAL3_Y_Z", 0);

	AP_Param::set_object_value(&ins, ins.var_info, "_ACC_CAL3_Z_X", 0);
	AP_Param::set_object_value(&ins, ins.var_info, "_ACC_CAL3_Z_Y", 0);
	AP_Param::set_object_value(&ins, ins.var_info, "_ACC_CAL3_Z_Z", 1);

	AP_Param::set_object_value(&ins, ins.var_info, "_ACC_ACC3OFFS_X", 0);
	AP_Param::set_object_value(&ins, ins.var_info, "_ACC_ACC3OFFS_Y", 0);
	AP_Param::set_object_value(&ins, ins.var_info, "_ACC_ACC3OFFS_Z", 0);

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
		for (uint16_t ii = 0; ii<500; ii++)
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

	// Reset all of the compass parameters
	for (int ii = 0; ii<compass.get_count(); ii++)
	{
		// Accelerometers
		// AP_Param::set_object_value(&ins, ins.var_info, "_PIN", 65);
	}

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

		fprintf(f[kk],"Magnetometer Calibration File\n");
		fprintf(f[kk],"=============================\n");
	}

	// Wait for user to confirm to take reading
	hal.console->printf("Rotate the vehicle around at least three axes\n\n");
	hal.console->printf("Press < return > to start calibration\n");

	while( !hal.console->available() ) {
		hal.scheduler->delay(20);
	}


	uint32_t last_update=0;

	// Collect the data
	for (int ii = 0; ii<500; ii++)
	{
		compass.accumulate();
		compass.read();

		// Wait for new data
		while (last_update>=compass.last_update_usec(0))
		{
			hal.scheduler->delay(5);
			compass.read();
		}
		last_update = compass.last_update_usec(0);

		// Record data
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
	uint8_t ch[2];
	Vector3f accel;

	ch[0] = 5;  // right aileron channel
	ch[1] = 6;  // left  aileron channel

	hal.console->print("Calibrating ailerons...\n");
	ins.init(100);
	wingtip.init();

	hal.console->print("\n");

	// Will I need to calibrate the accelerometer beforehand?  Probably yes...

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
		for (uint16_t pwm=1100; pwm<1700; pwm++)  // Improve this with detected min and max pwm for each servo
		{
			// Write new servo value
			hal.rcout->write(ch[ii], pwm);

			// read samples from ins
			ins.wait_for_sample();
			ins.update();
			while (!ins.is_still())  // This might only be on the primary INS, will have to check
			{
				ins.wait_for_sample();
				ins.update();
			}
			accel = ins.get_accel(0);

			//record PWM, accelerometer, and wingtip board data
			fprintf(f,"%u,%f,%f,%f,%u\n",pwm,accel.x,accel.y,accel.z,wingtip.get_de_raw(ii));  // will have to make sure this is the right way around
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
