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
  multicopter simulator class
*/

#include "SIM_Multicopter.h"
#include <AP_Motors/AP_Motors.h>
#include <math.h>

#include <stdio.h>

#define DTR 0.01745329f
using namespace SITL;

MultiCopter::MultiCopter(const char *home_str, const char *frame_str) :
    Aircraft(home_str, frame_str),
    frame(NULL)
{
    frame = Frame::find_frame(frame_str);
    if (frame == NULL) {
        printf("Frame '%s' not found", frame_str);
        exit(1);
    }
    if (strstr(frame_str, "-fast")) {
        frame->init(1.5, 0.5, 85, 4*radians(360));
    } else {
        frame->init(1.5, 0.51, 15, 4*radians(360));
    }
    frame_height = 0.1;
    ground_behavior = GROUND_BEHAVIOR_NO_MOVEMENT;

    // Initialise I_vector
    mass = 1.5;
    I_vector.x = 1; I_vector.y = 1; I_vector.z = 1;
}

// calculate rotational and linear accelerations
void MultiCopter::calculate_forces(const struct sitl_input &input, Vector3f &rot_accel, Vector3f &body_accel)
{
    frame->calculate_forces(*this, input, rot_accel, body_accel);
}
    
/*
  update the multicopter simulation by one time step
 */
void MultiCopter::update(const struct sitl_input &input)
{
    // get wind vector setup
    update_wind(input);

    Vector3f rot_accel;

    //========== EDIT FROM HERE
    // Initialise variables to zero
    accel_body.x = 0; accel_body.y = 0; accel_body.z = 0;
    rot_accel.x  = 0; rot_accel.y  = 0; rot_accel.z  = 0;
    M_prop.x = 0; M_prop.y = 0; M_prop.z = 0;
    F_prop.x = 0; F_prop.y = 0; F_prop.z = 0;
    M_aero.x = 0; M_aero.y = 0; M_aero.z = 0;
    F_aero.x = 0; F_aero.y = 0; F_aero.z = 0;

    // update rpms
    prop_dynamics(input);

    if (0)
    {
        // calculate new prop forces (new way)
        prop_forces();

        // calculate vehicle rates
        rot_accel.x = M_prop.x / I_vector.x;
        rot_accel.y = M_prop.y / I_vector.y;
        rot_accel.z = M_prop.z / I_vector.z;
        accel_body = F_prop / mass;

        // PS: This doesn't work...

    } else {
        // calculate prop forces (old way)
        calculate_forces(input, rot_accel, accel_body);
    }

    // calculate new aero forces
    aero_forces();

    // calculate vehicle rates
    rot_accel.x += (M_aero.x / I_vector.x);
    rot_accel.y += (M_aero.y / I_vector.y);
    rot_accel.z += (M_aero.z / I_vector.z);
    accel_body  += (F_aero   / mass      );


    //========== TO HERE
    // at this point, accel_body and rot_accel should be correctly calculated

    // Integrate vehicle state
    update_dynamics(rot_accel);

    // update lat/lon/altitude
    update_position();

    // update magnetic field
    update_mag_field_bf();
}

void MultiCopter::aero_forces(void)
{
    // Calculate aero forces
    Vector3f V = get_velocity_air_bf();
    float u = V.x;
    float v = V.y;
    float w = V.z;

    float alfa = atan2f(u,-w);

    // debugging loop
    if (counter % 20 == 0) {
        printf("alfa = %f deg\n",alfa/DTR);
        counter = 0;
    } else {
        counter++;
    }
    return;
}

void MultiCopter::prop_dynamics(const struct sitl_input &input)
{
   // Calculates the new prop RPMs
    rpm1 = input.servos[0]*0.75f + rpm1*0.25f;
    rpm2 = input.servos[1]*0.75f + rpm2*0.25f;
    rpm3 = input.servos[2]*0.75f + rpm3*0.25f;
    rpm4 = input.servos[3]*0.75f + rpm4*0.25f;

    return;
}

void MultiCopter::prop_forces(void)
{
    // Calculate prop forces
    // rpm1, rpm2, rpm3, rpm4

    // prop1
    F_prop.x +=  0;
    F_prop.y +=  0;
    F_prop.z += -calc_thrust(rpm1);

    M_prop.x += -0.7071*calc_thrust(rpm1);
    M_prop.y +=  0.7071*calc_thrust(rpm1);
    M_prop.z +=  calc_torque(rpm2);

    // prop2
    F_prop.x += 0; F_prop.y += 0;
    F_prop.z += -calc_thrust(rpm2);

    M_prop.x +=  0.7071*calc_thrust(rpm2);
    M_prop.y += -0.7071*calc_thrust(rpm2);
    M_prop.z += -calc_torque(rpm2);

    // prop3
    F_prop.x +=  0;
    F_prop.y +=  0;
    F_prop.z += -calc_thrust(rpm3);

    M_prop.x +=  0.7071*calc_thrust(rpm3);
    M_prop.y +=  0.7071*calc_thrust(rpm3);
    M_prop.z +=  calc_torque(rpm2);

    // prop4
    F_prop.x +=  0;
    F_prop.y +=  0;
    F_prop.z += -calc_thrust(rpm4);

    M_prop.x += -0.7071*calc_thrust(rpm4);
    M_prop.y += -0.7071*calc_thrust(rpm4);
    M_prop.z += -calc_torque(rpm2);

}

float MultiCopter::calc_thrust(float rpm)
{
    double thrust = CT*rho*D*D*D*D*rpm*rpm;

    return (float)(thrust);
}

float MultiCopter::calc_torque(float rpm)
{
    double torque = CQ*rho*D*D*D*D*D*rpm*rpm;

    return (float)(torque);
}



