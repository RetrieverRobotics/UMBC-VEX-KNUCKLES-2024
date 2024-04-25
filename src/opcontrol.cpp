/**
 * \file opcontrol.cpp
 *
 * Contains user implemented opcontrol. User must use the
 * parameters to the opcontrol function when referencing
 * the master V5 controller or partner V5 controller.
 */

#include "api.h"
#include "umbc.h"

#include <cstdint>
#include <vector>

using namespace pros;
using namespace umbc;
using namespace std;

#define MOTOR_RED_GEAR_MULTIPLIER    100
#define MOTOR_GREEN_GEAR_MULTIPLIER  200
#define MOTOR_BLUE_GEAR_MULTIPLIER   600
#define MOTOR_REVERSE                true

// ports for left drive
#define LEFT_FRONT_MOTOR_PORT 3
#define LEFT_MIDDLE_FRONT_MOTOR_PORT 5
#define LEFT_MIDDLE_BACK_MOTOR_PORT 4 // CHANGE PORTS AROUND IF NEEDED
#define LEFT_BACK_MOTOR_PORT 11

// ports for right drive
#define RIGHT_FRONT_MOTOR_PORT 1
#define RIGHT_MIDDLE_FRONT_MOTOR_PORT 2
#define RIGHT_MIDDLE_BACK_MOTOR_PORT 7 // CHANGE PORTS AROUND IF NEEDED
#define RIGHT_BACK_MOTOR_PORT  12

// ports for clamps
#define LEFT_CLAMP 13
#define RIGHT_CLAMP 14 // CHANGE PORTS AROUND IF NEEDED


// Limit Switch
#define LIMIT_SWITCH 1 //Port A

//#define 


void umbc::Robot::opcontrol() {

    // nice names for controllers (do not edit)
    umbc::Controller* controller_master = this->controller_master;
    umbc::Controller* controller_partner = this->controller_partner;

    // initialize left drive
    pros::Motor drive_left_front_motor = pros::Motor(LEFT_FRONT_MOTOR_PORT);
    pros::Motor drive_left_middle_back_motor = pros::Motor(LEFT_MIDDLE_BACK_MOTOR_PORT); //MOTOR_REVERSE if needed
    pros::Motor drive_left_middle_front_motor = pros::Motor(LEFT_MIDDLE_FRONT_MOTOR_PORT); //MOTOR_REVERSE if needed
	pros::Motor drive_left_back_motor = pros::Motor(LEFT_BACK_MOTOR_PORT);

    pros::MotorGroup drive_left = pros::MotorGroup(vector<pros::Motor>{drive_left_front_motor,
        drive_left_middle_front_motor, drive_left_middle_back_motor, drive_left_back_motor});
    drive_left.set_brake_modes(E_MOTOR_BRAKE_COAST);
    drive_left.set_gearing(E_MOTOR_GEAR_RED);
	
    // initialize right drive
    pros::Motor drive_right_front_motor = pros::Motor(RIGHT_FRONT_MOTOR_PORT);
    pros::Motor drive_right_middle_front_motor = pros::Motor(RIGHT_MIDDLE_FRONT_MOTOR_PORT); //MOTOR_REVERSE if needed
    pros::Motor drive_right_middle_back_motor = pros::Motor(RIGHT_MIDDLE_BACK_MOTOR_PORT); //MOTOR_REVERSE if needed
	pros::Motor drive_right_back_motor = pros::Motor(RIGHT_BACK_MOTOR_PORT);
    
    pros::MotorGroup drive_right = pros::MotorGroup(vector<pros::Motor>{drive_right_front_motor,
        drive_right_middle_front_motor, drive_right_middle_back_motor, drive_right_back_motor});
    drive_right.set_brake_modes(E_MOTOR_BRAKE_COAST);
    drive_right.set_gearing(E_MOTOR_GEAR_RED);

    // initialize clamps (ik not good name to call them, change later)
    pros::Motor left_clamp = pros::Motor(LEFT_CLAMP);
    pros::Motor right_clamp = pros::Motor(RIGHT_CLAMP, MOTOR_REVERSE);

    pros::MotorGroup clamps = pros::MotorGroup(vector<pros::Motor>{right_clamp,left_clamp});

    // initalize limit switch
    pinMode(LIMIT_SWITCH, INPUT);

    while(1) {

        // set velocity for drive (arcade controls)
        int32_t arcade_y = controller_master->get_analog(E_CONTROLLER_ANALOG_LEFT_Y);
        int32_t arcade_x = controller_master->get_analog(E_CONTROLLER_ANALOG_RIGHT_X);

        int32_t drive_left_velocity = (int32_t)(((double)(arcade_x - arcade_y) / (double)E_CONTROLLER_ANALOG_MAX)
                                        * MOTOR_RED_GEAR_MULTIPLIER);

        int32_t drive_right_velocity = (int32_t)(((double)(arcade_x + arcade_y) / (double)E_CONTROLLER_ANALOG_MAX)
                                        * MOTOR_RED_GEAR_MULTIPLIER);                                

        drive_left.move_velocity(drive_left_velocity);
        drive_right.move_velocity(drive_right_velocity);

        // manually control clamp arms
        if (controller_master->get_digital(E_CONTROLLER_DIGITAL_X)) {
            clamps.move_velocity(MOTOR_RED_GEAR_MULTIPLIER);
        } else if (controller_master->get_digital(E_CONTROLLER_DIGITAL_Y)) {
            clamps.move_velocity(-MOTOR_RED_GEAR_MULTIPLIER);
        } else {
            clamps.brake();
        }


//have the motors for firiing mech rotatin unless switch is press
        //UNLESS if fire button is fire button is

    // Limit Switch Stuff
    if ((digitalRead(LIMIT_SWITCH) == HIGH) && (get_digital(E_CONTROLLER_DIGITAL_R1) != true)){
        clamps.move_velocity(0) 
    } else if ((digitalRead(LIMIT_SWITCH) == HIGH) && (get_digital(E_CONTROLLER_DIGITAL_R1) == true)) { 
        clamps.move_velocity(MOTOR_RED_GEAR_MULTIPLIER)
    } else { 
        clamps.move_velocity(MOTOR_RED_GEAR_MULTIPLIER)
    }

        // required loop delay (do not edit)
        pros::Task::delay(this->opcontrol_delay_ms);
    }
}