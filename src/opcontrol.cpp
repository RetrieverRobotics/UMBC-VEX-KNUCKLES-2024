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
#define LEFT_FRONT_MOTOR_PORT        17
#define LEFT_MIDDLE_FRONT_MOTOR_PORT 18
#define LEFT_MIDDLE_BACK_MOTOR_PORT  7
#define LEFT_BACK_MOTOR_PORT         6

// ports for right drive
#define RIGHT_FRONT_MOTOR_PORT        14
#define RIGHT_MIDDLE_FRONT_MOTOR_PORT 13
#define RIGHT_MIDDLE_BACK_MOTOR_PORT  4
#define RIGHT_BACK_MOTOR_PORT         5

// ports for autocannon
#define LEFT_AUTOCANNON  16
#define RIGHT_AUTOCANNON 15

// Limit Switch (ADI Port H)
#define AUTOCANNON_SWITCH 8

void umbc::Robot::opcontrol() {

    // nice names for controllers (do not edit)
    umbc::Controller* controller_master = this->controller_master;
    umbc::Controller* controller_partner = this->controller_partner;

    // initialize left drive
    pros::Motor drive_left_front_motor = pros::Motor(LEFT_FRONT_MOTOR_PORT, MOTOR_REVERSE);
    pros::Motor drive_left_middle_back_motor = pros::Motor(LEFT_MIDDLE_BACK_MOTOR_PORT, MOTOR_REVERSE);
    pros::Motor drive_left_middle_front_motor = pros::Motor(LEFT_MIDDLE_FRONT_MOTOR_PORT);
	pros::Motor drive_left_back_motor = pros::Motor(LEFT_BACK_MOTOR_PORT);

    pros::MotorGroup drive_left = pros::MotorGroup(vector<pros::Motor>{drive_left_front_motor,
        drive_left_middle_front_motor, drive_left_middle_back_motor, drive_left_back_motor});
    drive_left.set_brake_modes(E_MOTOR_BRAKE_COAST);
    drive_left.set_gearing(E_MOTOR_GEAR_GREEN);
	
    // initialize right drive
    pros::Motor drive_right_front_motor = pros::Motor(RIGHT_FRONT_MOTOR_PORT);
    pros::Motor drive_right_middle_front_motor = pros::Motor(RIGHT_MIDDLE_FRONT_MOTOR_PORT, MOTOR_REVERSE);
    pros::Motor drive_right_middle_back_motor = pros::Motor(RIGHT_MIDDLE_BACK_MOTOR_PORT);
	pros::Motor drive_right_back_motor = pros::Motor(RIGHT_BACK_MOTOR_PORT, MOTOR_REVERSE);
    
    pros::MotorGroup drive_right = pros::MotorGroup(vector<pros::Motor>{drive_right_front_motor,
        drive_right_middle_front_motor, drive_right_middle_back_motor, drive_right_back_motor});
    drive_right.set_brake_modes(E_MOTOR_BRAKE_COAST);
    drive_right.set_gearing(E_MOTOR_GEAR_GREEN);

    // initialize autocannon
    pros::Motor left_autocannon = pros::Motor(LEFT_AUTOCANNON);
    pros::Motor right_autocannon = pros::Motor(RIGHT_AUTOCANNON, MOTOR_REVERSE);

    pros::MotorGroup autocannon = pros::MotorGroup(vector<pros::Motor>{left_autocannon, right_autocannon});
    autocannon.set_brake_modes(E_MOTOR_BRAKE_COAST);
    autocannon.set_gearing(E_MOTOR_GEAR_GREEN);

    // initialize autocannon sensor
    pros::ADIDigitalIn autocannon_switch = pros::ADIDigitalIn(AUTOCANNON_SWITCH);

    // initialize flag for if autocannon is enabled
    bool is_autocannon_enabled = true;

    while(1) {

        // calculate velocity for drive (arcade controls)
        int32_t arcade_y = controller_master->get_analog(E_CONTROLLER_ANALOG_LEFT_Y);
        int32_t arcade_x = controller_master->get_analog(E_CONTROLLER_ANALOG_RIGHT_X);

        int32_t drive_left_velocity = (int32_t)(((double)(arcade_y + arcade_x) / (double)E_CONTROLLER_ANALOG_MAX)
                                        * MOTOR_GREEN_GEAR_MULTIPLIER);

        int32_t drive_right_velocity = (int32_t)(((double)(arcade_y - arcade_x) / (double)E_CONTROLLER_ANALOG_MAX)
                                        * MOTOR_GREEN_GEAR_MULTIPLIER);
        
        // set velocity for drive
        drive_left.move_velocity(drive_left_velocity);
        drive_right.move_velocity(drive_right_velocity);

        // toggles for enabling/disabling autocannon
        bool is_autocannon_primed = autocannon_switch.get_new_press();
        if (controller_master->get_digital(E_CONTROLLER_DIGITAL_R1)) {
            is_autocannon_enabled = true;    
        } else if (is_autocannon_primed) {
            is_autocannon_enabled = false;
        } else if (controller_master->get_digital_new_press(E_CONTROLLER_DIGITAL_R2)) {
            is_autocannon_enabled = !autocannon_switch.get_value();;
        }  else if (controller_master->get_digital_new_press(E_CONTROLLER_DIGITAL_L1)) {
            is_autocannon_enabled = false;
        } else if (controller_master->get_digital_new_press(E_CONTROLLER_DIGITAL_L2)) {
            is_autocannon_enabled = false;
        } 

        // set velocity for autocannon
        if (is_autocannon_enabled) {
            autocannon.move_velocity(MOTOR_GREEN_GEAR_MULTIPLIER);
        } else {
            if (controller_master->get_digital(E_CONTROLLER_DIGITAL_L1)) {
                autocannon.move_velocity(-MOTOR_GREEN_GEAR_MULTIPLIER);
            } else {
                autocannon.brake();
            }
        }

        // required loop delay (do not edit)
        pros::Task::delay(this->opcontrol_delay_ms);
    }
}