#include "main.h"
#include "mylib/chassis.hpp"

// 1. HARDWARE & PID SETUP
// Initialize the config with placeholder motor ports and null sensor ports
mylib::ChassisConfig config = {
    {1, 2, 3},       // left_motors
    {-4, -5, -6},    // right_motors
    0,               // imu_port (0 = not installed)
    0,               // vertical_wheel_port
    0,               // horizontal_wheel_port
    0                // distance_sensor_port
};

// Initialize PID constants with basic values
mylib::PIDConstants drive_pid = {0.5f, 0.0f, 0.1f};
mylib::PIDConstants turn_pid = {0.4f, 0.0f, 0.05f};

// 2. CHASSIS INSTANCE (Correct 3-argument call)
mylib::Chassis chassis(config, drive_pid, turn_pid);

void initialize() {
    // Basic screen output to verify code is running
    pros::lcd::initialize();
    pros::lcd::set_text(1, "Vex U System: Ready");
    
    // Set baseline drive feel
    chassis.set_joystick_curves(0.3f, 5);
}

void disabled() {}

void competition_initialize() {}

void autonomous() {}

void opcontrol() {
    pros::Controller master(pros::E_CONTROLLER_MASTER);

    while (true) {
        // Standard Arcade Drive
        int throttle = master.get_analog(ANALOG_LEFT_Y);
        int turn = master.get_analog(ANALOG_RIGHT_X);

        chassis.arcade(throttle, turn);

        pros::delay(10); // Required task delay
    }
}