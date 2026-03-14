#include "main.h"
#include "mylib/chassis.hpp"

// 1. HARDWARE & PID SETUP
mylib::ChassisConfig config = {
    {1, 2, 3},       // left_motors
    {-4, -5, -6},    // right_motors
    0,               // imu_port (0 = not installed)
    0,               // vertical_wheel_port
    0,               // horizontal_wheel_port
    0                // distance_sensor_port
};

mylib::PIDConstants drive_pid = {0.5f, 0.0f, 0.1f};
mylib::PIDConstants turn_pid = {0.4f, 0.0f, 0.05f};

// 2. CHASSIS INSTANCE 
mylib::Chassis chassis(config, drive_pid, turn_pid);

void initialize() {
    pros::lcd::initialize();
    pros::lcd::set_text(1, "Vex U System: Ready");
    
    chassis.set_joystick_curves(0.3f, 5);
}

void disabled() {}

void competition_initialize() {}

void autonomous() {}

void opcontrol() {
    pros::Controller master(pros::E_CONTROLLER_MASTER);
    
    // Timer to prevent spamming the controller screen
    int loop_counter = 0; 

    while (true) {
        // --- 1. Driving Logic ---
        int throttle = master.get_analog(ANALOG_LEFT_Y);
        int turn = master.get_analog(ANALOG_RIGHT_X);
        chassis.arcade(throttle, turn);

        // --- 2. Odometry Tracking Logic ---
        mylib::Pose current_pose = chassis.get_pose();

        // Print to the Brain Screen every 10ms
        pros::lcd::print(2, "X: %.1f mm", current_pose.x);
        pros::lcd::print(3, "Y: %.1f mm", current_pose.y);
        pros::lcd::print(4, "Theta: %.2f rad", current_pose.theta);

        // Print to the Controller Screen every 100ms
        // (loop_counter % 10 == 0) means it runs exactly once every 10 loops
        if (loop_counter % 10 == 0) {
            master.print(0, 0, "X:%.0f Y:%.0f   ", current_pose.x, current_pose.y);
        }

        loop_counter++;
        pros::delay(10);
    }
}