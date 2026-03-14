#include "mylib/chassis.hpp" // Adjust this path if your folder structure is different

namespace mylib {

    // --- CONSTRUCTOR IMPLEMENTATION ---
    Chassis::Chassis(const ChassisConfig& config, 
                     const PIDConstants& drive_consts,
                     const PIDConstants& turn_consts)
        // 1. The Initializer List (Fast Initialization)
        : left_motors(config.left_motors),
          right_motors(config.right_motors),
          drive_pid(drive_consts),
          turn_pid(turn_consts) 
    {
        // 2. Dynamic Sensor Allocation (Safe Initialization)
        // Only build the sensor in memory if a valid port (> 0) was provided
        
        if (config.imu_port != 0) {
            imu = std::make_unique<pros::Imu>(config.imu_port);
        }
        
        if (config.vertical_wheel_port != 0) {
            vertical_encoder = std::make_unique<pros::Rotation>(config.vertical_wheel_port);
        }
        
        if (config.horizontal_wheel_port != 0) {
            horizontal_encoder = std::make_unique<pros::Rotation>(config.horizontal_wheel_port);
        }
        
        if (config.distance_sensor_port != 0) {
            distance_sensor = std::make_unique<pros::Distance>(config.distance_sensor_port);
        }
    }

    // --- DRIVER CONTROL IMPLEMENTATION ---
        void Chassis::arcade(int throttle, int turn) {
            // 1. The Arcade Math
            int left_power = throttle + turn;
            int right_power = throttle - turn;

            // 2. Command the Motors
            left_motors.move(left_power);
            right_motors.move(right_power);
        }

} // namespace mylib

