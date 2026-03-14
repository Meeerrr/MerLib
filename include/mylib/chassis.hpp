#pragma once
#include "api.h"
#include <vector>
#include <cstdint>
#include <memory> //safe memory.

namespace mylib {

    /**
     * @brief Structure to hold PID controller constants.
     */
    struct PIDConstants {
        float kP;
        float kI;
        float kD;
    };

    /**
     * @brief Configuration struct for all interchangeable hardware ports.
     * Setting a port to '0' tells the system that the sensor is not installed.
     */
    struct ChassisConfig {
        std::vector<int8_t> left_motors;
        std::vector<int8_t> right_motors;
        int8_t imu_port = 0;              
        int8_t vertical_wheel_port = 0;   
        int8_t horizontal_wheel_port = 0; 
        int8_t distance_sensor_port = 0;  
    };

    class Chassis {
    private:
        // --- Core Hardware (Always Required) ---
        pros::MotorGroup left_motors;
        pros::MotorGroup right_motors;

        // --- Optional Hardware (Smart Pointers) ---
        std::unique_ptr<pros::Imu> imu;
        std::unique_ptr<pros::Rotation> vertical_encoder;
        std::unique_ptr<pros::Rotation> horizontal_encoder;
        std::unique_ptr<pros::Distance> distance_sensor;

        // --- Controller Configurations ---
        PIDConstants drive_pid;
        PIDConstants turn_pid;

    public:
        /**
         * @brief UNIVERSAL CONSTRUCTOR
         * Handles 6 or 8 motors automatically and builds optional sensors dynamically.
         * * @param hw_config The hardware port configuration struct.
         * @param drive_consts PID constants for linear movement.
         * @param turn_consts PID constants for rotational movement.
         */
        Chassis(const ChassisConfig& hw_config, 
                const PIDConstants& drive_consts,
                const PIDConstants& turn_consts);

        /**
         * @brief Standard destructor safely handled by unique_ptrs.
         */
        ~Chassis() = default;

        // --- DRIVER CONTROL ---
        void arcade(int throttle, int turn);

        // --- AUTONOMOUS MOVEMENT ---
        void drive_distance(float target_inches);
        void turn_to_angle(float target_degrees);
    };
    

} // namespace mylib