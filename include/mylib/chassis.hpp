#pragma once
#include "api.h"
#include "mylib/odom.hpp" // The new Odometry library
#include <vector>
#include <cstdint>
#include <memory>

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
        // Core Hardware
        pros::MotorGroup left_motors;
        pros::MotorGroup right_motors;

        // Optional Hardware (Smart Pointers for memory safety)
        std::unique_ptr<pros::Imu> imu;
        std::unique_ptr<pros::Rotation> vertical_encoder;
        std::unique_ptr<pros::Rotation> horizontal_encoder;
        std::unique_ptr<pros::Distance> distance_sensor;

        // Controller Configurations
        PIDConstants drive_pid;
        PIDConstants turn_pid;

        // Enhanced driver control parameters
        float active_curve = 0.0f;  
        int active_deadband = 5;   

        // --- ODOMETRY COMPONENTS ---
        std::unique_ptr<Odometry> odom;
        std::unique_ptr<pros::Task> odom_task;

        /**
         * @brief Internal filter for deadbands and polynomial curves.
         */
        int apply_filters(int input);    

        /**
         * @brief The infinite loop that calculates position.
         */
        void odom_loop();

        /**
         * @brief A static "trampoline" function required by PROS to launch class methods as tasks.
         */
        static void odom_trampoline(void* context);

    public:
        /**
         * @brief Universal constructor for dynamic motor and sensor setup.
         */
        Chassis(const ChassisConfig& hw_config, 
                const PIDConstants& drive_consts,
                const PIDConstants& turn_consts);

        ~Chassis() = default;

        // Driver Control
        void arcade(int throttle, int turn);
        void set_joystick_curves(float curve_amount, int deadband_amount);

        // Autonomous Movement Placeholders
        void drive_distance(float target_inches);
        void turn_to_angle(float target_degrees);
    };

} // namespace mylib