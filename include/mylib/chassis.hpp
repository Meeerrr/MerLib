#pragma once
#include "api.h"
#include "mylib/odom.hpp" 
#include <vector>
#include <cstdint>
#include <memory>

namespace mylib {

    struct PIDConstants {
        float kP;
        float kI;
        float kD;
    };

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
        pros::MotorGroup left_motors;
        pros::MotorGroup right_motors;

        std::unique_ptr<pros::Imu> imu;
        std::unique_ptr<pros::Rotation> vertical_encoder;
        std::unique_ptr<pros::Rotation> horizontal_encoder;
        std::unique_ptr<pros::Distance> distance_sensor;

        PIDConstants drive_pid;
        PIDConstants turn_pid;

        float active_curve = 0.0f;  
        int active_deadband = 5;   

        std::unique_ptr<Odometry> odom;
        std::unique_ptr<pros::Task> odom_task;

        int apply_filters(int input);    
        void odom_loop();
        static void odom_trampoline(void* context);

    public:
        Chassis(const ChassisConfig& hw_config, 
                const PIDConstants& drive_consts,
                const PIDConstants& turn_consts);

        ~Chassis() = default;

        void arcade(int throttle, int turn);
        void set_joystick_curves(float curve_amount, int deadband_amount);

        void drive_distance(float target_inches);
        void turn_to_angle(float target_degrees);

        // --- NEW: Expose Odometry Coordinates ---
        Pose get_pose() const;
    };

} // namespace mylib