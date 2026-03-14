#include "mylib/chassis.hpp"
#include <cmath>     
#include <algorithm> 

namespace mylib {

    Chassis::Chassis(const ChassisConfig& config, 
                     const PIDConstants& drive_consts,
                     const PIDConstants& turn_consts)
        : left_motors(config.left_motors),
          right_motors(config.right_motors),
          drive_pid(drive_consts),
          turn_pid(turn_consts) 
    {
        if (config.imu_port != 0) 
            imu = std::make_unique<pros::Imu>(config.imu_port);
        
        if (config.vertical_wheel_port != 0) 
            vertical_encoder = std::make_unique<pros::Rotation>(config.vertical_wheel_port);
        
        if (config.horizontal_wheel_port != 0) 
            horizontal_encoder = std::make_unique<pros::Rotation>(config.horizontal_wheel_port);
        
        if (config.distance_sensor_port != 0) 
            distance_sensor = std::make_unique<pros::Distance>(config.distance_sensor_port);

        odom = std::make_unique<Odometry>(100.0f, 100.0f, 50.8f, 36000.0f);
        odom_task = std::make_unique<pros::Task>(odom_trampoline, this, "Odometry Task");
    }

    void Chassis::set_joystick_curves(float curve_amount, int deadband_amount) {
        if (curve_amount < 0.0f) curve_amount = 0.0f;
        if (curve_amount > 1.0f) curve_amount = 1.0f;
        
        active_curve = curve_amount;
        active_deadband = deadband_amount;
    }

    int Chassis::apply_filters(int input) {
        if (std::abs(input) < active_deadband) return 0;

        float x = input / 127.0f;
        float curved_x = (active_curve * (x * x * x)) + ((1.0f - active_curve) * x);
        
        return static_cast<int>(std::round(curved_x * 127.0f));
    }

    void Chassis::arcade(int throttle, int turn) {
        int f_throttle = apply_filters(throttle);
        int f_turn = apply_filters(turn);

        float left_p = static_cast<float>(f_throttle + f_turn);
        float right_p = static_cast<float>(f_throttle - f_turn);

        float max_val = std::max(std::abs(left_p), std::abs(right_p));
        if (max_val > 127.0f) {
            float scale = 127.0f / max_val;
            left_p *= scale;
            right_p *= scale;
        }

        left_motors.move(static_cast<int>(std::round(left_p)));
        right_motors.move(static_cast<int>(std::round(right_p)));
    }

    void Chassis::odom_trampoline(void* context) {
        static_cast<Chassis*>(context)->odom_loop();
    }

    void Chassis::odom_loop() {
        while (true) {
            float vert_ticks = vertical_encoder ? vertical_encoder->get_position() : 0.0f;
            float horiz_ticks = horizontal_encoder ? horizontal_encoder->get_position() : 0.0f;
            float heading_deg = imu ? imu->get_heading() : 0.0f;

            odom->update(vert_ticks, horiz_ticks, heading_deg);

            pros::delay(10);
        }
    }

    void Chassis::drive_distance(float target_inches) {}
    void Chassis::turn_to_angle(float target_degrees) {}

    // --- NEW: Odometry Getter ---
    Pose Chassis::get_pose() const {
        // Return the global coordinates safely from the Odometry pointer
        return odom->get_pose();
    }

} // namespace mylib