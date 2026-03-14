#include "mylib/chassis.hpp"
#include <cmath>     
#include <algorithm> 

namespace mylib {

    /**
     * @brief Constructor using member initializer lists for optimized startup.
     */
    Chassis::Chassis(const ChassisConfig& config, 
                     const PIDConstants& drive_consts,
                     const PIDConstants& turn_consts)
        : left_motors(config.left_motors),
          right_motors(config.right_motors),
          drive_pid(drive_consts),
          turn_pid(turn_consts) 
    {
        // Allocate optional sensors only if valid ports are provided
        if (config.imu_port != 0) 
            imu = std::make_unique<pros::Imu>(config.imu_port);
        
        if (config.vertical_wheel_port != 0) 
            vertical_encoder = std::make_unique<pros::Rotation>(config.vertical_wheel_port);
        
        if (config.horizontal_wheel_port != 0) 
            horizontal_encoder = std::make_unique<pros::Rotation>(config.horizontal_wheel_port);
        
        if (config.distance_sensor_port != 0) 
            distance_sensor = std::make_unique<pros::Distance>(config.distance_sensor_port);
    }

    /**
     * @brief Configure teleop joystick response feel.
     */
    void Chassis::set_joystick_curves(float curve_amount, int deadband_amount) {
        // Manual range check for maximum compatibility
        if (curve_amount < 0.0f) curve_amount = 0.0f;
        if (curve_amount > 1.0f) curve_amount = 1.0f;
        
        active_curve = curve_amount;
        active_deadband = deadband_amount;
    }

    /**
     * @brief Math helper to apply deadband and cubic polynomial curving.
     */
    int Chassis::apply_filters(int input) {
        if (std::abs(input) < active_deadband) return 0;

        float x = input / 127.0f;
        float curved_x = (active_curve * (x * x * x)) + ((1.0f - active_curve) * x);
        
        return static_cast<int>(std::round(curved_x * 127.0f));
    }

    /**
     * @brief Arcade drive with True Ratio Desaturation to maintain turning arcs.
     */
    void Chassis::arcade(int throttle, int turn) {
        int f_throttle = apply_filters(throttle);
        int f_turn = apply_filters(turn);

        float left_p = static_cast<float>(f_throttle + f_turn);
        float right_p = static_cast<float>(f_throttle - f_turn);

        // Desaturation logic: scale power down if it exceeds 127 while keeping the ratio
        float max_val = std::max(std::abs(left_p), std::abs(right_p));
        if (max_val > 127.0f) {
            float scale = 127.0f / max_val;
            left_p *= scale;
            right_p *= scale;
        }

        left_motors.move(static_cast<int>(std::round(left_p)));
        right_motors.move(static_cast<int>(std::round(right_p)));
    }

    // --- AUTONOMOUS STUBS ---
    // These must exist so the project can build successfully.
    void Chassis::drive_distance(float target_inches) {}
    void Chassis::turn_to_angle(float target_degrees) {}

} // namespace mylib