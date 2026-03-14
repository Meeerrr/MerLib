#include "mylib/odom.hpp"
#include <cmath>

namespace mylib {

    // Define mathematical Pi for high-precision conversions
    constexpr float PI = 3.14159265358979323846f;

    Odometry::Odometry(float vert_off_mm, float horiz_off_mm, float wheel_diam_mm, float ticks_rev)
        : vertical_offset(vert_off_mm),
          horizontal_offset(horiz_off_mm),
          wheel_circumference(wheel_diam_mm * PI),
          ticks_per_rev(ticks_rev) 
    {
        // Initialize position to the center of the coordinate system
        current_pose = {0.0f, 0.0f, 0.0f};
    }

    float Odometry::ticks_to_distance(float ticks) const {
        return (ticks / ticks_per_rev) * wheel_circumference;
    }

    void Odometry::update(float vert_ticks, float horiz_ticks, float imu_heading_deg) {
        // 1. Convert IMU degrees to radians for standard C++ trig functions
        float current_heading_rad = imu_heading_deg * (PI / 180.0f);
        
        // 2. Calculate the change (Delta) since the last 10ms cycle
        float delta_vert_ticks = vert_ticks - prev_vertical_ticks;
        float delta_horiz_ticks = horiz_ticks - prev_horizontal_ticks;
        float delta_theta = current_heading_rad - prev_heading_rad;

        // 3. Store current values as previous for the NEXT cycle
        prev_vertical_ticks = vert_ticks;
        prev_horizontal_ticks = horiz_ticks;
        prev_heading_rad = current_heading_rad;

        // 4. Convert raw ticks into physical millimeters
        float delta_L = ticks_to_distance(delta_vert_ticks);
        float delta_S = ticks_to_distance(delta_horiz_ticks);

        // 5. Calculate Local Translation (Compensate for tracking wheel offsets)
        float local_Y = delta_L;
        float local_X = delta_S;

        // Avoid division by zero if the robot drove perfectly straight
        if (std::abs(delta_theta) > 0.0001f) {
            // Isolate pure forward/sideways movement from rotational wheel scrubbing
            local_Y = delta_L - (delta_theta * vertical_offset);
            local_X = delta_S - (delta_theta * horizontal_offset);
            
            // Apply the Small Arc Approximation to get the exact chord length
            float sin_half_theta = std::sin(delta_theta / 2.0f);
            local_Y = 2.0f * sin_half_theta * (local_Y / delta_theta);
            local_X = 2.0f * sin_half_theta * (local_X / delta_theta);
        }

        // 6. Rotate local movements into the global coordinate grid
        // We use the average heading during the movement for maximum accuracy
        float avg_theta = current_pose.theta + (delta_theta / 2.0f);
        
        float cos_theta = std::cos(avg_theta);
        float sin_theta = std::sin(avg_theta);

        // Standard VEX Field layout: 0 degrees faces the positive Y-axis
        current_pose.x += (local_Y * sin_theta) + (local_X * cos_theta);
        current_pose.y += (local_Y * cos_theta) - (local_X * sin_theta);
        current_pose.theta = current_heading_rad;
    }

    Pose Odometry::get_pose() const {
        return current_pose;
    }

    void Odometry::set_pose(float x_mm, float y_mm, float theta_deg) {
        current_pose.x = x_mm;
        current_pose.y = y_mm;
        current_pose.theta = theta_deg * (PI / 180.0f);
        
        // Hard-reset the previous heading to prevent the robot from thinking 
        // it teleported and spinning wildly on the next cycle.
        prev_heading_rad = current_pose.theta; 
    }

} // namespace mylib