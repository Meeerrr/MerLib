#pragma once
#include <cmath>

namespace mylib {

    /**
     * @brief Structure to represent the robot's global position and heading.
     */
    struct Pose {
        float x;
        float y;
        float theta; // Stored in radians 
    };

    class Odometry {
    private:
        // The robot's current coordinate state
        Pose current_pose;

        // Variables to track the previous state (needed to calculate Deltas)
        float prev_vertical_ticks = 0.0f;
        float prev_horizontal_ticks = 0.0f;
        float prev_heading_rad = 0.0f;

        // Physical robot measurements required for the tracking math
        float vertical_offset;   // Distance from tracking center to the vertical wheel
        float horizontal_offset; // Distance from tracking center to the horizontal wheel
        float wheel_circumference; 
        float ticks_per_rev;

        // Math helper to convert encoder ticks to physical inches/mm
        float ticks_to_distance(float ticks) const;

    public:
        /**
         * @brief Constructor for the tracking engine.
         * @param vert_off Distance from tracking center to vertical wheel (inches)
         * @param horiz_off Distance from tracking center to horizontal wheel (inches)
         * @param wheel_diam Diameter of the tracking wheels (inches)
         * @param ticks_rev Number of encoder ticks per full wheel revolution
         */
        Odometry(float vert_off, float horiz_off, float wheel_diam, float ticks_rev);

        ~Odometry() = default;

        /**
         * @brief The core tracking algorithm. Must be called in a high-frequency loop (e.g., every 10ms).
         * @param vert_ticks Current raw ticks from the vertical tracking wheel
         * @param horiz_ticks Current raw ticks from the horizontal tracking wheel
         * @param imu_heading_deg Current absolute heading from the IMU in degrees
         */
        void update(float vert_ticks, float horiz_ticks, float imu_heading_deg);

        // Accessors and Mutators
        Pose get_pose() const;
        void set_pose(float x, float y, float theta_deg);
    };

} // namespace mylib