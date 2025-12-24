#include "Y2Trajectory/MotionBlender9D.hpp"

int main()
{
    YMatrix position = { // Example positions (x, y, z, wx, wy, wz, fx, fy, fz) (first element of force is ignored)
        {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
        {1.0, 1.0, 1.0, M_PI/4, M_PI/4, M_PI/4, 0.0, 0.0, 0.0},
        {1.0, 1.0, 1.0, M_PI/4, M_PI/4, M_PI/4, 0.0, 0.0, 5.0},
        {2.0, 2.0, 2.0, M_PI/3, M_PI/3, M_PI/3, 0.0, 0.0, 10.0},
        {1.0, 1.0, 1.0, M_PI/4, M_PI/4, M_PI/4, 0.0, 0.0, 10.0},
        {2.0, 2.0, 2.0, M_PI/3, M_PI/3, M_PI/3, 0.0, 0.0, 0.0}
    };
    std::vector<double> velocity = {0.0, 0.5, 0.0, 0.5, 0.5, 0.0};  // Example velocities (first element is ignored)
    std::vector<double> ang_velocity = {0.0, 0.0, 0.0, 0.5, 0.5, M_PI / 10};  // Example velocities (first element is ignored)
    std::vector<double> holding_time = {0.0, 0.0, 3.0, 0.0, 0.0, 0.0};  // Example holding time (holding time at this point, s)
    double angVelLimit = M_PI / 6;  // Example angular velocity limit (30 degrees per second)
    double startingTime = 2.0;  // Starting time of the motion
    double lastRestingTime = 2.0;  // Time when the motion comes to rest
    double accelerationTime = 1.0;  // Time taken to accelerate
    double samplingTime = 0.005;  // Sampling time interval

    MotionBlender9D blender(position, velocity, ang_velocity, holding_time, angVelLimit, startingTime, lastRestingTime, accelerationTime, samplingTime);
    YMatrix blendedMotion = blender.blendMotion();

    /* Save the motoion profile data */
    std::string source_file = __FILE__;
    std::filesystem::path source_path(source_file);
    std::string log_dir = source_path.parent_path().string() + "/log/blended_motion9D.txt";
    blendedMotion.saveToFile(log_dir);

    return 0;
}