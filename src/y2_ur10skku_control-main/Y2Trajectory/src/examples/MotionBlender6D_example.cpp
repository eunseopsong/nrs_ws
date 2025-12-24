#include "Y2Trajectory/MotionBlender6D.hpp"

int main()
{
    YMatrix position = {
        {0.0, 0.0, 0.0, 0.0, 0.0, 0.0},
        {1.0, 1.0, 1.0, M_PI/4, M_PI/4, M_PI/4},
        {1.0, 1.0, 1.0, M_PI/4, M_PI/4, M_PI/4},
        {2.0, 2.0, 2.0, M_PI/3, M_PI/3, M_PI/3}
    };
    std::vector<double> velocity = {0.5, 0.5, 0.0, 0.0};  // Example velocities (first element is ignored)
    std::vector<double> ang_velocity = {0.0, 0.0, 0.0, M_PI / 6};  // Example angular velocities (first element is ignored, rad/s)
    std::vector<double> holding_time = {0.0, 0.0, 3.0, 0.0};  // Example holding time (holding time at this point, s)
    double angVelLimit = M_PI / 6;  // Example angular velocity limit (30 degrees per second)
    double startingTime = 2.0;  // Starting time of the motion
    double lastRestingTime = 2.0;  // Time when the motion comes to rest
    double accelerationTime = 1.0;  // Time taken to accelerate
    double samplingTime = 0.005;  // Sampling time interval

    MotionBlender6D blender(position, velocity, ang_velocity, holding_time, angVelLimit, startingTime, lastRestingTime, accelerationTime, samplingTime);
    YMatrix blendedMotion = blender.blendMotion();

    /* Save the motoion profile data */
    std::string source_file = __FILE__;
    std::filesystem::path source_path(source_file);
    std::string log_dir = source_path.parent_path().string() + "/log/blended_motion6D.txt";
    blendedMotion.saveToFile(log_dir);

    return 0;
}