#include "rclcpp/rclcpp.hpp"
#include "Y2ForceCon/admittance_control.hpp"


class ACexampleNode: public rclcpp::Node
{
    public:
        ACexampleNode(): Node("ACexample"),adm_ctrl_(0.01)
        {
            // Set initial mass, damping, and stiffness
            adm_ctrl_.adm_1D_MDK(1.0, 5.0, 50.0);
            
            // Initialize variables
            xd_ = 0.0;
            Fd_ = 0.0;
            Fext_ = 0.0;
            
            // Create timer with 100Hz (10ms period)
            timer_ = this->create_wall_timer(
                std::chrono::milliseconds(10),
                std::bind(&ACexampleNode::timerCallback, this)
            );
            
            RCLCPP_INFO(this->get_logger(), "Admittance control node started");
        }

    private:
        void timerCallback()
        {
            // Generate external force (sine wave)
            Fext_ = 5.0 * sin(this->now().seconds());
            
            // Increment desired position
            xd_ += 0.001;
            
            // Compute admittance control output (position control command)
            double output = adm_ctrl_.adm_1D_control(xd_, Fd_, Fext_);
            
            // Log the results
            RCLCPP_INFO(this->get_logger(), "xd: %.3f, Fext: %.3f, xc: %.3f", 
                        xd_, Fext_, output);
        }

        Yadmittance_control adm_ctrl_;
        rclcpp::TimerBase::SharedPtr timer_;

        double xd_;
        double Fd_;
        double Fext_;
};



int main(int argc, char** argv)
{
    rclcpp::init(argc,argv);
    auto node = std::make_shared<ACexampleNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}