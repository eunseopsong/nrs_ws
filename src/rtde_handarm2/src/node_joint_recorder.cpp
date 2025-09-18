#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <fstream>
#include <string>
#include <vector>
#include <H5Cpp.h>   // hdf5 C++ API

class JointRecorderNode : public rclcpp::Node
{
public:
    JointRecorderNode()
    : Node("joint_recorder_node"), sample_count_(0)
    {
        // TXT 파일 열기
        txt_file_path_ = "/home/eunseop/nrs_lab2/datasets/joint_recording.txt";
        txt_file_.open(txt_file_path_, std::ios::out | std::ios::app);
        if (!txt_file_.is_open()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open file: %s", txt_file_path_.c_str());
        }

        // HDF5 파일 생성 (overwrite)
        h5_file_path_ = "/home/eunseop/nrs_lab2/datasets/joint_recording.h5";
        try {
            h5_file_ = std::make_unique<H5::H5File>(h5_file_path_, H5F_ACC_TRUNC);

            // 데이터셋 초기 shape: (0,6) → extendible
            hsize_t dims[2] = {0, 6};
            hsize_t maxdims[2] = {H5S_UNLIMITED, 6};
            H5::DataSpace dataspace(2, dims, maxdims);

            // chunking 설정 (성능 위해)
            hsize_t chunk_dims[2] = {1, 6};
            H5::DSetCreatPropList plist;
            plist.setChunk(2, chunk_dims);

            dataset_ = std::make_unique<H5::DataSet>(
                h5_file_->createDataSet("joint_positions", H5::PredType::NATIVE_DOUBLE, dataspace, plist)
            );
        } catch (H5::Exception &e) {
            RCLCPP_ERROR(this->get_logger(), "HDF5 init failed: %s", e.getDetailMsg().c_str());
        }

        // Subscriber
        sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/isaac_joint_commands", 10,
            std::bind(&JointRecorderNode::callback, this, std::placeholders::_1)
        );

        RCLCPP_INFO(this->get_logger(), "Recording to %s and %s",
                    txt_file_path_.c_str(), h5_file_path_.c_str());
    }

    ~JointRecorderNode()
    {
        if (txt_file_.is_open()) txt_file_.close();
        if (h5_file_) h5_file_->close();
    }

private:
    void callback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        if (msg->position.size() < 6) {
            RCLCPP_WARN(this->get_logger(), "Received less than 6 joints");
            return;
        }

        // --- TXT 저장 ---
        for (size_t i = 0; i < 6; i++) {
            txt_file_ << msg->position[i];
            if (i < 5) txt_file_ << " ";
        }
        txt_file_ << "\n";

        // --- HDF5 저장 ---
        try {
            hsize_t new_dims[2] = {sample_count_ + 1, 6};
            dataset_->extend(new_dims);

            H5::DataSpace fspace = dataset_->getSpace();
            hsize_t start[2] = {sample_count_, 0};
            hsize_t count[2] = {1, 6};
            fspace.selectHyperslab(H5S_SELECT_SET, count, start);

            H5::DataSpace mspace(2, count);
            double row[6];
            for (int i = 0; i < 6; i++) row[i] = msg->position[i];

            dataset_->write(row, H5::PredType::NATIVE_DOUBLE, mspace, fspace);
            sample_count_++;
        } catch (H5::Exception &e) {
            RCLCPP_ERROR(this->get_logger(), "HDF5 write failed: %s", e.getDetailMsg().c_str());
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_;
    std::ofstream txt_file_;
    std::string txt_file_path_;
    std::string h5_file_path_;

    std::unique_ptr<H5::H5File> h5_file_;
    std::unique_ptr<H5::DataSet> dataset_;
    size_t sample_count_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<JointRecorderNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
