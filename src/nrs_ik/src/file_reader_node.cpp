#include <rclcpp/rclcpp.hpp>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>
#include <iostream>

class FileReaderNode : public rclcpp::Node
{
public:
    FileReaderNode()
    : Node("file_reader_node")
    {
        std::string file_path = "/home/eunseop/nrs_ws/src/rtde_handarm2/data/Hand_G_recording.txt";
        RCLCPP_INFO(this->get_logger(), "📂 Trying to open file: '%s'", file_path.c_str());

        std::ifstream file(file_path);
        if (!file.is_open())
        {
            RCLCPP_ERROR(this->get_logger(), "❌ Failed to open file: '%s'", file_path.c_str());
            return;
        }

        std::string line;
        int line_num = 0;
        while (std::getline(file, line))
        {
            line_num++;
            std::istringstream iss(line);
            std::vector<double> values;
            double value;
            while (iss >> value)
            {
                values.push_back(value);
            }

            // 출력
            std::ostringstream oss;
            oss << "📄 Line " << line_num << ": ";
            for (size_t i = 0; i < values.size(); ++i)
            {
                oss << values[i];
                if (i != values.size() - 1)
                    oss << ", ";
            }
            RCLCPP_INFO(this->get_logger(), "%s", oss.str().c_str());
        }

        file.close();
        rclcpp::shutdown();  // 노드 자동 종료
    }
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FileReaderNode>());
    return 0;
}
