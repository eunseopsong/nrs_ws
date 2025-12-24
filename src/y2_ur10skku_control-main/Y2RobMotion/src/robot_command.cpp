#include "Y2RobMotion/robot_command.hpp"


robot_command::robot_command(rclcpp::Node* node, const std::string& robot_name, double control_period, PathGenParam& pg_param_)
: node_(node),robot_name_(robot_name),control_period_(control_period),pg_param(pg_param_),
current_position(6,0.0)
{
    // Topic names
    std::string current_p_sub_topic = robot_name_ + "/currentP";
    std::string cmd_motion_pub_topic = robot_name_ + "/cmdMotion";
    std::string cmd_pub_topic = robot_name_ + "/cmdMode";

    // Initialize publishers and subscribers
    current_p_sub_ = node_->create_subscription<std_msgs::msg::Float64MultiArray>(
        current_p_sub_topic, 1, 
        std::bind(&robot_command::currentPCallback, this, std::placeholders::_1));

    cmd_motion_pub_ = node_->create_publisher<std_msgs::msg::Float64MultiArray>(
        cmd_motion_pub_topic, 1);

    cmd_pub_ = node_->create_publisher<std_msgs::msg::String>(
        cmd_pub_topic, 10);

    RCLCPP_INFO(node_->get_logger(), "UrCmd node initialized for robot: %s", robot_name_.c_str());
}

void robot_command::currentPCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
{
    if (!robot_init) {
        robot_init = true;
        RCLCPP_INFO(node_->get_logger(), "Joint states received! Robot initialized.");
    }

    for (size_t i = 0; i < 6 && i < msg->data.size(); ++i) {
        current_position[i] = msg->data[i];
    }
}

void robot_command::sendCommand(const std::string& command, const YMatrix& loaded_motion)
{
    if (command == "PTP") {
        PTP_command_gen(loaded_motion);
    }
    else if (command == "TxtLoad") {
        TxtLoad_command_gen(loaded_motion);
    }
    else if (command == "Idling") {
        RCLCPP_INFO(node_->get_logger(), "Idling mode activated");
        cmd_msg_.data = command;
        cmd_pub_->publish(cmd_msg_);
    }
    else if (command == "Guiding") {
        RCLCPP_INFO(node_->get_logger(), "Guiding mode activated");
        cmd_msg_.data = command;
        cmd_pub_->publish(cmd_msg_);
    }
    else {
        RCLCPP_ERROR(node_->get_logger(), "Unknown command: %s", command.c_str());
    }
}

YMatrix robot_command::PTP_command_input()
{
    std::vector<double> ptp_target_pose(6, 0.0);
    YMatrix ptp_loaded_motion(1, 6);

    printf("\033[33mCurrent position(mm,degree) x:%.2f, y:%.2f, z:%.2f, wx:%.2f, wy:%.2f, wz:%.2f\033[0m\n",
           current_position[0], current_position[1], current_position[2],
           RadianToDegree(current_position[3]), RadianToDegree(current_position[4]), RadianToDegree(current_position[5]));

    std::cout << "Enter target pose(8888:Same with current, 9999:Quit):" << std::endl;
    
    // X coordinate
    std::cout << "X(mm): "; 
    std::cin >> ptp_target_pose[0];
    if (ptp_target_pose[0] == 9999) { 
        rclcpp::shutdown();
        std::exit(1); 
    }
    else if (ptp_target_pose[0] == 8888) { 
        ptp_target_pose[0] = current_position[0]; 
    }

    // Y coordinate
    std::cout << "Y(mm): "; 
    std::cin >> ptp_target_pose[1];
    if (ptp_target_pose[1] == 9999) { 
        rclcpp::shutdown();
        std::exit(1); 
    }
    else if (ptp_target_pose[1] == 8888) { 
        ptp_target_pose[1] = current_position[1]; 
    }

    // Z coordinate
    std::cout << "Z(mm): "; 
    std::cin >> ptp_target_pose[2];
    if (ptp_target_pose[2] == 9999) { 
        rclcpp::shutdown();
        std::exit(1); 
    }
    else if (ptp_target_pose[2] == 8888) { 
        ptp_target_pose[2] = current_position[2]; 
    }

    // Orientation Wx
    std::cout << "Wx(degree): "; 
    std::cin >> ptp_target_pose[3];
    if (ptp_target_pose[3] == 9999) { 
        rclcpp::shutdown();
        std::exit(1); 
    }
    else if (ptp_target_pose[3] == 8888) { 
        ptp_target_pose[3] = RadianToDegree(current_position[3]); 
    }

    // Orientation Wy
    std::cout << "Wy(degree): "; 
    std::cin >> ptp_target_pose[4];
    if (ptp_target_pose[4] == 9999) { 
        rclcpp::shutdown();
        std::exit(1); 
    }
    else if (ptp_target_pose[4] == 8888) { 
        ptp_target_pose[4] = RadianToDegree(current_position[4]); 
    }

    // Orientation Wz
    std::cout << "Wz(degree): "; 
    std::cin >> ptp_target_pose[5];
    if (ptp_target_pose[5] == 9999) { 
        rclcpp::shutdown();
        std::exit(1); 
    }
    else if (ptp_target_pose[5] == 8888) { 
        ptp_target_pose[5] = RadianToDegree(current_position[5]); 
    }

    // Target velocity
    std::cout << "Target_vel(degree/s, 9999:Quit): "; 
    std::cin >> pg_param.ptp_target_velocity;
    if (pg_param.ptp_target_velocity == 9999) { 
        rclcpp::shutdown();
        std::exit(1); 
    }

    std::cout << "Target pose: [" << ptp_target_pose[0] << ", " << ptp_target_pose[1] 
              << ", " << ptp_target_pose[2] << ", " << ptp_target_pose[3] 
              << ", " << ptp_target_pose[4] << ", " << ptp_target_pose[5] << "]" << std::endl;

    ptp_loaded_motion = {{ptp_target_pose[0], ptp_target_pose[1], ptp_target_pose[2], 
                          ptp_target_pose[3], ptp_target_pose[4], ptp_target_pose[5]}};

    return ptp_loaded_motion;
}

void robot_command::PTP_command_gen(const YMatrix& loaded_motion)
{
    // Command mode publishing
    cmd_msg_.data = "Position";
    cmd_pub_->publish(cmd_msg_);

    YMatrix position = {
        {current_position[0], current_position[1], current_position[2], 
         current_position[3], current_position[4], current_position[5]},
        {loaded_motion[0][0], loaded_motion[0][1], loaded_motion[0][2], 
         DegreeToRadian(loaded_motion[0][3]), DegreeToRadian(loaded_motion[0][4]), DegreeToRadian(loaded_motion[0][5])}
    };
    
    std::vector<double> velocity = {0.0, pg_param.ptp_target_velocity};
    std::vector<double> ang_velocity = {0.0, 0.0};
    std::vector<double> holding_time = {0.0, 0.0};
    double starting_time = pg_param.startingTime;
    double last_resting_time = pg_param.lastRestingTime;

    printf("*** Input pose ***\n");
    position.print();

    MotionBlender6D blender(position, velocity, ang_velocity, holding_time, DegreeToRadian(pg_param.angularVelocityLimit), 
                           starting_time, last_resting_time, pg_param.accelerationTime, control_period_);
    YMatrix blended_motion = blender.blendMotion(pg_param.defualt_travelTime);

    // Prepare and send motion commands
    printf("Path Transferring...\n");
    
    // auto control_period = std::chrono::duration<double>(control_period_);
    auto control_period = std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(control_period_)
    );
    auto next_time = std::chrono::steady_clock::now();
    
    for (const auto& pos : blended_motion) {
        if (!rclcpp::ok()) {
            RCLCPP_WARN(node_->get_logger(), "ROS is not ok, stopping motion transfer.");
            return;
        }
        
        cmd_motion_msg_.data.clear();
        for (const auto& value : pos) {
            cmd_motion_msg_.data.push_back(value);
        }

        cmd_motion_pub_->publish(cmd_motion_msg_);

        // Precise timing control
        next_time += control_period;

        std::this_thread::sleep_until(next_time);

    }
    
    printf("Path Transferred.\n");
}

void robot_command::TxtLoad_command_gen(const YMatrix& loaded_motion)
{
    // 6D Motion profile generation
    if (pg_param.loadFileType == "cmd_6D") {
        // Command mode publishing
        cmd_msg_.data = "Position";
        cmd_pub_->publish(cmd_msg_);

        // Announcements
        printf("\033[32mLoading 6D motion profile from %s...\033[0m\n", pg_param.loadFileType.c_str());
        loaded_motion.print();
        
        // colomn number warning
        if(loaded_motion.cols()!=9){
            printf("\033[33mthere is not proper setup inside of cmd_6D !! \033[0m\n");
            return;
        }
        
        // Disassemble to each command (Position, velocity)
        YMatrix loaded_position(loaded_motion.rows() + 2, 6);
        std::vector<double> loaded_velocity(loaded_motion.rows() + 2, 0.0);
        std::vector<double> loaded_ang_velocity(loaded_motion.rows() + 2, 0.0);
        std::vector<double> loaded_holding_time(loaded_motion.rows() + 2, 0.0);

        loaded_position.insert(1, 0, loaded_motion.extract(0, 0, loaded_motion.rows(), 6));
        auto loaded_velocity_matrix = loaded_motion.extract(0, 6, loaded_motion.rows(), 1);
        auto loaded_ang_velocity_matrix = loaded_motion.extract(0, 7, loaded_motion.rows(), 1);
        auto loaded_holding_time_matrix = loaded_motion.extract(0, 8, loaded_motion.rows(), 1);

        for (int i = 0; i < loaded_velocity_matrix.rows(); i++) {
            loaded_velocity[i + 1] = loaded_velocity_matrix[i][0];
            loaded_ang_velocity[i + 1] = loaded_ang_velocity_matrix[i][0];
            loaded_holding_time[i + 1] = loaded_holding_time_matrix[i][0];
        }
        
        // Convert degrees to radians
        for (int i = 0; i < loaded_position.rows(); i++) {
            for (int j = 0; j < loaded_position.cols(); j++) {
                if (j >= 3) { 
                    loaded_position[i][j] = DegreeToRadian(loaded_position[i][j]); 
                }
            }
        }

        // Insert start & end pose
        loaded_position.insert(0, 0, {{current_position[0], current_position[1], current_position[2], 
                                      current_position[3], current_position[4], current_position[5]}});
        loaded_position.insert(loaded_position.rows() - 1, 0, {{current_position[0], current_position[1], current_position[2], 
                                                               current_position[3], current_position[4], current_position[5]}});
        
        printf("Loaded Position:\n");
        loaded_position.print();

        // Insert start & end (velocity & angular_velocity & holding_time)
        loaded_velocity[1] = pg_param.initialTransferSpeed;
        loaded_velocity[loaded_velocity.size() - 1] = pg_param.initialTransferSpeed;

        loaded_ang_velocity[1] = 0.0;
        loaded_ang_velocity[loaded_ang_velocity.size() - 1] = 0.0;

        loaded_holding_time[1] = 0.0;
        loaded_holding_time[loaded_holding_time.size() - 1] = 0.0;
        
        printf("\033[31mLoaded Velocity:\033[0m\n");
        for (const auto& vel : loaded_velocity) { 
            std::cout << vel << " "; 
        }  
        std::cout << std::endl;

        printf("\033[31mLoaded Angular Velocity:\033[0m\n");
        for (const auto& vel : loaded_ang_velocity) { 
            std::cout << vel << " "; 
        }  
        std::cout << std::endl;

        printf("\033[31mLoaded Holding Time:\033[0m\n");
        for (const auto& vel : loaded_holding_time) { 
            std::cout << vel << " "; 
        }  
        std::cout << std::endl;

        // 6D Motion generation
        MotionBlender6D blender(loaded_position, loaded_velocity, loaded_ang_velocity, loaded_holding_time,
                               DegreeToRadian(pg_param.angularVelocityLimit), 
                               pg_param.startingTime, pg_param.lastRestingTime, 
                               pg_param.accelerationTime, control_period_);

        YMatrix blended_motion = blender.blendMotion(pg_param.defualt_travelTime);
        printf("Blended motion generated\n");

        // Transfer to Robot
        printf("Path Transferring...\n");
        
        // auto control_period = std::chrono::duration<double>(control_period_);
        auto control_period = std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::duration<double>(control_period_)
        );
        auto next_time = std::chrono::steady_clock::now();
        
        for (const auto& pos : blended_motion) {
            if (!rclcpp::ok()) {
                RCLCPP_WARN(node_->get_logger(), "ROS is not ok, stopping motion transfer.");
                return;
            }
            
            cmd_motion_msg_.data.clear();
            for (const auto& value : pos) {
                cmd_motion_msg_.data.push_back(value);
            }
            
            cmd_motion_pub_->publish(cmd_motion_msg_);
            
            next_time += control_period;
            std::this_thread::sleep_until(next_time);
        }
        
        printf("Path Transferred.\n");
    }
    else if (pg_param.loadFileType == "cmd_9D") {
        /* Command mode publishing */
        cmd_msg_.data = "Force";
        cmd_pub_->publish(cmd_msg_);

        /* Announcements */ 
        printf("\033[32mLoading 9D motion profile from %s...\033[0m\n", pg_param.loadFileType.c_str());
        loaded_motion.print();
        
        /* colomn number warning */
        if(loaded_motion.cols()!=12){
            printf("\033[33mthere is not proper setup inside of cmd_9D !! \033[0m\n");
            return;
        }

        /* Allocate the size of each command (Position, velocity, force) */
        YMatrix loaded_position(loaded_motion.rows() + 2, 9);
        std::vector<double> loaded_velocity(loaded_motion.rows() + 2, 0.0);
        std::vector<double> loaded_ang_velocity(loaded_motion.rows() + 2, 0.0);
        std::vector<double> loaded_holding_time(loaded_motion.rows() + 2, 0.0);

        loaded_position.insert(1, 0, loaded_motion.extract(0, 0, loaded_motion.rows(), 9));
        auto loaded_velocity_matrix = loaded_motion.extract(0, 9, loaded_motion.rows(), 1);
        auto loaded_ang_velocity_matrix = loaded_motion.extract(0, 10, loaded_motion.rows(), 1);
        auto loaded_holding_time_matrix = loaded_motion.extract(0, 11, loaded_motion.rows(), 1);

        for (int i = 0; i < loaded_velocity_matrix.rows(); i++) {
            loaded_velocity[i + 1] = loaded_velocity_matrix[i][0];
            loaded_ang_velocity[i + 1] = loaded_ang_velocity_matrix[i][0];
            loaded_holding_time[i + 1] = loaded_holding_time_matrix[i][0];
        }
        
        // Convert degrees to radians
        for (int i = 0; i < loaded_position.rows(); i++) {
            for (int j = 0; j < loaded_position.cols(); j++) {
                if (j >= 3 && j < 6) { 
                    loaded_position[i][j] = DegreeToRadian(loaded_position[i][j]); 
                }
            }
        }

        // Insert start & end pose
        loaded_position.insert(0, 0, {{current_position[0], current_position[1], current_position[2], 
                                      current_position[3], current_position[4], current_position[5]}});
        loaded_position.insert(loaded_position.rows() - 1, 0, {{current_position[0], current_position[1], current_position[2], 
                                                               current_position[3], current_position[4], current_position[5]}});
        
        printf("Loaded Position:\n");
        loaded_position.print();

        // Insert start & end (velocity & angular_velocity & holding_time)
        loaded_velocity[1] = pg_param.initialTransferSpeed;
        loaded_velocity[loaded_velocity.size() - 1] = pg_param.initialTransferSpeed;

        loaded_ang_velocity[1] = 0.0;
        loaded_ang_velocity[loaded_ang_velocity.size() - 1] = 0.0;

        loaded_holding_time[1] = 0.0;
        loaded_holding_time[loaded_holding_time.size() - 1] = 0.0;
        
        printf("\033[31mLoaded Velocity:\033[0m\n");
        for (const auto& vel : loaded_velocity) { 
            std::cout << vel << " "; 
        }  
        std::cout << std::endl;

        printf("\033[31mLoaded Angular Velocity:\033[0m\n");
        for (const auto& vel : loaded_ang_velocity) { 
            std::cout << vel << " "; 
        }  
        std::cout << std::endl;

        printf("\033[31mLoaded Holding Time:\033[0m\n");
        for (const auto& vel : loaded_holding_time) { 
            std::cout << vel << " "; 
        }  
        std::cout << std::endl;

        /* 9D Motion generation */ 
        MotionBlender9D blender(loaded_position, loaded_velocity, loaded_ang_velocity, loaded_holding_time,
                               DegreeToRadian(pg_param.angularVelocityLimit), 
                               pg_param.startingTime, pg_param.lastRestingTime, 
                               pg_param.accelerationTime, control_period_);

        YMatrix blended_motion = blender.blendMotion(pg_param.defualt_travelTime);
        printf("Blended motion generated\n");

        /* Save the motoion profile data */
        std::string source_file = __FILE__;
        std::filesystem::path source_path(source_file);
        std::string log_dir = source_path.parent_path().string() + "/../txtcmd/cmd_continue9D.txt";
        blended_motion.saveToFile(log_dir);
        RCLCPP_INFO(node_->get_logger(),"9D data was saved to cmd_continue9D");

        // Transfer to Robot
        printf("Path Transferring...\n");
        
        auto control_period = std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::duration<double>(control_period_)
        );
        auto next_time = std::chrono::steady_clock::now();
        
        for (const auto& pos : blended_motion) {
            if (!rclcpp::ok()) {
                RCLCPP_WARN(node_->get_logger(), "ROS is not ok, stopping motion transfer.");
                return;
            }
            
            cmd_motion_msg_.data.clear();
            for (const auto& value : pos) {
                cmd_motion_msg_.data.push_back(value);
            }
            
            cmd_motion_pub_->publish(cmd_motion_msg_);
            
            next_time += control_period;
            std::this_thread::sleep_until(next_time);
        }
        
        printf("Path Transferred.\n");


    }
    // Other motion profile types (cmd_continue6D, cmd_continue9D) can be implemented here
    else if (pg_param.loadFileType == "cmd_continue6D") {
        RCLCPP_WARN(node_->get_logger(), "cmd_continue6D motion profile not implemented yet");
    }
    else if (pg_param.loadFileType == "cmd_continue9D") {
        RCLCPP_WARN(node_->get_logger(), "cmd_continue9D motion profile not implemented yet");
    }
    else {
        RCLCPP_ERROR(node_->get_logger(), "Unknown file type: %s", pg_param.loadFileType.c_str());
    }
}