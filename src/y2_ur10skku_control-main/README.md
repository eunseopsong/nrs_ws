# y2_ur10skku_control

## HARDWARE
* MAIN ROBOT: UR10CB3 (Supporting: KUKA iiwa7, UR10e)
* FT SENSOR: AIDIN robotics AFT200-D80 (Supporting: eCAN, Ethernet)

## IP
* ROBOT: 192.168.0.47
* PC: 192.168.0.77
* Sensor: 192.168.0.44 (eCAN)

## OS
* UBUNTU 22.04
* ROS2 HUMBLE  

## REQUIRED PACKAGE

### Pytorch 
  > CPU version (w/o GPU, More Stable) -> Control PC  
  > CUDA version (w/ GPU, NVIDIA driver is essential) -> Learning PC  

  > __[CPU only]__  
  > $ cd ~  
  > $ wget https://download.pytorch.org/libtorch/cpu/libtorch-cxx11-abi-shared-with-deps-2.1.0%2Bcpu.zip  
  > $ unzip libtorch-cxx11-abi-shared-with-deps-2.1.0+cpu.zip

  > __[CUDA 12.1 -> example]__  
  > $ wget https://download.pytorch.org/libtorch/cu121/libtorch-cxx11-abi-shared-with-deps-2.1.0%2Bcu121.zip  
  > $ unzip libtorch-cxx11-abi-shared-with-deps-2.1.0+cu121.zip

  > __[Setup the Environment Var.]__  
  > $ echo 'export Torch_DIR=$HOME/libtorch/share/cmake/Torch' >> ~/.bashrc  
  > $ echo 'export LD_LIBRARY_PATH=$HOME/libtorch/lib:$LD_LIBRARY_PATH' >> ~/.bashrc  
  > $ source ~/.bashrc  

### ur_ros2_driver
  > $ sudo apt update  
  > $ sudo apt install ros-humble-ur  

  > __[Basic operation]__  
  > Robot: ur10, Robot IP: 192.168.0.47  
  > $ ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur10 robot_ip:=192.168.0.47  

### OSQP
  > ** Build from source file **  
  > __[Requrements]__   
  > $ sudo apt update  
  > $ sudo apt install git cmake build-essential  
  > __[Download source]__   
  > $ cd ~   
  > $ git clone --recursive https://github.com/osqp/osqp.git  
  > $ cd osqp  
  > __[Generate the build directory & Build]__  
  > $ mkdir build && cd build  
  > $ cmake -G "Unix Makefiles" -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/usr/local ..  
  > $ make -j$(nproc)  
  > __[Install]__  
  > $ sudo make install  
  > __[Update the library cash]__  
  > $ sudo ldconfig  

  > ** Confirm the installation **  
  > __[Confirm the header file]__  
  > $ ls /usr/local/include/osqp/  
  > __[Confirm the lib file]__  
  > $ ls /usr/local/lib | grep -E "libosqp(\.so|\.a)" || true  
  > __[Confirm the pkg-config setting]__  
  > $ export PKG_CONFIG_PATH=/usr/local/lib/pkgconfig:$PKG_CONFIG_PATH    
  > $ pkg-config --cflags --libs osqp    

  > ** Setting the env. variables **  
  > __[Add to ~/.bashrc]__  
  > $ echo 'export PKG_CONFIG_PATH=/usr/local/lib/pkgconfig:$PKG_CONFIG_PATH' >> ~/.bashrc  
  > $ echo 'export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH' >> ~/.bashrc  
  > $ source ~/.bashrc  

## Source Code Setting
### Robot-control Setting
  > __[Y2RobMotion-pkg] -> [include/Y2RobMotion] -> [setup_parameters.hpp]__  
  > New PC Setting : "PACKAGE_BUNDLE_DIR" Must be changed  
  > Robot Type Switching : "ROBOT_KINEMATICS", "CONTROL_PERIOD" Must be changed  
  > TCP Setting : "EE2TCP" Must be changed (Normally {3,4}-> z-axis dir length)

### Sensor Setting
  > __[Y2FT_AQ-pkg] -> [src] -> [FTGetMain.cpp]__  
  > New IP Setting : "FT1_IP" Must be changed  
  > New Tool Setting : "GRAVITY_COMPENSATION_MODE" (Refer. the Gravity Compensation Part)  

## Pendent operation (UR10CB3)  
### URCAP Setup  
  > [로봇 프로그램] -> [프로그램 비우기] -> [<비어있음> 클릭] -> [구조 클릭]  
  > -> [URCaps 에서 External Control 클릭]  
  > -> [위의 설치 클릭] -> [External Control에서 Host IP 설정]  
  > * Host name은 아무런 의미 없음  
  > * Custom port는 50002로 고정  

 ## EXECUTION
> ### Communication with actual robot (Prior. 1)
> ** Single __UR10__ activate **   
> $ ros2 launch Y2RobMotion ur_control.launch.py  
> ** Single __UR10e__ activate **   
> $ ros2 launch Y2RobMotion ur10e_control.launch.py  

> ### Execution robot motion  (Prior. 2)
> ** Execute Single motion **    
> $ ros2 run Y2RobMotion singleArm_motion  

> ### Communication with FT Sensor  (Prior. 3)
> ** Single FT-acuisition **  
> $ ros2 run Y2FT_AQ FTGetMain  

> ### Execution robot command  (Prior. 3)
> ** Execute Single command **    
> $ ros2 run Y2RobMotion singleArm_cmd   

> ### Data Aquisition  (Prior. Any)
> ** Execute Single measure **  
> $ ros2 run Y2RobMotion singleArm_measure  
