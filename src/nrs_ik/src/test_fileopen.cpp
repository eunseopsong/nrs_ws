#include <iostream>
#include <fstream>

int main() {
    std::string path = "/home/eunseop/nrs_ws/src/rtde_handarm2/data/Hand_G_recording.txt";
    std::ifstream file(path);
    
    if (!file.is_open()) {
        std::cerr << "❌ Failed to open file: " << path << std::endl;
        return -1;
    }

    std::cout << "✅ File opened successfully!" << std::endl;
    return 0;
}
