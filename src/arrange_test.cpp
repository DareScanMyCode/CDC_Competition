#include <iostream>
#include <vector>
#include <Eigen/Dense>
#include <fstream>
#include "../include/photo_taker.h"
// 包含你的 arrange_points 函数的定义

int main() {
    Eigen::Vector3d uav_pos(0.0, 0.0, 0.0);  // 替换为你的 UAV 位置
    std::vector<Eigen::Vector3d> target_pos = {
        Eigen::Vector3d(1.0, 2.0, 0.0),
        Eigen::Vector3d(3.0, 4.0, 0.0),
        Eigen::Vector3d(5.0, 6.0, 0.0),
        Eigen::Vector3d(7.0, 8.0, 0.0),
        Eigen::Vector3d(9.0, 10.0, 0.0),
        Eigen::Vector3d(11.0, 12.0, 0.0),
        // 添加更多目标位置
    };

    // 调用 arrange_points 函数
    std::vector<int> travel_index = arrange_points(uav_pos, target_pos);

    // 将结果保存到文件
    std::ofstream outfile("travel_path.txt");
    if (outfile.is_open()) {
        for (int i : travel_index) {
            outfile << i << std::endl;
        }
        outfile.close();
        std::cout << "Travel path saved to 'travel_path.txt'" << std::endl;
    } else {
        std::cerr << "Failed to open the output file." << std::endl;
    }

    return 0;
}
