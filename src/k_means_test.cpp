#include <iostream>
#include <vector>
#include <Eigen/Dense>
#include <random>
#include <ctime>
#include <opencv2/opencv.hpp>

#include "../include/k_means_cluster.h"

using namespace Eigen;
using namespace std;
using namespace cv;

// 定义K均值聚类函数和计算中心函数（与之前相同）

int main() {
    // 生成示例数据
    std::vector<Eigen::Vector3d> data;
    // 填充data，这里使用随机数据作为示例
    for (int i = 0; i < 100; ++i) {
        double x = static_cast<double>(rand()) / RAND_MAX * 800.0;
        double y = static_cast<double>(rand()) / RAND_MAX * 800.0;
        double z = static_cast<double>(rand()) / RAND_MAX * 800.0;
        data.push_back(Vector3d(x, y, z));
    }
    // 执行K均值聚类
    int numClusters = 5; // 设置聚类数
    double convergenceThreshold = 0.001; // 设置收敛阈值
    int maxIterations = 100; // 设置最大迭代次数
    std::vector<std::vector<Eigen::Vector3d>> clusters = kMeansClustering(data, numClusters, convergenceThreshold, maxIterations);

    // 可视化结果,三维
    Mat image(800, 800, CV_8UC3, Scalar(255, 255, 255));

    // 设置不同颜色
    vector<Scalar> colors = {Scalar(255, 0, 0), Scalar(0, 255, 0), Scalar(0, 0, 255)};

    for (int i = 0; i < numClusters; ++i) {
        Scalar color = colors[i];
        for (const Eigen::Vector3d& point : clusters[i]) {
            Point pt(point[0], point[1]);
            circle(image, pt, 5, color, -1);
        }
    }

    imshow("K-Means Clustering", image);
    waitKey(0);

    return 0;
}
