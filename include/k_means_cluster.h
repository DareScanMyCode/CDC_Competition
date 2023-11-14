#include <iostream>
#include <vector>
#include <Eigen/Dense>
#include <random>
#include <ctime>

using namespace Eigen;
using namespace std;

#ifndef K_MEANS_CLUSTER_H
#define K_MEANS_CLUSTER_H
// 定义K均值聚类函数
std::vector<std::vector<Eigen::Vector3d>>kMeansClustering(const std::vector<Eigen::Vector3d>& data, int numClusters, double convergenceThreshold, int maxIterations) {
    std::vector<Eigen::Vector3d> centroids;
    default_random_engine rng(static_cast<unsigned int>(time(0)));
    uniform_int_distribution<size_t> distribution(0, data.size() - 1);

    for (int i = 0; i < numClusters; ++i) {
        size_t randomIndex = distribution(rng);
        centroids.push_back(data[randomIndex]);
    }

    std::vector<std::vector<Eigen::Vector3d>> clusters(numClusters);
    for (int iteration = 0; iteration < maxIterations; ++iteration) {
        // 清空聚类
        for (int i = 0; i < numClusters; ++i) {
            clusters[i].clear();
        }
        // 分配数据点到最近的聚类中心
        for (const Eigen::Vector3d& point : data) {
            int nearestClusterIndex = 0;
            double minDistance = (point - centroids[0]).norm();

            for (int i = 1; i < numClusters; ++i) {
                double distance = (point - centroids[i]).norm();
                if (distance < minDistance) {
                    minDistance = distance;
                    nearestClusterIndex = i;
                }
            }

            clusters[nearestClusterIndex].push_back(point);
        }

        // 计算新的聚类中心
        std::vector<Eigen::Vector3d> newCentroids;
        for (int i = 0; i < numClusters; ++i) {
            //初始化聚类中心为0
            Eigen::Vector3d newCentroid(Eigen::Vector3d::Zero());
            for (const Eigen::Vector3d& point : clusters[i]) {
                newCentroid += point;
            }
            if (!clusters[i].empty()) {
                newCentroid /= clusters[i].size();
            }
            newCentroids.push_back(newCentroid);
        }

        // 计算新旧中心之间的差异
        double maxCentroidChange = 0.0;
        for (int i = 0; i < numClusters; ++i) {
            double change = (newCentroids[i] - centroids[i]).norm();
            maxCentroidChange = std::max(maxCentroidChange, change);
        }

        centroids = newCentroids;

        if (maxCentroidChange < convergenceThreshold) {
            // 退出迭代，满足退出条件
            break;
        }
    }

    return clusters;
}


std::vector<Eigen::Vector3d> cal_centroid(const std::vector<std::vector<Eigen::Vector3d>>& data, int numClusters){
    std::vector<Eigen::Vector3d> centroids;
    for (int i = 0; i < numClusters; ++i) {
        //初始化聚类中心为0
        Eigen::Vector3d newCentroid(Eigen::Vector3d::Zero());
        for (const Eigen::Vector3d& point : data[i]) {
            newCentroid += point;
        }
        if (!data[i].empty()) {
            newCentroid /= data[i].size();
        }
        centroids.push_back(newCentroid);
    }
    return centroids;
}
#endif //K_MEANS_CLUSTER_H