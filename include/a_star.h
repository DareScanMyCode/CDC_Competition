#include <Eigen/Dense>
#include <vector>
#include <unordered_set>
#include <unordered_map>
#include <queue>
#include <functional>
#include "inspector.h" //使用我的grid_map
#ifndef ASTAR_H
#define ASTAR_H


namespace std {
    template <>
    struct hash<Eigen::Matrix<int, 3, 1>> {
        std::size_t operator()(const Eigen::Matrix<int, 3, 1>& matrix) const {
            std::size_t seed = 0;
            for (int i = 0; i < 3; ++i) {
                // Combine the hash of each element in the matrix
                seed ^= std::hash<int>()(matrix(i));
            }
            return seed;
        }
    };
}


class AStar
{
public:
    GridMap map;
    bool search(std::vector<Eigen::Vector3d> &pos_list, Eigen::Vector3d start_pos, Eigen::Vector3d end_pos);

private:
    struct AStarNode {
        Eigen::Vector3i idx;
        AStarNode* parent;
        double g, h, f;
        
        AStarNode(const Eigen::Vector3i& idx, AStarNode* parent, double g, double h, double f)
            : idx(idx), parent(parent), g(g), h(h), f(f) {}
    };

    struct CompareNodes {
        bool operator()(const AStarNode* a, const AStarNode* b) const {
            return a->f > b->f;
        }
    };
    std::unordered_set<Eigen::Vector3i> close_list;
    // Use priority queue for open_list
    std::priority_queue<AStarNode*, std::vector<AStarNode*>, CompareNodes> open_list;
    std::unordered_set<Eigen::Vector3i> in_open_list; // To track nodes in open_list
};

bool AStar::search(std::vector<Eigen::Vector3d> &pos_list, Eigen::Vector3d start_pos, Eigen::Vector3d end_pos) {
    while (!open_list.empty()) {
        AStarNode* node = open_list.top();
        open_list.pop();
        delete node;
    }
    close_list.clear();
    in_open_list.clear();

    if (!map.inmap(end_pos)) {
        std::cout << "end_pos is out of map" << std::endl;
        return false;
    }

    Eigen::Vector3i start_idx, end_idx;
    map.pos2index(start_pos, start_idx);
    map.pos2index(end_pos, end_idx);

    std::vector<Eigen::Vector3i> direction;
    for (int i = -1; i <= 1; i++) {
        for (int j = -1; j <= 1; j++) {
            for (int k = -1; k <= 1; k++) {
                if (i == 0 && j == 0 && k == 0) {
                    continue;
                }
                Eigen::Vector3i dir(i, j, k);
                direction.push_back(dir);
            }
        }
    }

    open_list.push(new AStarNode(start_idx, nullptr, 0, (end_idx - start_idx).cast<double>().norm(), 0));
    in_open_list.insert(start_idx);

    // int max_loop = 1000;

    while (!open_list.empty() && close_list.size()<12000) {
        AStarNode* cur_node = open_list.top();
        open_list.pop();

        in_open_list.erase(cur_node->idx);
        close_list.insert(cur_node->idx);
    
        if (cur_node->idx == end_idx) {
            AStarNode* temp_node = cur_node;
            while (temp_node != nullptr) {
                Eigen::Vector3d pos = map.index2pos(temp_node->idx);
                pos_list.push_back(pos);
                temp_node = temp_node->parent;
            }
            return true;
        }

        for (size_t i = 0; i < direction.size(); i++) {
            Eigen::Vector3i new_idx = cur_node->idx + direction[i];

            //如果不在地图，或者已经在close_list中，跳过
            if (!map.inmap(new_idx) || close_list.find(new_idx) != close_list.end()) {
                continue;
            }
            //如果被占用，跳过
            if (map.get_occupancy_idx(new_idx, false)) {
                continue;
            }
            double g_x,g_y,g_z,h_x,h_y,h_z;
            g_x = new_idx(0) - cur_node->idx(0);
            g_y = new_idx(1) - cur_node->idx(1);
            g_z = new_idx(2) - cur_node->idx(2);
            h_x = end_idx(0) - new_idx(0);
            h_y = end_idx(1) - new_idx(1);
            h_z = end_idx(2) - new_idx(2);
            double g = cur_node->g + abs(g_x)+abs(g_y)+abs(g_z);
            double h = abs(h_x)+abs(h_y)+abs(h_z);
            // double g = cur_node->g + (new_idx - cur_node->idx).cast<double>().norm();
            // double h = (end_idx - new_idx).cast<double>().norm();
            double f = g*0.9 + h;
            //tie_breaker
            f*=1.00001;
            //如果也不在open_list中，加入open_list
            if (in_open_list.find(new_idx) == in_open_list.end()) {
                open_list.push(new AStarNode(new_idx, cur_node, g, h, f));
                in_open_list.insert(new_idx);
            }
        }
    }
    //如果最后找不到
    //pos_list填充close_list中的点
    for (auto it = close_list.begin(); it != close_list.end(); it++) {
        Eigen::Vector3d pos = map.index2pos(*it);
        pos_list.push_back(pos);
    }
    return false;
};


#endif // ASTAR_H