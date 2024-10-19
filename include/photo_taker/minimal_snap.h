#include <queue> 
#include <Eigen/Dense>
#include "inspector/trajectory_generator.h"

struct waypoint{
    Eigen::Vector3i idx;  // 格点坐标
    double cost;      // 从起点到该格点的总代价
    double heuristic; // 启发式函数的值
    waypoint* parent;  // 父节点
    double yaw = 0;
};

struct minimal_snap{
    
    minimal_snap(){}

    std::vector<waypoint*> waypoint_path;
    std::vector<waypoint*> simplified_waypoint_path;
    Eigen::MatrixXd _polyCoeff;     // 位置多项式
    Eigen::MatrixXd _polyCoeff_vel; // 速度多项式
    Eigen::MatrixXd _polyCoeff_acc; // 加速度多项式
    Eigen::VectorXd _polyTime;      // 时间分配（每段）

    double epsilon = 2;  // A star 简化阈值
    // 比较两个节点的总代价
    struct CompareWaypoint {
        bool operator()(const waypoint* a, const waypoint* b) const {
            return (a->cost + a->heuristic) > (b->cost + b->heuristic);
        }
    };

    std::vector<waypoint*> AStar(waypoint& start, waypoint& goal, GridMap_mini map) {
        std::vector<waypoint*> path;

        // 定义移动方向
        const int dx[] = {1, 1, 1, 0, 0, 0, -1, -1, -1, 1, 1, 1, 0,  0, -1, -1, -1, 1, 1, 1, 0, 0, 0, -1, -1, -1};
        const int dy[] = {1, 0, -1, 1, 0, -1, 1, 0, -1, 1, 0, -1, 1,  -1, 1, 0, -1, 1, 0, -1, 1, 0, -1, 1, 0, -1};
        const int dz[] = {1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0,  0, 0, 0, -1, -1, -1, -1, -1, -1, -1, -1, -1};

        const int x_length = map.max_index[0];
        const int y_length = map.max_index[1];
        const int z_length = map.max_index[2];

        // 创建数组来记录节点是否已经被访问
        std::vector<std::vector<std::vector<bool>>> visited(x_length, std::vector<std::vector<bool>>(y_length, std::vector<bool>(z_length, false)));

        // 创建数组来记录节点是否已经被add
        std::vector<std::vector<std::vector<bool>>> added(x_length, std::vector<std::vector<bool>>(y_length, std::vector<bool>(z_length, false)));

        // 创建优先队列用于节点扩展
        std::priority_queue<waypoint*, std::vector<waypoint*>, CompareWaypoint> openSet;

        // 初始化起始节点
        waypoint* startNode = new  waypoint{start.idx, 0, 0, nullptr};
        openSet.push(startNode);
        added[start.idx[0]][start.idx[1]][start.idx[2]] = true;

        if(start.idx==goal.idx){
            path.push_back(startNode);
            return path;
        }

        while (!openSet.empty()) {
            // 从优先队列中取出代价最小的节点
            waypoint* current = openSet.top();
            openSet.pop();

            // 如果当前节点是目标节点，构建并返回路径
            if (current->idx[0] == goal.idx[0] && current->idx[1] == goal.idx[1] && current->idx[2] == goal.idx[2]) {
                
                while (current != nullptr) {
                    path.push_back(current);
                    current = current->parent;
                }
                std::reverse(path.begin(),path.end());
                return path;
            }            
            visited[current->idx[0]][current->idx[1]][current->idx[2]] = true;

            // 扩展当前节点
            for (int i = 0; i < 26; i++) {
                int nextX = current->idx[0] + dx[i];
                int nextY = current->idx[1] + dy[i];
                int nextZ = current->idx[2] + dz[i];
                Eigen::Vector3i nextWaypoint(nextX,nextY,nextZ);

                // 检查是否越界或者是障碍物
                if (nextX < 0 || nextX >= x_length || nextY < 0 || nextY >= y_length || nextZ < 0 || nextZ >= z_length || map.grid_map[nextX][nextY][nextZ].is_occupied || map.grid_map[nextX][nextY][nextZ].is_inflated)
                    continue;

                // 如果节点已经访问过 or added，跳过
                if (visited[nextX][nextY][nextZ] || added[nextX][nextY][nextZ])
                    continue;

                // 计算下一个节点的代价和启发式函数值
                double nextCost = current->cost + std::pow(std::pow(nextX - current->idx[0],2) + std::pow(nextY - current->idx[1],2) + std::pow(nextZ - current->idx[2],2),0.5); 
                double heuristic = std::pow(std::pow(nextX - goal.idx[0],2) + std::pow(nextY - goal.idx[1],2) + std::pow(nextZ - goal.idx[2],2),0.5); 
                // 创建下一个节点
                waypoint* nextNode = new waypoint{nextWaypoint, nextCost, heuristic, current};

                openSet.push(nextNode);
                added[nextX][nextY][nextZ] = true;
            }
        }

        // // 如果无法找到路径，返回一个空路径
        // ROS_INFO("[%s]A* can not find valid waypoints to reach target!", name.c_str());
        return std::vector<waypoint*>();
    }

    // 计算点到直线的距离
    double pointToLineDistance(Eigen::Vector3d A, Eigen::Vector3d B, Eigen::Vector3d P) {
        // 计算向量 AB 和 AP
        double ABx = B[0] - A[0];
        double ABy = B[1] - A[1];
        double ABz = B[2] - A[2];

        double APx = P[0] - A[0];
        double APy = P[1] - A[1];
        double APz = P[2] - A[2];

        // 计算法向量 N
        double Nx = ABy * APz - ABz * APy;
        double Ny = ABz * APx - ABx * APz;
        double Nz = ABx * APy - ABy * APx;

        // 计算法向量 N 的模
        double N_length = std::sqrt(Nx * Nx + Ny * Ny + Nz * Nz);

        // 计算点到直线的距离
        double AB_length = std::sqrt(ABx * ABx + ABy * ABy + ABz * ABz);

        // 避免除以零的情况
        if (AB_length == 0.0) {
            throw std::invalid_argument("The length of line segment AB is zero.");
        }

        double distance = N_length / AB_length;

        return distance;
    }


    // 执行 Douglas-Peucker 算法 精简A* 不然中间点过多一卡一卡
    std::vector<waypoint *> douglasPeucker(std::vector<waypoint *> points, double epsilon) {
        std::vector<waypoint *> simplified;

        if (points.size() < 3) {
            return points;
        }

        // 寻找最远点
        double maxDistance = 0.0;
        size_t maxIndex = 0;

        waypoint * start = points.front();
        waypoint * end = points.back();

        for (size_t i = 1; i < points.size() - 1; ++i) {
            double d = pointToLineDistance(grid_map.index2pos(start->idx), grid_map.index2pos(end->idx), grid_map.index2pos(points[i]->idx));
            if (d > maxDistance) {
                maxDistance = d;
                maxIndex = i;
            }
        }

        // 如果最大距离大于阈值，则保留最远点，否则继续递归简化
        if (maxDistance > epsilon) {
            std::vector<waypoint *> firstPart(points.begin(), points.begin() + maxIndex + 1);
            std::vector<waypoint *> secondPart(points.begin() + maxIndex, points.end());
            std::vector<waypoint *> simplifiedFirst, simplifiedSecond;

            simplifiedFirst = douglasPeucker(firstPart, epsilon);
            simplifiedSecond = douglasPeucker(secondPart, epsilon);

            simplified.insert(simplified.end(), simplifiedFirst.begin(), simplifiedFirst.end() - 1);
            simplified.insert(simplified.end(), simplifiedSecond.begin(), simplifiedSecond.end());
        } else {
            simplified.clear();
            simplified.push_back(start);
            simplified.push_back(end);
        }
        return simplified;
    }


    // 找到trajectory不安全的首个点距离精简前的waypoints中最近的那一个 然后加进去
    std::vector<waypoint *> correct_trajectory(){
        Eigen::Vector3d unsafe_point;
        waypoint * add_point;
        std::vector<waypoint *> added_path;
        bool already_in = false;
        for(int i=0;i<_polyTime.size();i++){
            for(double t=0;t<=_polyTime[i];t+=0.1){
                unsafe_point = getPosPoly(_polyCoeff,i,t);
                if(grid_map.get_occupancy_pos(unsafe_point) || grid_map.get_occupancy_pos(unsafe_point,true)){
                    // std::cout<<"trajectory unsafe" << std::endl;
                    if(simplified_waypoint_path.size()==waypoint_path.size()){
                        // std::cout<<"simplified_waypoint_path have alread become waypoint_path" << std::endl;
                    }
                    double min_dis = DBL_MAX;
                    for(auto waypoint:waypoint_path){
                        double tmp = (grid_map.index2pos(waypoint->idx)-unsafe_point).norm();
                        if(tmp<min_dis){
                            for(auto s_waypoint:simplified_waypoint_path){
                                if(waypoint->idx==s_waypoint->idx){
                                    already_in = true;
                                    break;
                                }
                            }
                            if(already_in){
                                already_in = false;
                                continue;
                            }
                            min_dis = tmp;
                            add_point = waypoint;
                        }
                    }
                    // 保证顺序
                    for(auto waypoint:waypoint_path){
                        for(auto s_waypoint:simplified_waypoint_path){
                            if(waypoint->idx==s_waypoint->idx){
                                added_path.push_back(waypoint);
                                break;
                            }
                        }
                        if(waypoint->idx == add_point->idx) added_path.push_back(waypoint);
                            
                    }
                    return added_path;//为什么这里就return了
                }
            }
        }
        return added_path;
    }


    bool generate_trajetory(std::vector<waypoint *> waypoints){
        if(waypoints.size()==0){
            // ROS_INFO("[%s] error!  A* waypoints is empty",name.c_str());   
            return false;      
        }
        // 当前点即为目标点
        if(waypoints.size()==1){
            waypoints.push_back(waypoints[0]);
        }
        // 简化A*路点
        try{
            simplified_waypoint_path = douglasPeucker(waypoints,epsilon);
        }catch (const std::invalid_argument& e) {
            // std::cerr << "Error: " << e.what() << std::endl;
        }
        // // 两个点的直线snap也能跑歪 
        // if(simplified_waypoint_path.size()==2){
        //     waypoint* tmp = new waypoint;
        //     tmp->idx = (simplified_waypoint_path[0]->idx + simplified_waypoint_path[1]->idx)/2; 
        //     simplified_waypoint_path.insert(simplified_waypoint_path.begin() + 1, tmp);
        // }
        Eigen::MatrixXd waypoint_path_matrix(simplified_waypoint_path.size(), 3); 
        for(int k = 0; k < simplified_waypoint_path.size(); k++){
            waypoint_path_matrix.row(k) = Eigen::Vector3d(grid_map.index2pos(simplified_waypoint_path[k]->idx)[0],
                                                        grid_map.index2pos(simplified_waypoint_path[k]->idx)[1],
                                                        std::max(0.2,grid_map.index2pos(simplified_waypoint_path[k]->idx)[2]));
        }
        trajGeneration(waypoint_path_matrix,_polyCoeff,_polyTime);
        std::vector<waypoint *> correct_waypoint_path = correct_trajectory();
        // 如果traj不合法 尝试去对精简后的A*路点集添加原来的路点来使traj更贴近原始A*路点的轨迹
        while(correct_waypoint_path.size()!=0){
            // 即便是原始A*路点也不行 那就放弃这个目标
            if(simplified_waypoint_path.size()==correct_waypoint_path.size()){
                // ROS_INFO("[%s] can not find safe trajectory",name.c_str());  
                return false;
            }
            simplified_waypoint_path=correct_waypoint_path;
            waypoint_path_matrix = Eigen::MatrixXd(simplified_waypoint_path.size(), 3); 
            for(int k = 0; k < simplified_waypoint_path.size(); k++){
            waypoint_path_matrix.row(k) = Eigen::Vector3d(grid_map.index2pos(simplified_waypoint_path[k]->idx)[0],
                                                        grid_map.index2pos(simplified_waypoint_path[k]->idx)[1],
                                                        std::max(0.2,grid_map.index2pos(simplified_waypoint_path[k]->idx)[2]));
            }
            trajGeneration(waypoint_path_matrix,_polyCoeff,_polyTime);
            std::vector<waypoint *> correct_waypoint_path = correct_trajectory();
        }

        _polyCoeff_vel = calculate_polycoeff_de(_polyCoeff);
        _polyCoeff_acc = calculate_polycoeff_de(_polyCoeff_vel);
        return true;
    }
};