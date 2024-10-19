#ifndef _TRAJECTORY_GENERATOR_H_
#define _TRAJECTORY_GENERATOR_H_

#include <Eigen/Eigen>
#include <vector>

class TrajectoryGeneratorTool {
private:
    int Factorial(int x);

public:
    TrajectoryGeneratorTool() = default;

    ~TrajectoryGeneratorTool() = default;

    Eigen::MatrixXd SolveQPClosedForm(
            int order,
            const Eigen::MatrixXd &Path,
            const Eigen::MatrixXd &Vel,
            const Eigen::MatrixXd &Acc,
            const Eigen::VectorXd &Time);
};

Eigen::Vector3d getPosPoly(Eigen::MatrixXd polyCoeff, int k, double t);
Eigen::VectorXd timeAllocation(Eigen::MatrixXd Path);
void trajGeneration(Eigen::MatrixXd path,Eigen::MatrixXd& _polyCoeff,Eigen::VectorXd& _polyTime);
Eigen::MatrixXd calculate_polycoeff_de(Eigen::MatrixXd polyCoeff);
Eigen::Vector3d getVelPoly( Eigen::MatrixXd polyCoeff_vel, int k, double t );
Eigen::Vector3d getAccPoly( Eigen::MatrixXd polyCoeff_acc, int k, double t );
#endif
