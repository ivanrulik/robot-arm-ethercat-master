#include <iostream>
//#include <Eigen/Dense>
#include "WMAR.h"

int main(){
    WMAR Robot;
    double theta[6] = {0, 1.5707, 1.5707/2, 0, 1.5707/2, 1.5707};
    std::cout << "theta\t" << theta[0] << "\t" << theta[1] << "\t" << theta[2] << "\t" << theta[3] << "\t" << theta[4] << "\t" << theta[5] << "\n";
    std::cout << "\n";

    double FW[6];
    Robot.ForwardKinematics( theta, FW );
    std::cout << "FW\t" << FW[0] << "\t" << FW[1] << "\t" << FW[2] << "\t" << FW[3] << "\t" << FW[4] << "\t" << FW[5] << "\n";
    std::cout << "\n";

    double theta_ik[6];
    bool valid = Robot.InverseKinematics( FW, theta_ik );
    std::cout << "theta_ik\t" << theta_ik[0] << "\t" << theta_ik[1] << "\t" << theta_ik[2] << "\t" << theta_ik[3] << "\t" << theta_ik[4] << "\t" << theta_ik[5] << "\n";
    std::string response = (valid)? "Yes" : "No";
    std::cout << "could it calculate inverse kinematics? " << response << "\n";
    std::cout << "\n";

    double FW_ik[6];
    Robot.ForwardKinematics( theta_ik, FW_ik );
    std::cout << "FW_ik\t" << FW_ik[0] << "\t" << FW_ik[1] << "\t" << FW_ik[2] << "\t" << FW_ik[3] << "\t" << FW_ik[4] << "\t" << FW_ik[5] << "\n";
    std::cout << "\n";
    return 0;
}