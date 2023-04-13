#include <iostream>
#include <Eigen/Dense>
#include <math.h>  

using Eigen::MatrixXd;
using Eigen::VectorXd;
int main()
{
  double finalPosition = -40;
  double initialPosition = -50;
  double deltaTime = 10.5;
  double deltaPosition = finalPosition - initialPosition;
  MatrixXd a{
      {pow(deltaTime, 3), pow(deltaTime, 4), pow(deltaTime, 5)},
      {3*pow(deltaTime, 2), 4*pow(deltaTime, 3), 5*pow(deltaTime, 4)},
      {6*deltaTime, 12*pow(deltaTime, 2), 20*pow(deltaTime, 3)}
  };  
  MatrixXd b{
      {deltaPosition},
      {0},
      {0}
  };
  MatrixXd coefficients = a.colPivHouseholderQr().solve(b); 
  VectorXd extendedCoefficients{
      {initialPosition},
      {0},
      {0},
      {coefficients(0)},
      {coefficients(1)},
      {coefficients(2)}
  };
  double timePeriod = 1; //1 miliseconds 
  int sequence = (deltaTime + timePeriod)/timePeriod;
  VectorXd timeArray = VectorXd::LinSpaced(sequence, 0, deltaTime);
  for (int i = 0; i < sequence; i++){
      VectorXd timeProductArray{
          {1},
          {timeArray(i)},
          {pow(timeArray(i), 2)},
          {pow(timeArray(i), 3)},
          {pow(timeArray(i), 4)},
          {pow(timeArray(i), 5)}
      };
      std::cout << timeProductArray.transpose() * extendedCoefficients << std::endl;
      std::cout << "**************" << std::endl;
  }
}