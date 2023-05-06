#include "Motor.h"
#include <Eigen/Dense>
#include <math.h>  
using Eigen::MatrixXd;
using Eigen::VectorXd;
using namespace std;
class Robot
{
    public:
        Robot():joint1{(uint16) 1, 1},
                joint2{(uint16) 2, 2}
        {
            joint1.ConfigurePDOs();
            joint2.ConfigurePDOs();
            Motor::ConfigureMotors();
            Motor::ReadEthercat();
        }
    private:
        Motor joint1;
        Motor joint2;
        int actualJointPosition[2];
        double timePeriod = 740; //microseconds
    public:
        // Configure the profile position mode (mode = 1)
        //
        void ConfigureProfilePositionMode(){
            joint1.ConfigureProfilePositionMode();
            joint2.ConfigureProfilePositionMode();
            this->actualJointPosition[0] = joint1.GetPositionActualValue(); 
            this->actualJointPosition[1] = joint2.GetPositionActualValue();
        }
        // Configure the cylcic synchronous position mode (actualy deprecated)
        void ConfigureCyclicSynchronousPositionMode(){
            joint1.ConfigureCyclicSynchronousPositionMode();
            joint2.ConfigureCyclicSynchronousPositionMode();
            this->actualJointPosition[0] = joint1.GetPositionActualValue(); 
            this->actualJointPosition[1] = joint2.GetPositionActualValue();
        }
        // inputs: velocityAverage in Rad per Sec
        void ProfilePositionControl(int jointFinalPosition[], int velocityAverage = 20){
            velocityAverage = velocityAverage * 764; //Pulses/s
            int velocityAverageArray[2];
            int accelerationArray[2];
            int velocityMaxArray[2];
            int deltaJointArray[2];
            double deltaTime;
            for (int i = 0; i < 2; ++i){
                deltaJointArray[i] = abs(jointFinalPosition[i] - this->actualJointPosition[i]);
            }
            cout<<"Actual Joint1: "<<dec<<((int) actualJointPosition[0])<<endl;
            cout<<"Actual Joint2: "<<dec<<((int) actualJointPosition[1])<<endl;
            cout<<"Final Joint1: "<<dec<<((int) jointFinalPosition[0])<<endl;
            cout<<"Final Joint2: "<<dec<<((int) jointFinalPosition[1])<<endl;
            cout<<"Delta Joint1: "<<dec<<((int) deltaJointArray[0])<<endl;
            cout<<"Delta Joint2: "<<dec<<((int) deltaJointArray[1])<<endl;
            int maximumDisplacement = *max_element(deltaJointArray , deltaJointArray + 2);
            deltaTime = (double) maximumDisplacement / (double) velocityAverage; 
            for (int j = 0; j < 2; ++j){
                velocityAverageArray[j] = (int)(((double) deltaJointArray[j])/deltaTime);
                accelerationArray[j] = (int) (4*pow(velocityAverageArray[j], 2) / deltaJointArray[j] + 1000);
                velocityMaxArray[j] = (int) -(sqrt(accelerationArray[j]*deltaJointArray[j]*(- 4*pow(velocityAverageArray[j], 2) + accelerationArray[j]*deltaJointArray[j])) -
                                             accelerationArray[j]*deltaJointArray[j])/(2*velocityAverageArray[j]);
            }
            cout<<"Velocity Avg Joint1: "<<dec<<((uint32_t) (velocityAverageArray[0]*0.0125))<<endl;
            cout<<"Velocity Avg Joint2: "<<dec<<((uint32_t) (velocityAverageArray[1]*0.0125))<<endl;
            joint1.SetProfileAcceleration((uint32_t) (accelerationArray[0]*0.0125));
            joint2.SetProfileAcceleration((uint32_t) (accelerationArray[1]*0.0125));
            joint1.SetProfileDeceleration((uint32_t) (accelerationArray[0]*0.0125));
            joint2.SetProfileDeceleration((uint32_t) (accelerationArray[1]*0.0125));
            cout<<"Acceleration Joint1: "<<dec<<((uint32_t) (accelerationArray[0]*0.0125))<<endl;
            cout<<"Acceleration Joint2: "<<dec<<((uint32_t) (accelerationArray[1]*0.0125))<<endl;
            joint1.SetMaxProfileVelocity((uint32_t) (velocityMaxArray[0]*0.0125));
            joint2.SetMaxProfileVelocity((uint32_t) (velocityMaxArray[1]*0.0125));
            cout<<"Velocity Joint1: "<<dec<<((uint32_t) (velocityMaxArray[0]*0.0125))<<endl;
            cout<<"Velocity Joint2: "<<dec<<((uint32_t) (velocityMaxArray[1]*0.0125))<<endl;
            joint1.SetTargetPosition((int32_t) jointFinalPosition[0]);
            joint2.SetTargetPosition((int32_t) jointFinalPosition[1]);
            joint1.SetControlWord((uint16_t) 0x003f);
            joint2.SetControlWord((uint16_t) 0x003f);
            int cont = 3;
            do {
                for(int i = 0; i<cont; ++i){
                    Motor::ReadEthercat();
                    joint1.WritingPDOs();
                    joint1.ReadingPDOs();
                    joint2.WritingPDOs();
                    joint2.ReadingPDOs();
                    Motor::WriteEthercat();
                    needlf = TRUE;
                    osal_usleep(1000);
                }
                cont = 1;
                } 
                while ((joint1.GetTargetPosition() != joint1.GetPositionActualValue()) ||
                       (joint2.GetTargetPosition() != joint2.GetPositionActualValue()));
            joint1.SetControlWord((uint16_t) 0x000f);
            joint2.SetControlWord((uint16_t) 0x000f);
            cont = 3;
            do {
                for(int i = 0; i<cont; ++i){
                    Motor::ReadEthercat();
                    joint1.WritingPDOs();
                    joint1.ReadingPDOs();
                    joint2.WritingPDOs();
                    joint2.ReadingPDOs();
                    Motor::WriteEthercat();
                    needlf = TRUE;
                    osal_usleep(800);                }
            } 
            while (!((joint1.GetStatusWord() == 4663) || (joint1.GetStatusWord() == 1591)) ||
                   !((joint2.GetStatusWord() == 4663) || (joint2.GetStatusWord() == 1591)));
            this->actualJointPosition[0] = joint1.GetPositionActualValue(); 
            this->actualJointPosition[1] = joint2.GetPositionActualValue();
        }

        /*
            deltaTime in microSeconds;
            jointFinalPosition in pulses;
            motorTorques in percentage;
        */
        void CyclicSynchronousPositionControl(int jointFinalPositions[], int motorTorques[], double deltaTime){
            VectorXd coefficientsJoint1 = this->QuinticCoefficients(this->actualJointPosition[0], jointFinalPositions[0], deltaTime);
            VectorXd coefficientsJoint2 = this->QuinticCoefficients(this->actualJointPosition[1], jointFinalPositions[1], deltaTime);
            double intermediatePositionJoint1;
            double intermediatePositionJoint2;
            int sequence = (deltaTime + timePeriod)/timePeriod;
            VectorXd timeArray = VectorXd::LinSpaced(sequence, 0, deltaTime);
            auto start = high_resolution_clock::now();
            auto stop = high_resolution_clock::now();
            auto duration = duration_cast<microseconds>(stop - start);
            int durationInteger;
            for (int i = 0; i < sequence; i++){
                start = high_resolution_clock::now();
                VectorXd timeProductArray{
                    {1},
                    {timeArray(i)},
                    {pow(timeArray(i), 2)},
                    {pow(timeArray(i), 3)},
                    {pow(timeArray(i), 4)},
                    {pow(timeArray(i), 5)}
                };
                intermediatePositionJoint1 = timeProductArray.transpose()*coefficientsJoint1;
                intermediatePositionJoint2 = timeProductArray.transpose()*coefficientsJoint2;
                joint1.SetTargetPosition(intermediatePositionJoint1);
                joint2.SetTargetPosition(intermediatePositionJoint2);
                durationInteger = 0;
                while (durationInteger < this->timePeriod){
                    Motor::ReadEthercat();
                    joint1.WritingPDOs();
                    joint1.ReadingPDOs();
                    joint2.WritingPDOs();
                    joint2.ReadingPDOs();
                    Motor::WriteEthercat();
                    needlf = TRUE;
                    osal_usleep(770);  
                    stop = high_resolution_clock::now();
                    duration = duration_cast<microseconds>(stop - start);
                    durationInteger = duration.count();
                }
            }
            this->actualJointPosition[0] = joint1.GetPositionActualValue(); 
            this->actualJointPosition[1] = joint2.GetPositionActualValue();
        }

    private:
        VectorXd QuinticCoefficients(double initialPosition, double finalPosition, double deltaTime){
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
            return extendedCoefficients;
        }
};