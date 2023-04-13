/**
 * @file WMAR.h
 * @author Elias Munoz (eliasm@uwm.edu)
 * @brief Implementation of the kinematics for the 6-DOF WMAR of the mR2A 
 * project. It includes the forward kinematics, inverse kinematics, and OBB
 * object representation.
 * @details It incorporates a class, called WMAR, with the kinematics as 
 * functions.
 * @date 12/01/2022
 * @copyright Copyright (c) 2022
**/

#include <vector>
#include <cmath>
#include <Eigen/Dense>

class WMAR
{
    public:
        double L[6];

        WMAR(){
            // double L[6] = {170, 440.0, 450.0, 100.0, 140.0, 0.0};
            this->L[0] = 170;
            this->L[1] = 440;
            this->L[2] = 450;
            this->L[3] = 100;
            this->L[4] = 140;
            this->L[5] =   0;
        }

    private:
        Eigen::MatrixXf HomogeneousMatrix( double alpha, double a, double d, double theta ){
            Eigen::MatrixXf Tij(4, 4);

            Tij(0,0) = std::cos(theta);
            Tij(1,0) = std::cos(alpha) * std::sin(theta);
            Tij(2,0) = std::sin(alpha) * std::sin(theta);
            Tij(3,0) = 0;

            Tij(0,1) = - std::sin(theta);
            Tij(1,1) = std::cos(alpha) * std::cos(theta);
            Tij(2,1) = std::sin(alpha) * std::cos(theta);
            Tij(3,1) = 0;

            Tij(0,2) = 0;
            Tij(1,2) = - std::sin(alpha);
            Tij(2,2) = std::cos(alpha);
            Tij(3,2) = 0;

            Tij(0,3) = a;
            Tij(1,3) = - d* std::sin(alpha);
            Tij(2,3) = d * std::cos(alpha);
            Tij(3,3) = 1;
            
            return Tij;
        } 
        

        Eigen::MatrixXf EulXYZ2Rotm(double EulX, double EulY, double EulZ){
            Eigen::MatrixXf RotX(4,4);
            RotX(0, 0) = 1; RotX(0, 1) = 0;                 RotX(0, 2) = 0;                 RotX(0, 3) = 0;
            RotX(1, 0) = 0; RotX(1, 1) = std::cos(EulX);    RotX(1, 2) = -std::sin(EulX);   RotX(1, 3) = 0;
            RotX(2, 0) = 0; RotX(2, 1) = std::sin(EulX);    RotX(2, 2) = std::cos(EulX);    RotX(2, 3) = 0;
            RotX(3, 0) = 0; RotX(3, 1) = 0;                 RotX(3, 2) = 0;                 RotX(3, 3) = 1;
            
            Eigen::MatrixXf RotY(4,4);
            RotY(0, 0) = std::cos(EulY);    RotY(0, 1) = 0; RotY(0, 2) = std::sin(EulY);    RotY(0, 3) = 0;
            RotY(1, 0) = 0;                 RotY(1, 1) = 1; RotY(1, 2) = 0;                 RotY(1, 3) = 0;
            RotY(2, 0) = -std::sin(EulY);   RotY(2, 1) = 0; RotY(2, 2) = std::cos(EulY);    RotY(2, 3) = 0;
            RotY(3, 0) = 0;                 RotY(3, 1) = 0; RotY(3, 2) = 0;                 RotY(3, 3) = 1;
            
            Eigen::MatrixXf RotZ(4,4);
            RotZ(0, 0) = std::cos(EulZ);    RotZ(0, 1) = -std::sin(EulZ);   RotZ(0, 2) = 0; RotZ(0, 3) = 0;
            RotZ(1, 0) = std::sin(EulZ);    RotZ(1, 1) = std::cos(EulZ);    RotZ(1, 2) = 0; RotZ(1, 3) = 0;
            RotZ(2, 0) = 0;                 RotZ(2, 1) = 0;                 RotZ(2, 2) = 1; RotZ(2, 3) = 0;
            RotZ(3, 0) = 0;                 RotZ(3, 1) = 0;                 RotZ(3, 2) = 0; RotZ(3, 3) = 1;

            return RotX * RotY * RotZ;
        }

        Eigen::MatrixXf Rotm2EulXYZ( const Eigen::Ref<const Eigen::MatrixXf>& Tij ){     
            Eigen::MatrixXf Eul(1,3);
            Eul(0,1) = std::asin( Tij(0,2) );
            if( abs(Tij(0,2) - 1) > 1e-3 ){
                if( abs(Tij(1,2)) > 1e-3 && abs(Tij(2,2)) > 1e-3 ){
                    Eul(0,0) = std::atan2( -Tij(1,2), Tij(2,2) );
                }
                if( abs(Tij(0,0)) > 1e-3 && abs(Tij(0,1)) > 1e-3 ){
                    Eul(0,2) = std::atan2( -Tij(0,1), Tij(0,0) );
                }
            } else {
                Eul(0,0) = std::atan2( Tij(1,0), Tij(1,1) );
                Eul(0,2) = 0;
            }
            Eul(0,0) = std::fmod( Eul(0,0),  M_PI*2 );
            Eul(0,1) = std::fmod( Eul(0,1),  M_PI*2 );
            Eul(0,2) = std::fmod( Eul(0,2),  M_PI*2 );
            return Eul;
        }

        Eigen::MatrixXf T01_f(double theta1){
            return HomogeneousMatrix(0, 0, this->L[0], theta1);
        }

        Eigen::MatrixXf T12_f(double theta2){
            return HomogeneousMatrix(- M_PI_2, 0, 0, theta2 - M_PI_2);
        }

        Eigen::MatrixXf T23_f(double theta3){
            return HomogeneousMatrix( M_PI, this->L[1], 0, theta3 - M_PI_2);
        }

        Eigen::MatrixXf T34_f(double theta4){
            return HomogeneousMatrix(- M_PI_2, 0, this->L[2] + this->L[3], theta4);
        }

        Eigen::MatrixXf T45_f(double theta5){
            return HomogeneousMatrix(- M_PI_2, 0, 0, theta5);
        }

        Eigen::MatrixXf T56_f(double theta6){
            return HomogeneousMatrix( M_PI_2, 0, this->L[4] + this->L[5], theta6);
        }

    public:
        void ForwardKinematics(double theta[6], double* FW){
            Eigen::MatrixXf T06 = this->T01_f(theta[0]) * this->T12_f(theta[1]) * this->T23_f(theta[2]) * this->T34_f(theta[3]) * this->T45_f(theta[4]) * this->T56_f(theta[5]);
            // std::cout << T06 << "\n";
            Eigen::MatrixXf Eul = Rotm2EulXYZ( T06 );
            FW[0] = T06(0,3); FW[1] = T06(1,3); FW[2] = T06(2,3);
            FW[3] = Eul(0,0); FW[4] = Eul(0,1); FW[5] = Eul(0,2);
            // return FW
        }

        bool InverseKinematics( double FW[6], double* theta ){
            Eigen::MatrixXf T06 = this->EulXYZ2Rotm( FW[3], FW[4], FW[5] );
            T06(0,3) = FW[0]; T06(1,3) = FW[1]; T06(2,3) = FW[2];
            // std::cout << T06 << "\n";
            Eigen::MatrixXf PEE = T06.col(3) - (this->L[4] + this->L[5]) * T06.col(2);
            theta[0] = std::atan2( PEE(1,0), PEE(0,0) );
            theta[0] = std::fmod(theta[0], M_PI);
            
            Eigen::MatrixXf TEE = this->T01_f(theta[0]);
            PEE = TEE.inverse() * PEE;
            double h = std::pow(PEE(0,0),2) + std::pow(PEE(2,0),2);
            double a = ( std::pow(this->L[1],2) + h - std::pow(this->L[2]+this->L[3],2) )/( 2*this->L[1]*std::sqrt(h) );
            if( std::abs( a ) < 1 ){
                theta[1] = M_PI_2 - std::atan2( PEE(2,0), PEE(0,0) ) - std::acos( a );
            } else if ( std::abs( std::abs( a ) - 1) < 1e-3 ){
                theta[1] = M_PI_2 - std::atan2( PEE(2,0), PEE(0,0) ) - std::acos( a/std::abs( a ) );
            }else{
                return false;
            }
            
            TEE = this->T12_f(theta[1]);
            PEE = TEE.inverse() * PEE;
            theta[2] = std::atan2( -PEE(1,0), PEE(0,0) - this->L[1] );

            TEE = this->T01_f(theta[0]) * this->T12_f(theta[1]) * this->T23_f(theta[2]);
            PEE = TEE.inverse() * T06.col(3);
            theta[3] = std::atan2( -PEE(2,0), PEE(0,0) );

            TEE = this->T34_f(theta[3]);
            PEE = TEE.inverse() * PEE;
            theta[4] = std::atan2( PEE(0,0), PEE(2,0) );

            theta[5] = 0;
            TEE = this->T01_f(theta[0]) * this->T12_f(theta[1]) * this->T23_f(theta[2]) * this->T34_f(theta[3]) * this->T45_f(theta[4]);
            PEE = TEE.inverse() * T06;
            theta[5] = std::atan2( PEE(2,0), PEE(0,0) );
            
            // std::cout << PEE << "\n";
            // return theta;
            return true;
        }
};