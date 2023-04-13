/*
    Author: Javier Dario Sanjuan De Caro
    email: jsanjuan@uwm.edu or jdecaro@uninorte.edu.co
    
    Motor class
    
    This code is used to configured the Maxon motors and to run their different operation modes
    using PDO communication. 

    The code requires the 
*/
#include "ODVariable.h"
#include <iostream>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <inttypes.h>
#include <stdlib.h>
#include <unistd.h>
#include <chrono>
using namespace std;
using namespace std::chrono;

static boolean needlf;
enum ReadingVariables{
  STATUS_WORD,  
  POSITION_ACTUAL_VALUE,
  MODE_OF_OPERATION_DISPLAY,
  POSITION_DEMAND_VALUE,
  TORQUE_ACTUAL_VALUE,
};
enum WritingVariables{
  CONTROL_WORD,
  TARGET_TORQUE,
  MODE_OF_OPERATION,
  TARGET_POSITION,
  MAX_PROFILE_VELOCITY,
  PROFILE_ACCELERATION,
  PROFILE_DECELERATION,
};
class Motor
{ 
    private:
        uint16_t slave;
        int gearRatio;
        int incrementPerRevolution;
        int stallTorque;
        /* Reading Variables TPDOs*/
        int32_t positionActualValue{0};
        uint16_t statusWord{0};
        int8_t modeOfOperationDisplay{0};
        int32_t positionDemandValue{0};
        int16_t torqueActualValue{0}; 
        static const int arrayReadingSize{5};
        int TPDOBytes[arrayReadingSize]{2, 4, 1, 4, 2};

        /* Writing Variables RPDOs*/
        uint16_t controlWord{0};
        int16_t targetTorque{0};
        int8_t modeOfOperation{0};
        int32_t targetPosition{0};
        uint32_t maxProfileVelocity{0};
        uint32_t profileAcceleration{0};
        uint32_t profileDeceleration{0};
        static const int arrayWritingSize{7};
        int RPDOBytes[arrayWritingSize]{2, 2, 1, 4, 4, 4, 4};
        static int usedMemory;
        static int expectedWKC;
    public:
        Motor(uint16_t slave, int stallTorque, int gearRatio = 100, int incrementPerRevolution = 4800): syncManager2{(uint16) slave, (uint16) 0x1C12, UNSIGNED8},
                                                                                                      syncManager3{(uint16) slave, (uint16) 0x1C13, UNSIGNED8},
                                                                                                      rxPDO1{(uint16) slave, (uint16) 0x1600},
                                                                                                      txPDO1{(uint16) slave, (uint16) 0x1a00}
        {
            this->slave = slave;
            this->gearRatio = gearRatio;
            this->incrementPerRevolution = incrementPerRevolution;
            this->stallTorque = stallTorque;
        }
    private:
        ODVariable syncManager2;
        ODVariable syncManager3;
        ODVariable rxPDO1;
        ODVariable txPDO1;
    public:
        void ConfigurePDOs(){
            // Consider adding more rxPDOs/txPDOs 
            this->syncManager2.SDOwrite(0, (uint8) 0x0000, UNSIGNED8); //set index 0 to 0 so rxPDO1 can be configured
            this->syncManager3.SDOwrite(0, (uint8) 0x0000, UNSIGNED8); //set index 0 to 0 so txPDO1 can be configured
            this->rxPDO1.SDOwrite(0, 0x0000, UNSIGNED8); // set index 0 to 0 so the other indices can be added
            //value = index(xxxx)+subindex(xx)+bits(xx)
            this->rxPDO1.SDOwrite(0x60400010, 0x0001, UNSIGNED32); //0x6040 := Controlword; type := uint16; bytes := 2;
            this->rxPDO1.SDOwrite(0x60710010, 0x0002, UNSIGNED32); //0x6071 := Target torque; type := int16; bytes := 2; 
            this->rxPDO1.SDOwrite(0x60600008, 0x0003, UNSIGNED32); //0x6060 := Modes of operation; type := int8; bytes := 1;
            this->rxPDO1.SDOwrite(0x607A0020, 0x0004, UNSIGNED32); //0x607A := Target position; type := int32; bytes := 4;
            this->rxPDO1.SDOwrite(0x607F0020, 0x0005, UNSIGNED32); //0x607F := Max profile velocity; type := uint32; bytes := 4;
            this->rxPDO1.SDOwrite(0x60830020, 0x0006, UNSIGNED32); //0x6083 := Profile acceleration; type := uint32; bytes := 4;
            this->rxPDO1.SDOwrite(0x60840020, 0x0007, UNSIGNED32); //0x6084 := Profile deceleration; type := uint32; bytes := 4;
            this->rxPDO1.SDOwrite(0x07, 0x0000, UNSIGNED8);// set index 0 to 7 (number of variables in PDO)
            this->txPDO1.SDOwrite(0, 0x0000, UNSIGNED8);
            this->txPDO1.SDOwrite(0x60410010, 0x0001, UNSIGNED32); //0x6041 := Statusword; type := uint16; bytes := 2;
            this->txPDO1.SDOwrite(0x60640020, 0x0002, UNSIGNED32); //0x6064 := Position actual value; type := int32; bytes := 4
            this->txPDO1.SDOwrite(0x60610008, 0x0003, UNSIGNED32); //0x6061 := Modes of operation display; type := int8; bytes := 1
            this->txPDO1.SDOwrite(0x60620020, 0x0004, UNSIGNED32); //0x6064 := Position demand value; type := int32; bytes := 4
            this->txPDO1.SDOwrite(0x60770010, 0x0005, UNSIGNED32); //0x6077 := Torque actual value; type := int16; bytes := 2;
            this->txPDO1.SDOwrite(0x05, 0x0000, UNSIGNED8);
            this->syncManager2.SDOwrite(0x1600, 0x0001, UNSIGNED16); //set subindex 1 with the value of the rxPDO1 in hexa 
            this->syncManager2.SDOwrite(1, 0x0000, UNSIGNED8); //Activate syncManager
            this->syncManager3.SDOwrite(0x1a00, 0x0001, UNSIGNED16);
            this->syncManager3.SDOwrite(1, 0x0000, UNSIGNED8); 
        }
        static void ConfigureMotors(){
            char IOmap[4096];
            static int usedMemory = ec_config_map(IOmap);
            static int expectedWKC = (ec_group[0].outputsWKC * 2) + ec_group[0].inputsWKC;
            ec_configdc();
            ec_statecheck(0, EC_STATE_SAFE_OP,  EC_TIMEOUTSTATE * 4); 
            ec_slave[0].state = EC_STATE_OPERATIONAL;
            ec_send_processdata();
            ec_receive_processdata(EC_TIMEOUTRET);
            ec_writestate(0); /* request OP state for the motor */
            int chk = 200;
            do
            {
                ec_send_processdata();
                ec_receive_processdata(EC_TIMEOUTRET);
                ec_statecheck(0, EC_STATE_OPERATIONAL, 50000);
            }
            while (chk-- && (ec_slave[0].state != EC_STATE_OPERATIONAL));   
        }
        static void ReadEthercat(){
            ec_receive_processdata(EC_TIMEOUTRET);
        }
        static void WriteEthercat(){
            ec_send_processdata();
        }
    public:
        void ReadingPDOs(){
            int startByte = 0;
            int lastByte = 0;
            int initWriting = 0;
            uint8_t *readingData;
            readingData = ec_slave[this->slave].inputs;
            int count;
            this->statusWord = 0;
            this->positionActualValue = 0;
            this->modeOfOperationDisplay = 0;
            this->positionDemandValue = 0;
            this->torqueActualValue = 0;
            for (int j = 0; j < this->arrayReadingSize; j++){
                lastByte += this->TPDOBytes[j];
                count = 0;
                for (int k = startByte; k < lastByte; k++){
                    switch(j){
                        case STATUS_WORD: {
                            this->statusWord |= (readingData[k] <<  8*count) & (0xFF << 8*count);
                        } break;
                        case POSITION_ACTUAL_VALUE:{
                            this->positionActualValue |= (readingData[k] <<  8*count) & (0xFF << 8*count);
                        } break;
                        case MODE_OF_OPERATION_DISPLAY:{
                            this->modeOfOperationDisplay |= (readingData[k] <<  8*count) & (0xFF << 8*count);
                        } break;
                        case POSITION_DEMAND_VALUE:{
                            this->positionDemandValue |= (readingData[k] <<  8*count) & (0xFF << 8*count);
                        } break;
                        case TORQUE_ACTUAL_VALUE:{
                            this->torqueActualValue |= (readingData[k] <<  8*count) & (0xFF << 8*count);
                        } break;
                    }
                    count++;
                }
                startByte += this->TPDOBytes[j];
            }
        }
        void WritingPDOs(){
            int startByte = 0;
            int lastByte = 0;
            int count;
            uint8_t *writingData;
            writingData = ec_slave[this->slave].outputs;
            for (int k = 0; k < this->arrayWritingSize; k++){
                count = 0;
                lastByte += this->RPDOBytes[k];
                for (int jj = startByte; jj < lastByte; jj++){
                    switch(k){
                        case CONTROL_WORD:{
                            writingData[jj] = this->controlWord >> count;
                        } break;
                        case TARGET_TORQUE:{
                            writingData[jj] = this->targetTorque >> count;
                        } break;
                        case MODE_OF_OPERATION:{
                            writingData[jj] = this->modeOfOperation >> count;
                        } break;
                        case TARGET_POSITION:{
                            writingData[jj] = this->targetPosition >> count;
                        } break;
                        case MAX_PROFILE_VELOCITY:{
                            writingData[jj] = this->maxProfileVelocity >> count;
                        } break;
                        case PROFILE_ACCELERATION:{
                            writingData[jj] = this->profileAcceleration >> count;
                        } break;
                        case PROFILE_DECELERATION:{
                            writingData[jj] = this->profileDeceleration >> count;
                        } break;
                    }
                    count += 8;
                }   
                startByte += this->RPDOBytes[k];
            } 
        }
    private:
        void OperationConfiguration(){
            this->SetControlWord((uint16_t) 0x0000);
            int checker = this->GetStatusWord();
            while (checker != 576 ){
                Motor::ReadEthercat();
                this->WritingPDOs();
                this->ReadingPDOs();
                Motor::WriteEthercat();
                needlf = TRUE;
                osal_usleep(1000);
                checker = this->GetStatusWord();
            } 
            this->SetControlWord((uint16_t) 0x0006);
            while (checker != 545){
                Motor::ReadEthercat();
                this->WritingPDOs();
                this->ReadingPDOs();
                Motor::WriteEthercat();
                needlf = TRUE;
                osal_usleep(1000);
                checker = this->GetStatusWord();
            } 
            this->SetControlWord((uint16_t) 0x000f);
            while (!((checker = 4663) || (checker = 1591))){
                Motor::ReadEthercat();
                this->WritingPDOs();
                this->ReadingPDOs();
                Motor::WriteEthercat();
                needlf = TRUE;
                osal_usleep(1000);
                checker = this->GetStatusWord();
            } 
        }
    public:
    //Configuration of each operation mode
        void ConfigureCyclicSynchronousTorqueMode(){
            this->SetModeOfOperation((int8_t) 10);
            this->OperationConfiguration();
            // cout<<"Status Word: "<<std::dec<<this->GetStatusWord()<<endl;
        }
        void ConfigureCyclicSynchronousPositionMode(){
            this->SetModeOfOperation((int8_t) 8);
            this->OperationConfiguration();
        }
        void ConfigureProfilePositionMode(){
            this->SetModeOfOperation((int8_t) 1);
            this->OperationConfiguration();
        }
    public:
        /* Getters */
        int GetPositionActualValue(){
            return (int) this->positionActualValue;
        }
        int GetStatusWord(){
            return (int) this->statusWord;
        }
        int GetModeOfOperationDisplay(){
            return (int) this->modeOfOperationDisplay;
        }
        int GetPositionDemandValue(){
            return (int) this->positionDemandValue;
        }
        int GetTargetPosition(){
            return (int) this->targetPosition;
        }
        int GetControlWord(){
            return (int) this->controlWord;
        }
        int GetTorqueActualValue(){
            return (int) this->torqueActualValue;
        }
        /* Setters */
        void SetControlWord(uint16_t controlWord){
            this->controlWord = controlWord;
        }
        void SetTargetTorque(int16_t targetTorque){
            this->targetTorque = targetTorque;
        }
        void SetModeOfOperation(int8_t modeOfOperation){
            this->modeOfOperation = modeOfOperation;
        }
        void SetTargetPosition(int32_t targetPosition){
            this->targetPosition = targetPosition;
        }
        void SetMaxProfileVelocity(uint32_t maxProfileVelocity){
            this->maxProfileVelocity = maxProfileVelocity;
        }
        void SetProfileAcceleration(uint32 profileAcceleration){
            this->profileAcceleration = profileAcceleration;
        }
        void SetProfileDeceleration(uint32 profileDeceleration){
            this->profileDeceleration = profileDeceleration;
        }
};  