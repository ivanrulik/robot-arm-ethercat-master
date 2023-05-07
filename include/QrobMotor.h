/**
 * @file QRobMotor.h
 * @author Ivan Rulik
 * @details It incorporates a class, called QRobMotor making it simple to interface with this drivers
 * @date 05/07/2023
 * @copyright Copyright (c) 2023
**/
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
enum ReadingVariables
{
    STATUS_WORD,
    POSITION_ACTUAL_VALUE,
};
enum WritingVariables
{
    CONTROL_WORD,
    TARGET_POSITION,
    PROFILE_VELOCITY,
    PROFILE_ACCELERATION,
    PROFILE_DECELERATION,
    TARGET_TORQUE,
    MODE_OF_OPERATION,
    EMPTY_CASE_WRITTING,
};

class QrobMotor
{
private:
    uint16_t slave;
    int gearRatio;
    int incrementPerRevolution;
    int stallTorque;
    /* Reading Variables TPDOs*/
    uint16_t statusWord{0};
    int32_t positionActualValue{0};
    static const int arrayReadingSize{2};
    int TPDOBytes[arrayReadingSize]{2, 4};

    /* Writing Variables RPDOs*/
    uint16_t controlWord{0};
    int32_t targetPosition{0};
    uint32_t profileVelocity{0};
    uint32_t profileAcceleration{0};
    uint32_t profileDeceleration{0};
    uint16_t targetTorque{0};
    int8_t modeOfOperation{0};
    static const int arrayWritingSize{8};
    int RPDOBytes[arrayWritingSize]{2, 4, 4, 4, 4, 2, 1, 1};
    static int usedMemory;
    static int expectedWKC;

public:
    QrobMotor(uint16_t slave, int stallTorque, int gearRatio = 100, int incrementPerRevolution = 4800) : syncManager2{(uint16)slave, (uint16)0x1C12, UNSIGNED8},
                                                                                                         syncManager3{(uint16)slave, (uint16)0x1C13, UNSIGNED8},
                                                                                                         rxPDO1{(uint16)slave, (uint16)0x1600},
                                                                                                         txPDO1{(uint16)slave, (uint16)0x1a00}
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
    void ConfigurePDOs()
    {
        // Consider adding more rxPDOs/txPDOs
        this->syncManager2.SDOwrite(0, (uint8)0x0000, UNSIGNED8); // set index 0 to 0 so rxPDO1 can be configured
        this->syncManager3.SDOwrite(0, (uint8)0x0000, UNSIGNED8); // set index 0 to 0 so txPDO1 can be configured
        this->rxPDO1.SDOwrite(0, 0x0000, UNSIGNED8);              // set index 0 to 0 so the other indices can be added
        // value = index(xxxx)+subindex(xx)+bits(xx)
        this->rxPDO1.SDOwrite(0x60400010, 0x0001, UNSIGNED32); // 0x6040 := Controlword; type := uint16; bytes := 2;
        this->rxPDO1.SDOwrite(0x607A0020, 0x0002, UNSIGNED32); // 0x607A := Target position; type := int32; bytes := 4;
        this->rxPDO1.SDOwrite(0x60810020, 0x0003, UNSIGNED32); // 0x6081 := Profile velocity; type := uint32; bytes:= 4;
        this->rxPDO1.SDOwrite(0x60830020, 0x0004, UNSIGNED32); // 0x6083 := Profile acceleration; type := uint32; bytes := 4;
        this->rxPDO1.SDOwrite(0x60840020, 0x0005, UNSIGNED32); // 0x6084 := Profile deceleration; type := uint32; bytes := 4;
        this->rxPDO1.SDOwrite(0x60710010, 0x0006, UNSIGNED32); // 0x6071 := Target torque; type := int16; bytes := 2;
        this->rxPDO1.SDOwrite(0x60600008, 0x0007, UNSIGNED32); // 0x6060 := Modes of operation; type := int8; bytes := 1;
        this->rxPDO1.SDOwrite(0x00000008, 0x0008, UNSIGNED32); // 0x0000 := Empty space (needed by qrob); bytes := 1;
        this->rxPDO1.SDOwrite(0x08, 0x0000, UNSIGNED8);        // set index 0 to 8 (number of variables in PDO)
        this->txPDO1.SDOwrite(0, 0x0000, UNSIGNED8);
        this->txPDO1.SDOwrite(0x60410010, 0x0001, UNSIGNED32); // 0x6041 := Statusword; type := uint16; bytes := 2;
        this->txPDO1.SDOwrite(0x60640020, 0x0002, UNSIGNED32); // 0x6064 := Position actual value; type := int32; bytes := 4
        this->txPDO1.SDOwrite(0x02, 0x0000, UNSIGNED8);
        this->syncManager2.SDOwrite(0x1600, 0x0001, UNSIGNED16); // set subindex 1 with the value of the rxPDO1 in hexa
        this->syncManager2.SDOwrite(1, 0x0000, UNSIGNED8);       // Activate syncManager
        this->syncManager3.SDOwrite(0x1a00, 0x0001, UNSIGNED16);
        this->syncManager3.SDOwrite(1, 0x0000, UNSIGNED8);
    }
    static void ConfigureMotors()
    {
        char IOmap[4096];
        static int usedMemory = ec_config_map(&IOmap);
        static int expectedWKC = (ec_group[0].outputsWKC * 2) + ec_group[0].inputsWKC;
        ec_configdc();
        ec_statecheck(0, EC_STATE_SAFE_OP, EC_TIMEOUTSTATE * 4);
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
        } while (chk-- && (ec_slave[0].state != EC_STATE_OPERATIONAL));
    }
    static int ReadEthercat()
    {
        int wkc = ec_receive_processdata(EC_TIMEOUTRET);
        return wkc;
    }
    static void WriteEthercat()
    {
        ec_send_processdata();
    }
    static void CloseCommunication()
    {
        ec_slave[0].state = EC_STATE_INIT;
        // ec_writestate(0);
        // ec_close(); // closes the network
    }

public:
    void ReadingPDOs()
    {
        int startByte = 0;
        int lastByte = 0;
        int initWriting = 0;
        uint8_t *readingData;
        readingData = ec_slave[this->slave].inputs;
        int count;
        this->statusWord = 0;
        this->positionActualValue = 0;
        for (int j = 0; j < this->arrayReadingSize; j++)
        {
            lastByte += this->TPDOBytes[j];
            count = 0;
            for (int k = startByte; k < lastByte; k++)
            {
                switch (j)
                {
                case STATUS_WORD:
                {
                    this->statusWord |= (readingData[k] << 8 * count) & (0xFF << 8 * count);
                }
                break;
                case POSITION_ACTUAL_VALUE:
                {
                    this->positionActualValue |= (readingData[k] << 8 * count) & (0xFF << 8 * count);
                }
                break;
                }
                count++;
            }
            startByte += this->TPDOBytes[j];
        }
    }
    void WritingPDOs()
    {
        int startByte = 0;
        int lastByte = 0;
        int count;
        uint8_t *writingData;
        writingData = ec_slave[this->slave].outputs;
        for (int k = 0; k < this->arrayWritingSize; k++)
        {
            count = 0;
            lastByte += this->RPDOBytes[k];
            for (int jj = startByte; jj < lastByte; jj++)
            {
                switch (k)
                {
                case CONTROL_WORD:
                {
                    writingData[jj] = this->controlWord >> count;
                }
                break;
                case TARGET_POSITION:
                {
                    writingData[jj] = this->targetPosition >> count;
                }
                break;
                case PROFILE_VELOCITY:
                {
                    writingData[jj] = this->profileVelocity >> count;
                }
                break;
                case PROFILE_ACCELERATION:
                {
                    writingData[jj] = this->profileAcceleration >> count;
                }
                break;
                case PROFILE_DECELERATION:
                {
                    writingData[jj] = this->profileDeceleration >> count;
                }
                break;
                case TARGET_TORQUE:
                {
                    writingData[jj] = this->targetTorque >> count;
                }
                break;
                case MODE_OF_OPERATION:
                {
                    writingData[jj] = this->modeOfOperation >> count;
                }
                break;
                case EMPTY_CASE_WRITTING:
                {
                }
                break;
                }
                count += 8;
            }
            startByte += this->RPDOBytes[k];
        }
    }
    int ErrorChecker()
    {
        int checker = 0;
        this->SetControlWord((uint16_t)0x0088);
        int cont = 0;
        while (checker == 0)
        {
            QrobMotor::ReadEthercat();
            this->WritingPDOs();
            this->ReadingPDOs();
            QrobMotor::WriteEthercat();
            needlf = TRUE;
            osal_usleep(1000);
            cout << std::dec << ++cont << endl;
            if (cont > 2)
            {
                checker = this->GetStatusWord();
                cout << "Status Word: " << std::dec << checker << endl;
                cout << "State0 of slave" << std::dec << this->slave << endl;
            }
            if (cont == 100)
            {
                checker = 4616;
                break;
            }
        }
        if (checker == 4616)
        {
            return 1;
        }
        return 0;
    }

private:
    void OperationConfiguration()
    {
        int checker;
        this->SetControlWord((uint16_t)0x0088);
        checker = 0;
        while (checker != 4688)
        {
            QrobMotor::ReadEthercat();
            this->WritingPDOs();
            this->ReadingPDOs();
            QrobMotor::WriteEthercat();
            needlf = TRUE;
            osal_usleep(1000);
            checker = this->GetStatusWord();
            cout << "Status Word: " << std::dec << this->GetStatusWord() << endl;
            cout << "Position Actual Value: " << std::dec << this->GetPositionActualValue() << endl;
            cout << "State1 of slave" << std::dec << this->slave << endl;
            if (checker == 4616)
                break;
        }
        this->SetControlWord((uint16_t)0x0000);
        checker = 0;
        while (checker != 4688)
        {
            QrobMotor::ReadEthercat();
            this->WritingPDOs();
            this->ReadingPDOs();
            QrobMotor::WriteEthercat();
            needlf = TRUE;
            osal_usleep(1000);
            checker = this->GetStatusWord();
            cout << "Status Word: " << std::dec << this->GetStatusWord() << endl;
            cout << "Position Actual Value: " << std::dec << this->GetPositionActualValue() << endl;
            cout << "State2 of slave" << std::dec << this->slave << endl;
            if (checker == 4616)
                break;
        }
        this->SetControlWord((uint16_t)0x0006);
        checker = 0;
        while (checker != 4657)
        {
            QrobMotor::ReadEthercat();
            this->WritingPDOs();
            this->ReadingPDOs();
            QrobMotor::WriteEthercat();
            needlf = TRUE;
            osal_usleep(1000);
            checker = this->GetStatusWord();
            cout << "Status Word: " << std::dec << this->GetStatusWord() << endl;
            cout << "Position Actual Value: " << std::dec << this->GetPositionActualValue() << endl;
            cout << "State3 of slave" << std::dec << this->slave << endl;
        }
        this->SetControlWord((uint16_t)0x0007);
        while (checker != 4659)
        {
            QrobMotor::ReadEthercat();
            this->WritingPDOs();
            this->ReadingPDOs();
            QrobMotor::WriteEthercat();
            needlf = TRUE;
            osal_usleep(1000);
            checker = this->GetStatusWord();
            cout << "Status Word: " << std::dec << this->GetStatusWord() << endl;
            cout << "Position Actual Value: " << std::dec << this->GetPositionActualValue() << endl;
            cout << "State4 of slave" << std::dec << this->slave << endl;
        }
        this->SetControlWord((uint16_t)0x000f);
        while (checker != 5687)
        {
            QrobMotor::ReadEthercat();
            this->WritingPDOs();
            this->ReadingPDOs();
            QrobMotor::WriteEthercat();
            needlf = TRUE;
            osal_usleep(1000);
            checker = this->GetStatusWord();
            cout << "Status Word: " << std::dec << this->GetStatusWord() << endl;
            cout << "Position Actual Value: " << std::dec << this->GetPositionActualValue() << endl;
            cout << "State5 of slave" << std::dec << this->slave << endl;
        }
    }

public:
    // Configuration of each operation mode
    void ConfigureCyclicSynchronousTorqueMode()
    {
        this->SetModeOfOperation((int8_t)10);
        this->OperationConfiguration();
        // cout<<"Status Word: "<<std::dec<<this->GetStatusWord()<<endl;
    }
    void ConfigureCyclicSynchronousPositionMode()
    {
        this->SetModeOfOperation((int8_t)8);
        this->OperationConfiguration();
    }
    void ConfigureProfilePositionMode()
    {
        this->SetModeOfOperation((int8_t)1);
        this->OperationConfiguration();
    }

public:
    /* Getters */
    int GetStatusWord()
    {
        return (int)this->statusWord;
    }
    int GetPositionActualValue()
    {
        return (int)this->positionActualValue;
    }
    /* Setters */
    void SetControlWord(uint16_t controlWord)
    {
        this->controlWord = controlWord;
    }
    void SetTargetPosition(int32_t targetPosition)
    {
        this->targetPosition = targetPosition;
    }
    void SetProfileVelocity(uint32 profileVelocity)
    {
        this->profileVelocity = profileVelocity;
    }
    void SetProfileAcceleration(uint32 profileAcceleration)
    {
        this->profileAcceleration = profileAcceleration;
    }
    void SetProfileDeceleration(uint32 profileDeceleration)
    {
        this->profileDeceleration = profileDeceleration;
    }
    void SetTargetTorque(int16_t targetTorque)
    {
        this->targetTorque = targetTorque;
    }
    void SetModeOfOperation(int8_t modeOfOperation)
    {
        this->modeOfOperation = modeOfOperation;
    }
};