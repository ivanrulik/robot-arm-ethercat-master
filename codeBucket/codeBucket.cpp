/**
 * @file pdo_cubemars.cpp
 * @authors Ivan Rulik, Javier Sanjuan (rulik@uwm.edu, jsanjuan@uwm.edu)
 * @brief EtherCAT code in C++ that uses SOEM to run an EtherCAt master, load PDO maps and test 1 EtherCAT slave
 * with Cyclic Synchronous Torque (CST) over using PDOs
 * @details to run this code you will need to run the following commands in terminal:
 * 1. cd ~/<wherever you stored SOEM folder>/SOEM/
 * 2. mkdir build
 * 3. cd build
 * 4. cmake ..
 * 5. make
 * 6. cd test/linux/pdo_cubemars/
 * 7. sudo ./pdo_cubemars.cpp <your network interface for EtherCAT>
 * 8. example ./pdo_cubemars.cpp enp3s0 (use ifconfig command in terminal to find the name of your network interface for ethercat)
 * When you run the code properly you will get something like:
 * O: 00 00 00 00 0a I:40 02 8b 3b 00 00 0a statusword 576 Position Values 15243
 * @version 0.1
 * @date 2022-04-06
 * 
 * @copyright Copyright (c) 2022
 * 
 */


/**
 * 
 * 
 * If you meet any Issues. Please contact me directly by email.
 * I do not like to communicate with you through a salesperson.
 * 
 *    By a engineer.                            anjizcx@gmail.com
 * 
 **/




#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <inttypes.h>
#include <stdlib.h>
#include "ethercat.h"

#define EC_TIMEOUTMON 1000

struct motorCommands
{
   uint16 removeError;
   uint16 prepare2Power;
   uint16 switchOn;
   uint16 operational;
   uint16 disable;
   uint16 move;
};
struct motorStateMachine
{
   uint16 state_error;
   uint16 state_disabled;
   uint16 state_ready2SwitchOn;
   uint16 state_switchedOn;
   uint16 state_operational;
};
struct ObjDict
{
   uint16 index;
   uint16 valueU16;
   int size;
   int8 value8;
   int32 value32;
   int16 value16;
   int size2;
};
struct motorCommands epos4ControlWords = {0x0080,0x0006,0x0007,0x000f,0x0000,0x003f};
struct motorStateMachine epos4StateMachine = {4616,576,545,547,4663};
struct ObjDict syncMan2 = {0x1c12,0x0000,0x10,0x00,0,0,0x08};
struct ObjDict syncMan3 = {0x1c13,0x0000,0x10,0x00,0,0,0x08};
struct ObjDict receivePDO1 = {0x1600,0x0000,0x20,0x00,0,0,0x08};
struct ObjDict transmitPDO1 = {0x1a00,0x0000,0x20,0x00,0,0,0x08};
struct ObjDict statusWord = {0x6041,0x0000,0x10,0x00,0,0,0};
struct ObjDict controlWord = {0x6040,0x0000,0x10,0x00,0,0,0};
struct ObjDict modeOperation = {0x6060,0x0000,0x08,0x00,0,0,0};
struct ObjDict modeOperationDisplay = {0x6061,0x0000,0x08,0x00,0,0,0};
struct ObjDict positionActualValue = {0x6064,0x0000,0x20,0x00,0,0,0};
struct ObjDict targetPosition = {0x607a,0x0000,0x20,0x00,0,0,0};
struct ObjDict targetTorque = {0x6071,0x0000,0x10,0x00,0,0,0};
struct ObjDict torqueActualValue = {0x6077,0x0000,0x10,0x00,0,0,0};

char IOmap[4096];
int usedmem;
OSAL_THREAD_HANDLE thread1;
int expectedWKC;
boolean needlf;
volatile int wkc;
boolean inOP;
uint8 currentgroup = 0;

OSAL_THREAD_FUNC ecatcheck(void *ptr)
{
    int slave;
    (void)ptr;                  /* Not used */
    printf("Thread on\n");
    // printf("%d\n",ec_group[currentgroup].docheckstate);
    while(1)
    {
        if( inOP && ((wkc < expectedWKC) || ec_group[currentgroup].docheckstate))
        {
            if (needlf)
            {
               needlf = FALSE;
               printf("\n");
            }
            /* one ore more slaves are not responding */
            ec_group[currentgroup].docheckstate = FALSE;
            ec_readstate();
            for (slave = 1; slave <= ec_slavecount; slave++)
            {
               if ((ec_slave[slave].group == currentgroup) && (ec_slave[slave].state != EC_STATE_OPERATIONAL))
               {
                  ec_group[currentgroup].docheckstate = TRUE;
                  if (ec_slave[slave].state == (EC_STATE_SAFE_OP + EC_STATE_ERROR))
                  {
                     printf("ERROR : slave %d is in SAFE_OP + ERROR, attempting ack.\n", slave);
                     ec_slave[slave].state = (EC_STATE_SAFE_OP + EC_STATE_ACK);
                     ec_writestate(slave);
                  }
                  else if(ec_slave[slave].state == EC_STATE_SAFE_OP)
                  {
                     printf("WARNING : slave %d is in SAFE_OP, change to OPERATIONAL.\n", slave);
                     ec_slave[slave].state = EC_STATE_OPERATIONAL;
                     ec_writestate(slave);
                  }
                  else if(ec_slave[slave].state > EC_STATE_NONE)
                  {
                     if (ec_reconfig_slave(slave, EC_TIMEOUTMON))
                     {
                        ec_slave[slave].islost = FALSE;
                        printf("MESSAGE : slave %d reconfigured\n",slave);
                     }
                  }
                  else if(!ec_slave[slave].islost)
                  {
                     /* re-check state */
                     ec_statecheck(slave, EC_STATE_OPERATIONAL, EC_TIMEOUTRET);
                     if (ec_slave[slave].state == EC_STATE_NONE)
                     {
                        ec_slave[slave].islost = TRUE;
                        printf("ERROR : slave %d lost\n",slave);
                     }
                  }
               }
               if (ec_slave[slave].islost)
               {
                  if(ec_slave[slave].state == EC_STATE_NONE)
                  {
                     if (ec_recover_slave(slave, EC_TIMEOUTMON))
                     {
                        ec_slave[slave].islost = FALSE;
                        printf("MESSAGE : slave %d recovered\n",slave);
                     }
                  }
                  else
                  {
                     ec_slave[slave].islost = FALSE;
                     printf("MESSAGE : slave %d found\n",slave);
                  }
               }
            }
            if(!ec_group[currentgroup].docheckstate)
               printf("OK : all slaves resumed OPERATIONAL.\n");
        }
        osal_usleep(10000);
    }
}

int epos4setup(uint16 slave)
{
   int retval;
   uint16 map_1c12[2] = {0x0001,0x1600}; //outputs
   uint16 map_1c13[2] = {0x0001,0x1a00}; //inputs
   uint16 map_1a00[5] = {0x6041, 0x6064, 0x606c, 0x6077, 0x6061}; //inputs
   // bitl 0x10, 0x20, 0x20, 0x10, 0x08, 0x20
   uint16 map_1600[5] = {0x6040, 0x607A, 0x60FF, 0x60B2, 0x6060}; // outputs
   // bitl 0x10, 0x10, 0x10, 0x08, 0x20
   // sub idx 0x00, 0x00, 0x00, 0x00, 0x01 
   retval = 0;
   int rv = 0;

   // BEGIN
   // Rx PDO
   printf("Loading PDO Map\n");
   uint8 init_map_1c12 = 0x00; //outputs
   uint8 check_map_1c12 = 0x00;
   while(retval == 0)
   {
      rv = ec_SDOread(slave, syncMan2.index, (uint8)0x00, FALSE, &syncMan2.size2, &check_map_1c12, EC_TIMEOUTRXM);
      printf("epos4 slave %d set, retval = %d, map sync2 sent = %x\n", slave, rv,check_map_1c12);
      retval = ec_SDOwrite(slave, syncMan2.index, (uint8)0x00, FALSE, syncMan2.size2, &init_map_1c12, EC_TIMEOUTRXM);
      printf("epos4 slave %d set, retval = %d, map sync2 sent = %x\n", slave, retval,init_map_1c12);
      rv = ec_SDOread(slave, syncMan2.index, (uint8)0x00, FALSE, &syncMan2.size2, &check_map_1c12, EC_TIMEOUTRXM);
      printf("epos4 slave %d set, retval = %d, map sync2 sent = %x\n", slave, rv,check_map_1c12);
   }
   // Tx PDO
   uint8 init_map_1c13 = 0x00; //outputs
   uint8 check_map_1c13 = 0x00;
   while(retval == 0)
   {
      rv = ec_SDOread(slave, syncMan3.index, (uint8)0x00, FALSE, &syncMan3.size2, &check_map_1c13, EC_TIMEOUTRXM);
      printf("epos4 slave %d set, retval = %d, map sync2 sent = %x\n", slave, rv,check_map_1c13);
      retval = ec_SDOwrite(slave, syncMan3.index, (uint8)0x00, FALSE, syncMan3.size2, &init_map_1c13, EC_TIMEOUTRXM);
      printf("epos4 slave %d set, retval = %d, map sync2 sent = %x\n", slave, retval,init_map_1c13);
      rv = ec_SDOread(slave, syncMan3.index, (uint8)0x00, FALSE, &syncMan3.size2, &check_map_1c13, EC_TIMEOUTRXM);
      printf("epos4 slave %d set, retval = %d, map sync2 sent = %x\n", slave, rv,check_map_1c13);
   }
   //Rx_PDO1
   retval = 0;
   uint8 zero_RXPDO1 = 0x00;
   uint8 checkZero_RXPDO1 = 0x00; 
   
   while(retval == 0)
   {
   rv = ec_SDOread(slave,receivePDO1.index,(uint8)0x00,FALSE,&receivePDO1.size2,&checkZero_RXPDO1,EC_TIMEOUTSAFE);// a) Writethevalue“0”(zero)tosubindex0x00(disablePDO).
   printf("epos4 slave %d set, retval = %d, map 1600 read = %x\n", slave, rv,checkZero_RXPDO1);
   retval = ec_SDOwrite(slave,receivePDO1.index,(uint8)0x00,FALSE,receivePDO1.size2,&zero_RXPDO1,EC_TIMEOUTSAFE);// a) Writethevalue“0”(zero)tosubindex0x00(disablePDO).
   printf("epos4 slave %d set, retval = %d, map 1600 sent = %x\n", slave, retval,zero_RXPDO1);
   rv = ec_SDOread(slave,receivePDO1.index,(uint8)0x00,FALSE,&receivePDO1.size2,&checkZero_RXPDO1,EC_TIMEOUTSAFE);// a) Writethevalue“0”(zero)tosubindex0x00(disablePDO).
   printf("epos4 slave %d set, retval = %d, map 1600 read = %x\n", slave, rv,checkZero_RXPDO1);
   }
   retval = 0;
   uint32 pdo1_RXPDO1 = 0x60400010;
   uint32 check_RXPDO1 = 0x00000000;
   while(retval == 0)
   {
   rv = ec_SDOread(slave,receivePDO1.index,(uint8)0x01,FALSE,&receivePDO1.size,&check_RXPDO1,EC_TIMEOUTSAFE);// b) Modifythedesiredobjectsinsubindex0x01...0x0n.
   printf("epos4 slave %d set, retval = %d, map 1600 read = %x\n", slave, retval,check_RXPDO1);
   retval = ec_SDOwrite(slave,receivePDO1.index,(uint8)0x01,FALSE,receivePDO1.size,&pdo1_RXPDO1,EC_TIMEOUTSAFE);// b) Modifythedesiredobjectsinsubindex0x01...0x0n.
   printf("epos4 slave %d set, retval = %d, map 1600 sent = %x\n", slave, retval,pdo1_RXPDO1);
   rv = ec_SDOread(slave,receivePDO1.index,(uint8)0x01,FALSE,&receivePDO1.size,&check_RXPDO1,EC_TIMEOUTSAFE);// b) Modifythedesiredobjectsinsubindex0x01...0x0n.
   printf("epos4 slave %d set, retval = %d, map 1600 read = %x\n", slave, retval,check_RXPDO1);
   }
   retval = 0;
   uint32 pdo2_RXPDO1 = 0x60710010;
   check_RXPDO1 = 0x00000000;
   while(retval == 0)
   {
   rv = ec_SDOread(slave,receivePDO1.index,(uint8)0x02,FALSE,&receivePDO1.size,&check_RXPDO1,EC_TIMEOUTSAFE);// b) Modifythedesiredobjectsinsubindex0x01...0x0n.
   printf("epos4 slave %d set, retval = %d, map 1600 read = %x\n", slave, retval,check_RXPDO1);
   retval = ec_SDOwrite(slave,receivePDO1.index,(uint8)0x02,FALSE,receivePDO1.size,&pdo2_RXPDO1,EC_TIMEOUTSAFE);// b) Modifythedesiredobjectsinsubindex0x01...0x0n.
   printf("epos4 slave %d set, retval = %d, map 1600 sent = %x\n", slave, retval,pdo2_RXPDO1);
   rv = ec_SDOread(slave,receivePDO1.index,(uint8)0x02,FALSE,&receivePDO1.size,&check_RXPDO1,EC_TIMEOUTSAFE);// b) Modifythedesiredobjectsinsubindex0x01...0x0n.
   printf("epos4 slave %d set, retval = %d, map 1600 read = %x\n", slave, retval,check_RXPDO1);
   }
   retval = 0;
   uint32 pdo3_RXPDO1 = 0x60600008;
   check_RXPDO1 = 0x00000000;
   while(retval == 0)
   {
   retval = ec_SDOwrite(slave,receivePDO1.index,(uint8)0x03,FALSE,receivePDO1.size,&pdo3_RXPDO1,EC_TIMEOUTSAFE);// b) Modifythedesiredobjectsinsubindex0x01...0x0n.
   printf("epos4 slave %d set, retval = %d, map 1600 sent = %x\n", slave, retval,pdo3_RXPDO1);
   rv = ec_SDOread(slave,receivePDO1.index,(uint8)0x03,FALSE,&receivePDO1.size,&check_RXPDO1,EC_TIMEOUTSAFE);// b) Modifythedesiredobjectsinsubindex0x01...0x0n.
   printf("epos4 slave %d set, retval = %d, map 1600 read = %x\n", slave, retval,check_RXPDO1);
   }
   retval = 0;
   uint8 pdo_list_RXPDO1 = 0x03;
   check_RXPDO1 = 0x00000000;
   while(retval == 0)
   {
   retval = ec_SDOwrite(slave,receivePDO1.index,(uint8)0x00,FALSE,receivePDO1.size2,&pdo_list_RXPDO1,EC_TIMEOUTSAFE);// c) Write the desired number of mapped objects to subindex 0x00
   printf("epos4 slave %d set, retval = %d, map 1600 sent = %x\n", slave, retval,pdo_list_RXPDO1);
   rv = ec_SDOread(slave,receivePDO1.index,(uint8)0x00,FALSE,&receivePDO1.size2,&check_RXPDO1,EC_TIMEOUTSAFE);// c) Write the desired number of mapped objects to subindex 0x00
   printf("epos4 slave %d set, retval = %d, map 1600 read = %x\n", slave, rv,check_RXPDO1);
   }
   //Tx_PDO1///////////////////////////////////
   retval = 0;
   uint8 zero_TXPDO1 = 0x00;
   uint8 checkZero_TXPDO1 = 0x00; 
   while(retval == 0)
   {
   retval = ec_SDOwrite(slave,transmitPDO1.index,(uint8)0x00,FALSE,transmitPDO1.size2,&zero_TXPDO1,EC_TIMEOUTSAFE);// a) Writethevalue“0”(zero)tosubindex0x00(disablePDO).
   printf("epos4 slave %d set, retval = %d, map 1600 sent = %x\n", slave, retval,zero_TXPDO1);
   rv = ec_SDOread(slave,transmitPDO1.index,(uint8)0x00,FALSE,&transmitPDO1.size2,&checkZero_TXPDO1,EC_TIMEOUTSAFE);// a) Writethevalue“0”(zero)tosubindex0x00(disablePDO).
   printf("epos4 slave %d set, retval = %d, map 1600 read = %x\n", slave, rv,checkZero_TXPDO1);
   }
   retval = 0;
   uint32 pdo1_TXPDO1 = 0x60410010;
   uint32 check_TXPDO1 = 0x00000000;
   while(retval == 0)
   {
   retval = ec_SDOwrite(slave,transmitPDO1.index,(uint8)0x01,FALSE,transmitPDO1.size,&pdo1_TXPDO1,EC_TIMEOUTSAFE);// b) Modifythedesiredobjectsinsubindex0x01...0x0n.
   printf("epos4 slave %d set, retval = %d, map 1600 sent = %x\n", slave, retval,pdo1_TXPDO1);
   rv = ec_SDOread(slave,transmitPDO1.index,(uint8)0x01,FALSE,&transmitPDO1.size,&check_TXPDO1,EC_TIMEOUTSAFE);// b) Modifythedesiredobjectsinsubindex0x01...0x0n.
   printf("epos4 slave %d set, retval = %d, map 1600 read = %x\n", slave, retval,check_TXPDO1);
   }
   retval = 0;
   uint32 pdo2_TXPDO1 = 0x60640020;
   check_TXPDO1 = 0x00000000;
   while(retval == 0)
   {
   retval = ec_SDOwrite(slave,transmitPDO1.index,(uint8)0x02,FALSE,transmitPDO1.size,&pdo2_TXPDO1,EC_TIMEOUTSAFE);// b) Modifythedesiredobjectsinsubindex0x01...0x0n.
   printf("epos4 slave %d set, retval = %d, map 1600 sent = %x\n", slave, retval,pdo2_TXPDO1);
   rv = ec_SDOread(slave,transmitPDO1.index,(uint8)0x02,FALSE,&transmitPDO1.size,&check_TXPDO1,EC_TIMEOUTSAFE);// b) Modifythedesiredobjectsinsubindex0x01...0x0n.
   printf("epos4 slave %d set, retval = %d, map 1600 read = %x\n", slave, retval,check_TXPDO1);
   }
   retval = 0;
   uint32 pdo3_TXPDO1 = 0x60610008;
   check_TXPDO1 = 0x00000000;
   while(retval == 0)
   {
   retval = ec_SDOwrite(slave,transmitPDO1.index,(uint8)0x03,FALSE,transmitPDO1.size,&pdo3_TXPDO1,EC_TIMEOUTSAFE);// b) Modifythedesiredobjectsinsubindex0x01...0x0n.
   printf("epos4 slave %d set, retval = %d, map 1600 sent = %x\n", slave, retval,pdo3_TXPDO1);
   rv = ec_SDOread(slave,transmitPDO1.index,(uint8)0x03,FALSE,&transmitPDO1.size,&check_TXPDO1,EC_TIMEOUTSAFE);// b) Modifythedesiredobjectsinsubindex0x01...0x0n.
   printf("epos4 slave %d set, retval = %d, map 1600 read = %x\n", slave, retval,check_TXPDO1);
   }
   retval = 0;
   uint8 pdo_list_TXPDO1 = 0x03;
   check_TXPDO1 = 0x00000000;
   while(retval == 0)
   {
   retval = ec_SDOwrite(slave,transmitPDO1.index,(uint8)0x00,FALSE,transmitPDO1.size2,&pdo_list_TXPDO1,EC_TIMEOUTSAFE);// c) Write the desired number of mapped objects to subindex 0x00
   printf("epos4 slave %d set, retval = %d, map 1600 sent = %x\n", slave, retval,pdo_list_TXPDO1);
   rv = ec_SDOread(slave,transmitPDO1.index,(uint8)0x00,FALSE,&transmitPDO1.size2,&check_TXPDO1,EC_TIMEOUTSAFE);// c) Write the desired number of mapped objects to subindex 0x00
   printf("epos4 slave %d set, retval = %d, map 1600 read = %x\n", slave, rv,check_TXPDO1);
   }
   // END
   // Rx PDO
   retval = 0;
   uint16 rx_PDO1_idx = 0x1600;
   uint16 check_PDO1_idx = 0x0000;
   while(retval == 0)
   {
   retval = ec_SDOwrite(slave, syncMan2.index, (uint8)0x01, FALSE, syncMan2.size, &rx_PDO1_idx, EC_TIMEOUTSAFE);
   printf("epos4 slave %d set, retval = %d, map sync2 sent = %x\n", slave, retval,rx_PDO1_idx);
   rv = ec_SDOread(slave, syncMan2.index, (uint8)0x01, FALSE, &syncMan2.size, &check_PDO1_idx, EC_TIMEOUTSAFE);
   printf("epos4 slave %d set, retval = %d, map sync2 read = %x\n", slave, rv,check_PDO1_idx);
   }
   retval = 0;
   uint8 rxPDO_num = 0x01;
   uint8 check_rxPDO_num = 0x00;
   while(retval == 0)
   {
   retval = ec_SDOwrite(slave, syncMan2.index, (uint8)0x00, FALSE, syncMan2.size2, &rxPDO_num, EC_TIMEOUTSAFE);
   printf("epos4 slave %d set, retval = %d, map sync2 sent = %x\n", slave, retval,rxPDO_num);
   rv = ec_SDOread(slave, syncMan2.index, (uint8)0x00, FALSE, &syncMan2.size2, &check_rxPDO_num, EC_TIMEOUTSAFE);
   printf("epos4 slave %d set, retval = %d, map sync2 read = %x\n", slave, rv,check_rxPDO_num);
   }
   // Tx PDO
   retval = 0;
   uint16 tx_PDO1_idx = 0x1a00;
   check_PDO1_idx = 0x0000;
   while(retval == 0)
   {
   retval = ec_SDOwrite(slave, syncMan3.index, (uint8)0x01, FALSE, syncMan3.size, &tx_PDO1_idx, EC_TIMEOUTSAFE);
   printf("epos4 slave %d set, retval = %d, map sync2 sent = %x\n", slave, retval,tx_PDO1_idx);
   rv = ec_SDOread(slave, syncMan2.index, (uint8)0x01, FALSE, &syncMan3.size, &check_PDO1_idx, EC_TIMEOUTSAFE);
   printf("epos4 slave %d set, retval = %d, map sync2 read = %x\n", slave, rv,check_PDO1_idx);
   }
   retval = 0;
   uint8 txPDO_num = 0x01;
   uint8 check_txPDO_num = 0x00;
   while(retval == 0)
   {
   retval = ec_SDOwrite(slave, syncMan3.index, (uint8)0x00, FALSE, syncMan3.size2, &txPDO_num, EC_TIMEOUTSAFE);
   printf("epos4 slave %d set, retval = %d, map sync2 sent = %x\n", slave, retval,txPDO_num);
   rv = ec_SDOread(slave, syncMan2.index, (uint8)0x00, FALSE, &syncMan3.size2, &check_txPDO_num, EC_TIMEOUTSAFE);
   printf("epos4 slave %d set, retval = %d, map sync2 read = %x\n", slave, rv,check_txPDO_num);
   }
   printf("PDO map load finished\n");
   return 1;        
}

void set_output_int8 (uint16_t slave_nb, uint8_t module_index, int8_t value)
 {
  	uint8_t *data_ptr;
 
  	data_ptr = ec_slave[slave_nb].outputs;
  	/* Move pointer to correct module index*/
  	data_ptr += module_index * 2;
  	/* Read value byte by byte since all targets can't handle misaligned
     	   addresses */
  	*data_ptr++ = (value >> 0);// & 0xFF;
 }
void get_input_int8(uint16_t slave_nb, uint8_t module_index, int8_t *value)
 {
   uint8_t *data_ptr;
   
   data_ptr = ec_slave[slave_nb].inputs;
  	/* Move pointer to correct module index*/
  	data_ptr += module_index * 2;
   /* Read value byte by byte since all targets can't handle misaligned
   addresses */
   *value |= ((*data_ptr++) & 0xFF);
 }
void set_output_int16 (uint16_t slave_nb, uint8_t module_index, int16_t value)
 {
  	uint8_t *data_ptr;
 
  	data_ptr = ec_slave[slave_nb].outputs;
  	/* Move pointer to correct module index*/
  	data_ptr += module_index * 2;
  	/* Read value byte by byte since all targets can't handle misaligned
     	   addresses */
  	*data_ptr++ = (value >> 0);// & 0xFF;
  	*data_ptr++ = (value >> 8);// & 0xFF00;
 }
void get_input_int16(uint16_t slave_nb, uint8_t module_index, int16_t *value)
 {
   uint8_t *data_ptr;
   
   data_ptr = ec_slave[slave_nb].inputs;
  	/* Move pointer to correct module index*/
  	data_ptr += module_index * 2;
   /* Read value byte by byte since all targets can't handle misaligned
   addresses */
   *value |= ((*data_ptr++) & 0xFF);
   *value |= ((*data_ptr) << 8) & 0xff00;
 }
void get_input_int32(uint16_t slave_nb, uint8_t module_index, int32_t *value)
 {
   uint8 *data_ptr;
   data_ptr = ec_slave[slave_nb].inputs;
   // printf("%x\n",data_ptr);
   // printf("%d \n", *data_ptr);0
  	/* Move pointer to correct module index*/
  	data_ptr += module_index * 2;
   /* Read value byte by byte since all targets can't handle misaligned addresses */
   *value |= ((*data_ptr++) & 0xFF);
   *value |= ((*data_ptr++) << 8) & 0xff00;
   *value |= ((*data_ptr++) << 16) & 0xff0000;
   *value |= ((*data_ptr) << 24) & 0xff000000;
 }

void pdo_cubemars(char *ifname)
{
    if (ec_init(ifname))
    {
      int32_t positionValue; 
      int32_t velocityValue;
      int32_t torqueValue;
      int16_t statusValue;
      int8_t modeValue;
      int retval = 0;
      uint16 slave = 2;
      printf("cubemars test begin\n");
      if ( ec_config_init(FALSE) > 0 )
      {
         // slaves are now in PRE_OP
         printf("%d slaves found and configured.\n",ec_slavecount);
         // use PDO Assign or PDO Config here or before
         printf("slave 1 current state: %x\n",ec_slave[0].state);
         // epos4setup(slave);
         usedmem=ec_config_map(&IOmap);
         printf("Used memory by IO map %d\n",usedmem);
         for (size_t i = 0; i < usedmem; i++)
         {
            printf("mapdata %x\n",IOmap[i]);
         }
         ec_configdc();
         printf("Slaves mapped, state to SAFE_OP.\n");
         /* wait for all slaves to reach SAFE_OP state */
         ec_statecheck(0, EC_STATE_SAFE_OP,  EC_TIMEOUTSTATE * 4);
         int oloop = ec_slave[0].Obytes;
         if ((oloop == 0) && (ec_slave[0].Obits > 0)) oloop = 1;
         if (oloop > 8) oloop = 8;
         printf("oloop:%d\n",oloop);
         int iloop = ec_slave[0].Ibytes;
         if ((iloop == 0) && (ec_slave[0].Ibits > 0)) iloop = 1;
         if (iloop > 8) iloop = 8;
         uint8 inputValues[iloop];
         printf("slave 1 current state: %x\n",ec_slave[0].state);
         // set to op
         printf("Request operational state for all slaves\n");
         expectedWKC = (ec_group[0].outputsWKC * 2) + ec_group[0].inputsWKC;
         printf("Calculated workcounter %d\n", expectedWKC);
         ec_slave[0].state = EC_STATE_OPERATIONAL;
         ec_send_processdata();
         ec_receive_processdata(EC_TIMEOUTRET);
         /* request OP state for all slaves */
         ec_writestate(0); //esto no lo estamos haciendo 
         int chk = 200;
         /* wait for all slaves to reach OP state */
         do
         {
            ec_send_processdata();
            ec_receive_processdata(EC_TIMEOUTRET);
            ec_statecheck(0, EC_STATE_OPERATIONAL, 50000);
         }
         while (chk-- && (ec_slave[0].state != EC_STATE_OPERATIONAL));
         for(int i = 1; i <= 5000; i++)
            {
               ec_send_processdata();
               wkc = ec_receive_processdata(EC_TIMEOUTRET);

                    if(wkc >= expectedWKC)
                    {
                        printf("O:");
                        for(int j = 0 ; j < oloop; j++) // loop to read output buffer for slave 1
                        {
                            printf(" %2.2x", *(ec_slave[0].outputs + j));
                        }
                        printf(" I:");
                        for(int j = 0 ; j < iloop; j++) // loop to read input buffer for slave 1
                        {
                            inputValues[j] = *(ec_slave[0].inputs + j);
                            printf("%2.2x ", inputValues[j]);
                        }
                        // printf("\n");
                        if(i == 100)
                        {
                           ec_slave[0].outputs[0] = 0x1f;
                           ec_slave[0].outputs[1] = 0x00;
                           ec_slave[0].outputs[14] = 0x0a;     //start  test torque mode 
                        }
                        if(i == 200)
                        {
                           ec_slave[0].outputs[10] = 0xC0;
                           ec_slave[0].outputs[11] = 0xF2;
                           ec_slave[0].outputs[12] = 0xFC;
                           ec_slave[0].outputs[13] = 0xFF;     // turn -20NM
 
                        }
                        if(i == 1200)
                        {
                           ec_slave[0].outputs[10] = 0x00;
                           ec_slave[0].outputs[11] = 0x00;
                           ec_slave[0].outputs[12] = 0x00;
                           ec_slave[0].outputs[13] = 0x00;     // turn 0NM
                        }
                        if(i == 1300)
                        {
                           ec_slave[0].outputs[10] = 0x40;
                           ec_slave[0].outputs[11] = 0x0d;
                           ec_slave[0].outputs[12] = 0x03;
                           ec_slave[0].outputs[13] = 0x00;     //turn 20NM
                        }
                        if(i == 2300)
                        {
                           ec_slave[0].outputs[14] = 0x0b;  //test P-T MIX and hot change
                        }
                        if(i == 2400)
                        {
                           ec_slave[0].outputs[2] = 0xa0;
                           ec_slave[0].outputs[3] = 0x86;
                           ec_slave[0].outputs[4] = 0x01;
                           ec_slave[0].outputs[5] = 0x00;     //turn to 10rad
                           ec_slave[0].outputs[10] = 0x00;
                           ec_slave[0].outputs[11] = 0x00;
                           ec_slave[0].outputs[12] = 0x00;
                           ec_slave[0].outputs[13] = 0x00;     // offset 0NM
                        }
                        if(i == 3000)
                        {
                           ec_slave[0].outputs[2] = 0x00;
                           ec_slave[0].outputs[3] = 0x00;
                           ec_slave[0].outputs[4] = 0x00;
                           ec_slave[0].outputs[5] = 0x00;     //turn to 0rad
                           ec_slave[0].outputs[10] = 0x00;
                           ec_slave[0].outputs[11] = 0x00;
                           ec_slave[0].outputs[12] = 0x00;
                           ec_slave[0].outputs[13] = 0x00;     // offset 0NM
                        }
                        if(i == 4500)
                        {
                           ec_slave[0].outputs[0] = 0x07;
                           ec_slave[0].outputs[1] = 0x00;
                        }
                     
                        statusValue = (int16_t)(((uint16_t)ec_slave[0].inputs[0])|((uint16_t)ec_slave[0].inputs[1] << 8));
                        modeValue=ec_slave[0].inputs[14] ;
                        positionValue = ((uint32_t)ec_slave[0].inputs[2])|((uint32_t)ec_slave[0].inputs[3]<<8)|((uint32_t)ec_slave[0].inputs[4]<<16)|((uint32_t)ec_slave[0].inputs[5]<<24);
                        velocityValue = ((uint32_t)ec_slave[0].inputs[6])|((uint32_t)ec_slave[0].inputs[7]<<8)|((uint32_t)ec_slave[0].inputs[8]<<16)|((uint32_t)ec_slave[0].inputs[9]<<24);
                        torqueValue =   ((uint32_t)ec_slave[0].inputs[10])|((uint32_t)ec_slave[0].inputs[11]<<8)|((uint32_t)ec_slave[0].inputs[12]<<16)|((uint32_t)ec_slave[0].inputs[13]<<24);
      
                        printf("statusword %d; ", statusValue);
                        printf("modeValue %d; ", modeValue);
                        printf("Position Values %.2frad; ", positionValue/10000.0);
                        printf("velocity Value  %.2frad/s; ", velocityValue/10000.0);
                        printf("torque Value  %.2fNm; ", torqueValue/10000.0);
                        printf("cnt %d\n", i);
                        needlf = TRUE;
                    }
               osal_usleep(5000);
            }
         ec_slave[0].state = EC_STATE_INIT;
         /* request INIT state for all slaves */
         ec_writestate(0);
         ec_close(); // closes the network
      }
    }
    else
    {
      printf("No socket connection on %s\nExecute as root\n",ifname);
    }

}

int main(int argc, char *argv[])
{
    if(argc > 1)
    {
      osal_thread_create(&thread1, 128000, &ecatcheck, NULL);
      pdo_cubemars(argv[1]);
    }
    else
    {
      printf("no network port given\n");
    }
    return 1;
}

