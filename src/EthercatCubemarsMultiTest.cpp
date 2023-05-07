/**
 * @file EthercatCubemarsMultiTest.cpp
 * @authors Ivan Rulik
 * @brief EtherCAT code in C++ that uses SOEM over jsanjuan custom library to run an EtherCAt master, load PDO maps and test 1 EtherCAT slave
 * with Cyclic Synchronous Torque (CST) over using PDOs
 * @details to run this code you will need to run the following commands in terminal:
 * 1. cd ~/<wherever you stored SOEM folder>/SOEM/
 * 2. mkdir build
 * 3. cd build
 * 4. cmake ..
 * 5. make
 * 6. cd test/linux/pdo_test_cpp/
 * 7. sudo ./pdo_test_cpp.cpp <your network interface for EtherCAT>
 * 8. example ./pdo_test_cpp.cpp enp3s0 (use ifconfig command in terminal to find the name of your network interface for ethercat)
 * When you run the code properly you will get something like:
 * O: 00 00 00 00 0a I:40 02 8b 3b 00 00 0a statusword 576 Position Values 15243
 * @version 0.1
 * @date 2022-11-06
 * 
 * @copyright Copyright (c) 2022
 * 
 */

// #include "ODVariable.h"

// #include "Motor.h"
#include "CubemarsMotor.h"
#include "ethercat.h"
#define NSEC_PER_SEC 1000000000
#define EC_TIMEOUTMON 20000
using namespace std;
char IOmap[4096];
int usedmem;
OSAL_THREAD_HANDLE thread1,thread2;
int expectedWKC;
volatile int wkc;
boolean inOP;
uint8 currentgroup = 0;
int slave1 = 1;
int slave2 = 2;
int slave3 = 3;
int slave4 = 4;
int slave5 = 5;
int slave6 = 6;
int slave7 = 7;
int32_t positionValue; 
int16_t statusValue;
int8_t modeValue;
boolean needlf;
// Real Time vars
/* activate cyclic process data */
int dorun = 0;
int64 toff, gl_delta;
// struct motorStateMachine
// {
//    uint16 state_error;
//    uint16 state_disabled;
//    uint16 state_ready2SwitchOn;
//    uint16 state_switchedOn;
//    uint16 state_operational;
// };
// struct motorCommands
// {
//    uint16_t removeError;
//    uint16_t prepare2Power;
//    uint16_t switchOn;
//    uint16_t operational;
//    uint16_t disable;
//    uint16_t move;
// };
// struct motorCommands controlWords = {0x0080,0x0006,0x0007,0x000f,0x0000,0x003f};
// struct motorStateMachine cubemarsStateMachine = {4616,576,545,547,4663};
// Real Time Thread Block
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
/* add ns to timespec */
void add_timespec(struct timespec *ts, int64 addtime)
{
   int64 sec, nsec;

   nsec = addtime % NSEC_PER_SEC;
   sec = (addtime - nsec) / NSEC_PER_SEC;
   ts->tv_sec += sec;
   ts->tv_nsec += nsec;
   if ( ts->tv_nsec >= NSEC_PER_SEC )
   {
      nsec = ts->tv_nsec % NSEC_PER_SEC;
      ts->tv_sec += (ts->tv_nsec - nsec) / NSEC_PER_SEC;
      ts->tv_nsec = nsec;
   }
}
/* PI calculation to get linux time synced to DC time */
void ec_sync(int64 reftime, int64 cycletime , int64 *offsettime)
{
   static int64 integral = 0;
   int64 delta;
   /* set linux sync point 50us later than DC sync, just as example */
   delta = (reftime - 50000) % cycletime;
   if(delta> (cycletime / 2)) { delta= delta - cycletime; }
   if(delta>0){ integral++; }
   if(delta<0){ integral--; }
   *offsettime = -(delta / 100) - (integral / 20);
   gl_delta = delta;
}
/* RT EtherCAT thread */
OSAL_THREAD_FUNC_RT ecatthread(void *ptr)
{
   struct timespec   ts, tleft;
   int ht;
   int64 cycletime;
   // uint16 slave = 2;
   int flip = 0;
   clock_gettime(CLOCK_MONOTONIC, &ts);
   ht = (ts.tv_nsec / 1000000) + 1; /* round to nearest ms */
   ts.tv_nsec = ht * 1000000;
   if (ts.tv_nsec >= NSEC_PER_SEC) {
      ts.tv_sec++;
      ts.tv_nsec -= NSEC_PER_SEC;
   }
   cycletime = *(int*)ptr * 1000; /* cycletime in ns */
   toff = 0;
   dorun = 0;
   ec_send_processdata();
   while(1)
   {
      /* calculate next cycle start */
      add_timespec(&ts, cycletime + toff);
      /* wait to cycle start */
      clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &ts, &tleft);
      if (dorun>0)
      {
         wkc = ec_receive_processdata(EC_TIMEOUTRET);

         dorun++;
         if (dorun>50)
         {
            if(flip == 0)
            {
               set_output_int8(slave1, (uint8_t) 0,(int8_t) 0b00000001);
               flip = 1;
            }
            else
            {
               set_output_int8(slave1, (uint8_t) 0,(int8_t) 0b00000000);
               flip = 0;
            }
         }
         // printf("loop %d \n",dorun);
         if (ec_slave[0].hasdc)
         {
            /* calulate toff to get linux time and DC synced */
            ec_sync(ec_DCtime, cycletime, &toff);
         }
         ec_send_processdata();
      }
   }
}
// Real Time Thread Block
OSAL_THREAD_FUNC ecatcheck(void *ptr)
{
    int slave;
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
void test(int i, CubemarsMotor cubemars1){

}
void runner(char *ifname)
{
    if (ec_init(ifname))
    {
      if ( ec_config_init(FALSE) > 0 )
      {
         // Motor initialization
         CubemarsMotor cubemars1((uint16) slave2, 2);
         cubemars1.ConfigurePDOs();
         CubemarsMotor cubemars2((uint16) slave3, 3);
         cubemars2.ConfigurePDOs();
         CubemarsMotor cubemars3((uint16) slave4, 4);
         cubemars3.ConfigurePDOs();
         CubemarsMotor cubemars4((uint16) slave5, 5);
         cubemars4.ConfigurePDOs();
         CubemarsMotor cubemars5((uint16) slave6, 6);
         cubemars5.ConfigurePDOs();
         CubemarsMotor cubemars6((uint16) slave7, 7);
         cubemars6.ConfigurePDOs();
         CubemarsMotor::ConfigureMotors();
         cout<<"*****Motors_Configured*****"<<endl;
         cubemars1.ConfigureCyclicSynchronousPositionMode();
         cubemars2.ConfigureCyclicSynchronousPositionMode();
         cubemars3.ConfigureCyclicSynchronousPositionMode();
         cubemars4.ConfigureCyclicSynchronousPositionMode();
         cubemars5.ConfigureCyclicSynchronousPositionMode();
         cubemars6.ConfigureCyclicSynchronousPositionMode();
         string command = "";
         dorun = 1;
         for(int i=0;i<5000;i++){
               CubemarsMotor::ReadEthercat();
               if(i==40){
                  cubemars1.SetTargetPosition((int32_t) 100000);
                  cubemars2.SetTargetPosition((int32_t) 100000);
                  cubemars3.SetTargetPosition((int32_t) 100000);
                  cubemars4.SetTargetPosition((int32_t) 100000);
                  cubemars5.SetTargetPosition((int32_t) 100000);
                  cubemars6.SetTargetPosition((int32_t) 100000);
                  command = "move to 10 rad";
               }
               else if(i==500){
                  cubemars1.SetTargetPosition((int32_t) 0);
                  cubemars2.SetTargetPosition((int32_t) 0);
                  cubemars3.SetTargetPosition((int32_t) 0);
                  cubemars4.SetTargetPosition((int32_t) 0);
                  cubemars5.SetTargetPosition((int32_t) 0);
                  cubemars6.SetTargetPosition((int32_t) 0);
                  command = "move to 0 rad";
               }
               else if(i==1500){
                  cubemars1.SetTargetPosition((int32_t) -100000);
                  cubemars2.SetTargetPosition((int32_t) -100000);
                  cubemars3.SetTargetPosition((int32_t) -100000);
                  cubemars4.SetTargetPosition((int32_t) -100000);
                  cubemars5.SetTargetPosition((int32_t) -100000);
                  cubemars6.SetTargetPosition((int32_t) -100000);
                  command = "move to -10 rad";
               }
               else if(i==2000){
                  cubemars1.SetModeOfOperation((int8_t) 0x0a);
                  cubemars2.SetModeOfOperation((int8_t) 0x0a);
                  cubemars3.SetModeOfOperation((int8_t) 0x0a);
                  cubemars4.SetModeOfOperation((int8_t) 0x0a);
                  cubemars5.SetModeOfOperation((int8_t) 0x0a);
                  cubemars6.SetModeOfOperation((int8_t) 0x0a);
               }
               else if(i==2100){
                  cubemars1.SetTargetTorque((int32_t) 110000);
                  cubemars2.SetTargetTorque((int32_t) 110000);
                  cubemars3.SetTargetTorque((int32_t) 110000);
                  cubemars4.SetTargetTorque((int32_t) 110000);
                  cubemars5.SetTargetTorque((int32_t) 110000);
                  cubemars6.SetTargetTorque((int32_t) 110000);
                  command = "do 15 Nm";
               }
               else if(i==4000){
                  cubemars1.SetTargetTorque((int32_t) 0);
                  cubemars2.SetTargetTorque((int32_t) 0);
                  cubemars3.SetTargetTorque((int32_t) 0);
                  cubemars4.SetTargetTorque((int32_t) 0);
                  cubemars5.SetTargetTorque((int32_t) 0);
                  cubemars6.SetTargetTorque((int32_t) 0);
                  command = "do 0 Nm";
               }
               else if(i==4100){
                  cubemars1.SetTargetTorque((int32_t) -110000);
                  cubemars2.SetTargetTorque((int32_t) -110000);
                  cubemars3.SetTargetTorque((int32_t) -110000);
                  cubemars4.SetTargetTorque((int32_t) -110000);
                  cubemars5.SetTargetTorque((int32_t) -110000);
                  cubemars6.SetTargetTorque((int32_t) -110000);
                  command = "do -15 Nm";
               }
               else if(i==4800){
                  cubemars1.SetTargetTorque((int32_t) 0);
                  cubemars2.SetTargetTorque((int32_t) 0);
                  cubemars3.SetTargetTorque((int32_t) 0);
                  cubemars4.SetTargetTorque((int32_t) 0);
                  cubemars5.SetTargetTorque((int32_t) 0);
                  cubemars6.SetTargetTorque((int32_t) 0);
                  command = "do 0 Nm";
               }
               else if(i==4900){
                  cubemars1.SetControlWord((uint16_t) 0x00007);
                  cubemars2.SetControlWord((uint16_t) 0x00007);
                  cubemars3.SetControlWord((uint16_t) 0x00007);
                  cubemars4.SetControlWord((uint16_t) 0x00007);
                  cubemars5.SetControlWord((uint16_t) 0x00007);
                  cubemars6.SetControlWord((uint16_t) 0x00007);
               }
               else if(i==5000){
                  cubemars1.SetControlWord((uint16_t) 0x00000);
                  cubemars2.SetControlWord((uint16_t) 0x00000);
                  cubemars3.SetControlWord((uint16_t) 0x00000);
                  cubemars4.SetControlWord((uint16_t) 0x00000);
                  cubemars5.SetControlWord((uint16_t) 0x00000);
                  cubemars6.SetControlWord((uint16_t) 0x00000);
               }
               cubemars1.WritingPDOs();
               cubemars2.WritingPDOs();
               cubemars3.WritingPDOs();
               cubemars4.WritingPDOs();
               cubemars5.WritingPDOs();
               cubemars6.WritingPDOs();
               cubemars1.ReadingPDOs();
               cubemars2.ReadingPDOs();
               cubemars3.ReadingPDOs();
               cubemars4.ReadingPDOs();
               cubemars5.ReadingPDOs();
               cubemars6.ReadingPDOs();
               CubemarsMotor::WriteEthercat();
               needlf = TRUE;
               osal_usleep(5000);
               system("clear");
               cout<<"Last command: "<<command<<endl;
               cout<<"Cubemars 1 status word: "<<dec<<cubemars1.GetStatusWord()<<endl;
               cout<<"Cubemars 1 Mode of Operation Display: "<<dec<<cubemars1.GetModeOfOperation()<<endl;
               cout<<"Cubemars 1 position actual value: "<<dec<<cubemars1.GetPositionActualValue()/10000.0<<" rad"<<endl;
               cout<<"Cubemars 1 torque actual value: "<<dec<<cubemars1.GetTorqueActualValue()/10000.0<<" Nm"<<endl;
               cout<<"Cubemars 2 status word: "<<dec<<cubemars2.GetStatusWord()<<endl;
               cout<<"Cubemars 2 Mode of Operation Display: "<<dec<<cubemars2.GetModeOfOperation()<<endl;
               cout<<"Cubemars 2 position actual value: "<<dec<<cubemars2.GetPositionActualValue()/10000.0<<" rad"<<endl;
               cout<<"Cubemars 2 torque actual value: "<<dec<<cubemars2.GetTorqueActualValue()/10000.0<<" Nm"<<endl;
               cout<<"Cubemars 3 status word: "<<dec<<cubemars3.GetStatusWord()<<endl;
               cout<<"Cubemars 3 Mode of Operation Display: "<<dec<<cubemars3.GetModeOfOperation()<<endl;
               cout<<"Cubemars 3 position actual value: "<<dec<<cubemars3.GetPositionActualValue()/10000.0<<" rad"<<endl;
               cout<<"Cubemars 3 torque actual value: "<<dec<<cubemars3.GetTorqueActualValue()/10000.0<<" Nm"<<endl;
               cout<<"Cubemars 4 status word: "<<dec<<cubemars4.GetStatusWord()<<endl;
               cout<<"Cubemars 4 Mode of Operation Display: "<<dec<<cubemars4.GetModeOfOperation()<<endl;
               cout<<"Cubemars 4 position actual value: "<<dec<<cubemars4.GetPositionActualValue()/10000.0<<" rad"<<endl;
               cout<<"Cubemars 4 torque actual value: "<<dec<<cubemars4.GetTorqueActualValue()/10000.0<<" Nm"<<endl;
               cout<<"Cubemars 5 status word: "<<dec<<cubemars5.GetStatusWord()<<endl;
               cout<<"Cubemars 5 Mode of Operation Display: "<<dec<<cubemars5.GetModeOfOperation()<<endl;
               cout<<"Cubemars 5 position actual value: "<<dec<<cubemars5.GetPositionActualValue()/10000.0<<" rad"<<endl;
               cout<<"Cubemars 5 torque actual value: "<<dec<<cubemars5.GetTorqueActualValue()/10000.0<<" Nm"<<endl;
               cout<<"Cubemars 6 status word: "<<dec<<cubemars6.GetStatusWord()<<endl;
               cout<<"Cubemars 6 Mode of Operation Display: "<<dec<<cubemars6.GetModeOfOperation()<<endl;
               cout<<"Cubemars 6 position actual value: "<<dec<<cubemars6.GetPositionActualValue()/10000.0<<" rad"<<endl;
               cout<<"Cubemars 6 torque actual value: "<<dec<<cubemars6.GetTorqueActualValue()/10000.0<<" Nm"<<endl;
               
            }
         cout<<"**********"<<endl;
         ec_slave[0].state = EC_STATE_INIT;
         ec_writestate(0);
         ec_close(); // closes the network
      }
    }
    else
    {
      printf("No socket connection on %s\nExecute as root\n",ifname);
    }

}

int main(int argc, char *argv[]) {
   dorun = 0;
   int ctime = 1000; // cycletime in microseconds
   if(argc > 1){
      // Real Time Thread cyclic loop
      osal_thread_create_rt(&thread1, 128000, &ecatthread, (void*) &ctime);
      // Network Control Thread
      osal_thread_create(&thread2, 128000, &ecatcheck, NULL);
      // Main excecution acyclic loop
      runner(argv[1]);
   }
   else{
      printf("no network port given\n");
   }
   return 1;
} 

