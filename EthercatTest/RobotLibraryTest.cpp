// #include "ODVariable.h"
#include "Robot.h"
#define EC_TIMEOUTMON 20000
using namespace std;
char IOmap[4096];
int usedmem;
OSAL_THREAD_HANDLE thread1;
int expectedWKC;
volatile int wkc;
boolean inOP;
uint8 currentgroup = 0;
int slave1 = 1;
int slave2 = 2;
int32_t positionValue; 
int16_t statusValue;
int8_t modeValue;
struct motorCommands
{
   uint16 removeError;
   uint16 prepare2Power;
   uint16 switchOn;
   uint16 operational;
   uint16 disable;
   uint16 move;
};
struct motorCommands epos4ControlWords = {0x0080,0x0006,0x0007,0x000f,0x0000,0x003f};
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

void pdo_test_cpp(char *ifname)
{
    if (ec_init(ifname))
    {
      if ( ec_config_init(FALSE) > 0 )
      {
         Robot testRobot;
         int desiredPosition[2]{1000, 500};
         int desiredTorque[2]{0, 0};
         int time{1500000};
         // testRobot.ConfigureProfilePositionMode();
         // testRobot.ProfilePositionControl(desiredPosition);
         // int desiredPosition2[2] {0, 0};
         // testRobot.ProfilePositionControl(desiredPosition2);
         testRobot.ConfigureCyclicSynchronousPositionMode();
         cout<<"**********"<<endl;
         testRobot.CyclicSynchronousPositionControl(desiredPosition, desiredTorque, time);
         int desiredPosition2[2] {0, 0};
         testRobot.CyclicSynchronousPositionControl(desiredPosition2, desiredTorque, time);
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
  if(argc > 1)
    {
      osal_thread_create(&thread1, 128000, &ecatcheck, NULL);
      pdo_test_cpp(argv[1]);
    }
    else
    {
      printf("no network port given\n");
    }
    return 1;
} 