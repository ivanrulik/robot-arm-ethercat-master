// #include "ODVariable.h"

#include "Motor.h"
#include "ethercat.h"
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
         Motor maxon1((uint16) slave1, 1);
         maxon1.ConfigurePDOs();
         Motor maxon2((uint16) slave2, 2);
         maxon2.ConfigurePDOs();
         Motor::ConfigureMotors();
         cout<<"**********"<<endl;
         maxon1.ConfigureProfilePositionMode();
         maxon2.ConfigureProfilePositionMode();
         maxon1.SetTargetPosition((int32_t) 4800);
         maxon2.SetTargetPosition((int32_t) 2400);
         maxon1.SetControlWord((uint16_t) 0x003f);
         maxon2.SetControlWord((uint16_t) 0x003f);
         cout<<"Maxon 1 position actual value: "<<dec<<maxon1.GetPositionActualValue()<<endl;
         cout<<"Maxon 2 position actual value: "<<dec<<maxon2.GetPositionActualValue()<<endl;
         cout<<"Maxon 1 target position value: "<<dec<<maxon1.GetTargetPosition()<<endl;
         cout<<"Maxon 2 target position value: "<<dec<<maxon2.GetTargetPosition()<<endl;
         int cont = 3;
         do {
            for(int i = 0; i<cont; ++i){
                Motor::ReadEthercat();
                maxon1.WritingPDOs();
                maxon1.ReadingPDOs();
                maxon2.WritingPDOs();
                maxon2.ReadingPDOs();
                Motor::WriteEthercat();
                needlf = TRUE;
                osal_usleep(5000);
               //  cout<<"Maxon 1 position actual value: "<<dec<<maxon1.GetPositionActualValue()<<endl;
               //  cout<<"Maxon 2 position actual value: "<<dec<<maxon2.GetPositionActualValue()<<endl;
               //  cout<<"Maxon 1 target position value: "<<dec<<maxon1.GetTargetPosition()<<endl;
               //  cout<<"Maxon 2 target position value: "<<dec<<maxon2.GetTargetPosition()<<endl;
            }
            cont = 1;
         } 
         while ((maxon1.GetTargetPosition() != maxon1.GetPositionActualValue()) ||
                (maxon2.GetTargetPosition() != maxon2.GetPositionActualValue()));
         maxon1.SetControlWord((uint16_t) 0x000f);
         maxon2.SetControlWord((uint16_t) 0x000f);
         cont = 3;
         do {
               for(int i = 0; i<cont; ++i){
                Motor::ReadEthercat();
                maxon1.WritingPDOs();
                maxon1.ReadingPDOs();
                maxon2.WritingPDOs();
                maxon2.ReadingPDOs();
                Motor::WriteEthercat();
                needlf = TRUE;
                osal_usleep(5000);
                cout<<"Maxon 1 Status Word:"<<dec<< maxon1.GetStatusWord()<<endl;
                cout<<"Maxon 2 Status Word:"<<dec<< maxon2.GetStatusWord()<<endl;
               }
         } 
         while (!((maxon1.GetStatusWord() == 4663) || (maxon1.GetStatusWord() == 1591)) ||
                !((maxon2.GetStatusWord() == 4663) || (maxon2.GetStatusWord() == 1591)));
         maxon1.SetTargetPosition((int32_t) 0);
         maxon2.SetTargetPosition((int32_t) 0);
         maxon1.SetControlWord((uint16_t) 0x003f);
         maxon2.SetControlWord((uint16_t) 0x003f);
         cont = 3;
         do {
            for(int i = 0; i<cont; ++i){
                Motor::ReadEthercat();
                maxon1.WritingPDOs();
                maxon1.ReadingPDOs();
                maxon2.WritingPDOs();
                maxon2.ReadingPDOs();
                Motor::WriteEthercat();
                needlf = TRUE;
                osal_usleep(5000);
               //  cout<<"Maxon 1 position actual value: "<<dec<<maxon1.GetPositionActualValue()<<endl;
               //  cout<<"Maxon 2 position actual value: "<<dec<<maxon2.GetPositionActualValue()<<endl;
               //  cout<<"Maxon 1 target position value: "<<dec<<maxon1.GetTargetPosition()<<endl;
               //  cout<<"Maxon 2 target position value: "<<dec<<maxon2.GetTargetPosition()<<endl;
            }
            cont = 1;
         } 
         while ((maxon1.GetTargetPosition() != maxon1.GetPositionActualValue()) ||
                (maxon2.GetTargetPosition() != maxon2.GetPositionActualValue()));
         cout<<"**********"<<endl;
         cout<<"Maxon 1 position actual value: "<<dec<<maxon1.GetPositionActualValue()<<endl;
         cout<<"Maxon 2 position actual value: "<<dec<<maxon2.GetPositionActualValue()<<endl;
         cout<<"Maxon 1 target position value: "<<dec<<maxon1.GetTargetPosition()<<endl;
         cout<<"Maxon 2 target position value: "<<dec<<maxon2.GetTargetPosition()<<endl;
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