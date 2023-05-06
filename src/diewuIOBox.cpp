/**
 * @file diewuIOBox.cpp
 * @authors Ivan Rulik, Javier Sanjuan (rulik@uwm.edu, jsanjuan@uwm.edu)
 * @brief EtherCAT code in C++ that uses SOEM to run an EtherCAt master, load PDO maps and test 1 EtherCAT slave
 * with Cyclic Synchronous Torque (CST) over using PDOs
 * @details to run this code you will need to run the following commands in terminal:
 * 1. cd ~/<wherever you stored SOEM folder>/SOEM/
 * 2. mkdir build
 * 3. cd build
 * 4. cmake ..
 * 5. make
 * 6. cd test/linux/diewuIOBox/
 * 7. sudo ./diewuIOBox.cpp <your network interface for EtherCAT>
 * 8. example ./diewuIOBox.cpp enp3s0 (use ifconfig command in terminal to find the name of your network interface for ethercat)
 * When you run the code properly you will get something like:
 * O: 00 00 00 00 0a I:40 02 8b 3b 00 00 0a statusword 576 Position Values 15243
 * @version 0.1
 * @date 2022-04-06
 * 
 * @copyright Copyright (c) 2022
 * 
 */
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <inttypes.h>
#include <stdlib.h>
#include "ethercat.h"
// Thread lib
#include <sched.h>
#include <sys/time.h>
#include <sys/sysinfo.h>
#include <time.h>
#include <math.h>
#include <pthread.h>

#define NSEC_PER_SEC 1000000000
#define EC_TIMEOUTMON 1000

struct ObjDict
{
   uint16 index;
   uint16 valueU16;
   int size;
   int8 value8;
   int32 value32;
   int16 value16;
   int size2;
   uint8 subindex;
};
struct ObjDict syncMan2 = {0x1c12,0x0000,0x10,0x00,0,0,0x08,0x00};
struct ObjDict syncMan3 = {0x1c13,0x0000,0x10,0x00,0,0,0x08,0x00};
struct ObjDict receivePDO1 = {0x1600,0x0000,0x20,0x00,0,0,0x08,0x00};
struct ObjDict transmitPDO1 = {0x1a00,0x0000,0x20,0x00,0,0,0x08,0x00};
struct ObjDict statusWord = {0x6041,0x0000,0x10,0x00,0,0,0,0x00};
struct ObjDict controlWord = {0x6040,0x0000,0x10,0x00,0,0,0,0x00};
struct ObjDict SM0_output = {0x3101,0x0000,0x08,0x00,0,0,0,0x01};
struct ObjDict SM1_output = {0x3101,0x0000,0x08,0x00,0,0,0,0x02};
struct ObjDict SM2_input = {0x3001,0x0000,0x08,0x00,0,0,0,0x01};
struct ObjDict SM2_input2 = {0x3001,0x0000,0x08,0x00,0,0,0,0x02};

char IOmap[4096];
int usedmem;
// OSAL_THREAD_HANDLE thread1;
int expectedWKC;
boolean needlf;
volatile int wkc;
boolean inOP;
uint8 currentgroup = 0;
// Thread vars
pthread_t thread1, thread2;
struct timeval tv, t1, t2;
int dorun = 0;
int64 toff, gl_delta;

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
  	*data_ptr++ = (value >> 0);// & 0xFF; *********
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
void set_output_uint32 (uint16_t slave_nb, uint8_t module_index, uint32_t value)
 {
  	uint8_t *data_ptr;
 
  	data_ptr = ec_slave[slave_nb].outputs;
  	/* Move pointer to correct module index*/
  	data_ptr += module_index * 2;
  	/* Read value byte by byte since all targets can't handle misaligned
     	   addresses */
  	*data_ptr++ = (value >> 0);// & 0xFF; *********
  	*data_ptr++ = (value >> 8);// & 0xFF00;
   *data_ptr++ = (value >> 16);// & 0xFF; *********
  	*data_ptr++ = (value >> 24);// & 0xFF00;
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
   uint16 slave = 1;
   int flip = 0;
   clock_gettime(CLOCK_MONOTONIC, &ts);
   ht = (ts.tv_nsec / 1000000) + 1; /* round to nearest ms */
   ts.tv_nsec = ht * 1000000;
   if (ts.tv_nsec >= NSEC_PER_SEC) {
      ts.tv_sec++;
      ts.tv_nsec -= NSEC_PER_SEC;
   }
   cycletime = *(int*)ptr * 1000; /* cycletime in, ns */
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
         /* if we have some digital output, cycle */
         // if( digout ) *digout = (uint8) ((dorun / 16) & 0xff);
         // printf("loop %d \n",dorun);
         if (dorun>50)
         {
            if(flip == 0)
            {
               set_output_int8(slave, (uint8_t) 0,(int8_t) 0b00000001);
               flip = 1;
            }
            else
            {
               set_output_int8(slave, (uint8_t) 0,(int8_t) 0b00000000);
               flip = 0;
            }
         }
         // if(dorun == 10)
         // {
         //    set_output_int8(slave, (uint8_t) 0,(int8_t) 0b00000001);
         // }
         // if(dorun == 60)
         // {
         //    set_output_int8(slave, (uint8_t) 0,(int8_t) 0b00000000);
         // }
         // if(dorun == 70)
         // {
         //    set_output_int8(slave, (uint8_t) 0,(int8_t) 0b00000001);
         // }
         // if(dorun == 1950)
         // {
         //    set_output_int8(slave, (uint8_t) 0,(int8_t) 0);
         // }
         if (ec_slave[0].hasdc)
         {
            /* calulate toff to get linux time and DC synced */
            ec_sync(ec_DCtime, cycletime, &toff);
         }
         ec_send_processdata();
      }
   }
}
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

void diewu_gpio(char *ifname)
{
   int cnt, i, j, oloop, iloop;
   int8_t SM2_input_value;
   int8_t SM2_input2_value; 
   int retval = 0;
   uint16 slave = 1;
   SM2_input_value = 0;
   SM2_input2_value = 0;
   printf("Starting Thread test\n");
   /* initialise SOEM, bind socket to ifname */
   // (void) ifname2;
   // if (ec_init_redundant(ifname, ifname2))
   if (ec_init(ifname))
   {
      printf("ec_init on %s succeeded.\n",ifname);
      /* find and auto-config slaves */
      if ( ec_config_init(FALSE) > 0 )
      {
         // epos4setup(slave);
         usedmem=ec_config_map(&IOmap);
         printf("Used memory by IO map %d\n",usedmem);
         printf("%d slaves found and configured.\n",ec_slavecount);
         /* wait for all slaves to reach SAFE_OP state */
         ec_statecheck(0, EC_STATE_SAFE_OP,  EC_TIMEOUTSTATE);
         /* configure DC options for every DC capable slave found in the list */
         ec_configdc();
         /* read indevidual slave state and store in ec_slave[] */
         ec_readstate();
         for(cnt = 1; cnt <= ec_slavecount ; cnt++)
         {
            printf("Slave:%d Name:%s Output size:%3dbits Input size:%3dbits State:%2d delay:%d.%d\n",
                  cnt, ec_slave[cnt].name, ec_slave[cnt].Obits, ec_slave[cnt].Ibits,
                  ec_slave[cnt].state, (int)ec_slave[cnt].pdelay, ec_slave[cnt].hasdc);
            printf("         Out:%p,%4d In:%p,%4d\n",
                  ec_slave[cnt].outputs, ec_slave[cnt].Obytes, ec_slave[cnt].inputs, ec_slave[cnt].Ibytes);
         }
         expectedWKC = (ec_group[0].outputsWKC * 2) + ec_group[0].inputsWKC;
         printf("Calculated workcounter %d\n", expectedWKC);

         printf("Request operational state for all slaves\n");
         ec_slave[0].state = EC_STATE_OPERATIONAL;
         /* request OP state for all slaves */
         ec_writestate(0);
         /* activate cyclic process data */
         dorun = 1;
         /* wait for all slaves to reach OP state */
         ec_statecheck(0, EC_STATE_OPERATIONAL,  5 * EC_TIMEOUTSTATE);
         oloop = ec_slave[0].Obytes;
         if ((oloop == 0) && (ec_slave[0].Obits > 0)) oloop = 1;
         if (oloop > 8) oloop = 8;
         iloop = ec_slave[0].Ibytes;
         if ((iloop == 0) && (ec_slave[0].Ibits > 0)) iloop = 1;
         if (iloop > 8) iloop = 8;
         if (ec_slave[0].state == EC_STATE_OPERATIONAL )
         {
            printf("Operational state reached for all slaves.\n");
            inOP = TRUE;
            /* acyclic loop 2000 x 5ms = 10s */
            for(i = 1; i <= 5500; i++)
            {
               printf("Processdata cycle %5d , Wck %3d, DCtime %12"PRId64", dt %12"PRId64", O:",dorun, wkc , ec_DCtime, gl_delta);
               for(j = 0 ; j < oloop; j++)
               {
                  printf(" %2.2x", *(ec_slave[0].outputs + j));
               }
               printf(" I:");
               for(j = 0 ; j < iloop; j++)
               {
                  printf(" %2.2x", *(ec_slave[0].inputs + j));
               }
               printf("\r");
               SM2_input_value = 0;
               // SM2_input2_value = 0;
               get_input_int8(slave, (uint8_t) 0,&SM2_input_value);
               // // get_input_int32(slave, (uint8_t) 3,&SM2_input2_value);
               
               printf("\nSM2_input_value %d ", SM2_input_value);
               // // printf("SM2_input2_value %d\n", SM2_input2_value);
               fflush(stdout);
               osal_usleep(5000);//5ms
               // printf("\r");
            }
            dorun = 0;
            inOP = FALSE;
         }
         else
         {
            printf("Not all slaves reached operational state.\n");
             ec_readstate();
             for(i = 1; i<=ec_slavecount ; i++)
             {
                 if(ec_slave[i].state != EC_STATE_OPERATIONAL)
                 {
                     printf("Slave %d State=0x%2.2x StatusCode=0x%4.4x : %s\n",
                         i, ec_slave[i].state, ec_slave[i].ALstatuscode, ec_ALstatuscode2string(ec_slave[i].ALstatuscode));
                 }
             }
         }
         printf("Request safe operational state for all slaves\n");
         ec_slave[0].state = EC_STATE_SAFE_OP;
         /* request SAFE_OP state for all slaves */
         ec_writestate(0);
      }
      else
      {
         printf("No slaves found!\n");
      }
      printf("End redundant test, close socket\n");
      /* stop SOEM, close socket */
      ec_close();
   }
   else
   {
      printf("No socket connection on %s\nExcecute as root\n",ifname);
   }
}

#define stack64k (64 * 1024)
int main(int argc, char *argv[])
{
   dorun = 0;
   int ctime = 5000; // cycletime in microseconds
   if(argc > 1)
   {
      osal_thread_create_rt(&thread1, 128000, &ecatthread, (void*) &ctime);
      osal_thread_create(&thread2, 128000, &ecatcheck, NULL);
      diewu_gpio(argv[1]);
   }
   else
   {
      printf("no network port given\n");
   }
   printf("End program\n");
   return 1;
}