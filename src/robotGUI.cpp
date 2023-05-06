#include <gtkmm.h>
#include "WMAR.h"
#include "csvfile.h"
/*
	EtherCAT
*/ 
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
int32_t positionValue; 
int16_t statusValue;
int8_t modeValue;
boolean needlf;
int dorun = 0;
int64 toff, gl_delta;
/*
	Inverse Kinematics
*/
WMAR Robot;
double theta[6];
double thetaD[6];
double torque[6];
double cartesianTarget[6];
double FW[6];
double theta_ik[6];
unsigned int robot_theta_ik[6];
bool valid_ik;
bool ik_sent = FALSE;
/*
	EtherCAT
*/

// GUI control
bool STOP_ECAT = TRUE;
char *ifname;
int slaveNumber = 0;
unsigned int targetJointVal[6] ={0,0,0,0,0,0};
unsigned int target1Val = 0;
unsigned int target2Val = 0;
unsigned int target3Val = 0;
unsigned int target4Val = 0;
unsigned int target5Val = 0;
unsigned int target6Val = 0;
int modeGUI = 0;
int lowLim[6]={-1,-1,-1,-1,-1,-1};
int upLim[6]={1,1,1,1,1,1};
int limCounter = 0;
// Data logger
bool recordData = FALSE;
int logCounter = 0;
int counter4Counter = 0;
string logFileName = "tempData.csv";
int ecatSleepInUS = 3000;
int sampleFactor = 100;
double timeFactor = sampleFactor*(ecatSleepInUS/1000000.0);
// Joystick Mode
int8_t jsRead = 0;
int8_t jsReadPast = 0;
int jsMode = 0;
string jsModeName = "";
double jsFW[6];
double jsIK[6];
double linStepSize = 5;
double rotStepSize = 0.05;
bool moving = false;
int movingCounter =0;


using namespace std;
using namespace Gtk;


// Record Data
void dataRecorder()
{
	if(recordData)
	{
		counter4Counter++;
	}
	else
	{
		logCounter = 0;
		counter4Counter = 0;
	}
	if(counter4Counter>=sampleFactor)
	{
		logCounter++;
		counter4Counter = 0 ;
		if(logCounter == 1)
		{
			printf("started logging %d \n",logCounter);
			csvfile csv(logFileName,false); // throws exceptions!
			// Hearer
			csv <<"Time" << "J1_pos" << "J2_pos" << "J3_pos" << "J4_pos" << "J5_pos" << "J6_pos" <<
						   	"J1_vel" << "J2_vel" << "J3_vel" << "J4_vel" << "J5_vel" << "J6_vel" <<
							"J1_tor" << "J2_tor" << "J3_tor" << "J4_tor" << "J5_tor" << "J6_tor" <<
							"J1_tpos" << "J2_tpos" << "J3_tpos" << "J4_tpos" << "J5_tpos" << "J6_tpos" << endrow;
			logCounter++;
		}
		else if(logCounter>1)
		{
			csvfile csv(logFileName,true); // throws exceptions!
			// Hearer
			if(modeGUI == 1)
			{
				csv <<logCounter*timeFactor << 	theta[0]  << theta[1] 	<< theta[2]  	<< theta[3]  << theta[4]   << theta[5]  << 
										   		thetaD[0] << thetaD[1] 	<< thetaD[2] 	<< thetaD[3] << thetaD[4]  << thetaD[5] <<
										   		torque[0] << torque[1] 	<< torque[2] 	<< torque[3] << torque[4]  << torque[5] << 
												targetJointVal[0]/10000.0	<< targetJointVal[1]/10000.0 	<< targetJointVal[2]/10000.0	<< targetJointVal[3]/10000.0 	<< targetJointVal[4]/10000.0 	<< targetJointVal[5]/10000.0	<<	endrow;
			}
			else if(modeGUI == 2)
			{
				csv <<logCounter*timeFactor <<  theta[0]  	<< theta[1] 	<< theta[2]  	<< theta[3]  	<< theta[4]  	<< theta[5] 	<< 
												thetaD[0] 	<< thetaD[1] 	<< thetaD[2]  	<< thetaD[3] 	<< thetaD[4]  	<< thetaD[5] 	<<
												torque[0] 	<< torque[1] 	<< torque[2] 	<< torque[3] 	<< torque[4]  	<< torque[5] 	<< 
												theta_ik[0]	<< theta_ik[1] 	<< theta_ik[2]	<< theta_ik[3] 	<< theta_ik[4] 	<< theta_ik[5]	<<	endrow;
			}
			else if(modeGUI == 3)
			{
				csv <<logCounter*timeFactor << 	theta[0]  << theta[1] 	<< theta[2]  	<< theta[3]  << theta[4]   << theta[5]  << 
										   		thetaD[0] << thetaD[1] 	<< thetaD[2]  	<< thetaD[3] << thetaD[4]  << thetaD[5] <<
										   		torque[0] << torque[1] 	<< torque[2] 	<< torque[3] << torque[4]  << torque[5] << 
												jsIK[0]	<< jsIK[1] 	<< jsIK[2]	<< jsIK[3] 	<< jsIK[4] 	<< jsIK[5]	<<	endrow;
			}
			
		}
	}
	
}


// jsModeControl
int jsModeControl()
{
	// if(jsRead == 63)
	// {
	// 	moving = false;
	// }
	if(jsRead != 63 && jsRead != 47 && jsRead != 31 && jsRead != 0)
	{
		printf("jsRead: %d\n",jsRead);
		if (movingCounter> 10)
		{
			moving = false;
			movingCounter=0;
			
		}
		if (moving == false)
		{
			Robot.ForwardKinematics( theta, jsFW );
		}
		else{
			movingCounter ++;
		}
		
		if(jsRead == 62) //Fwd
		{
			if(jsMode == 0)
			{
				jsFW[0] = jsFW[0] + linStepSize;
			}
			else if(jsMode == 1)
			{
				jsFW[2] = jsFW[2] - linStepSize;
			}
			else if(jsMode == 2)
			{
				jsFW[3] = jsFW[3] + rotStepSize;
			}
			moving = true;
		}
		else if(jsRead == 61) //Bwd
		{
			if(jsMode == 0)
			{
				jsFW[0] = jsFW[0] - linStepSize;
			}
			else if(jsMode == 1)
			{
				jsFW[2] = jsFW[2] + linStepSize;
			}
			else if(jsMode == 2)
			{
				jsFW[3] = jsFW[3] - rotStepSize;
			}
			moving = true;
		}
		if(jsRead == 59) //Lft
		{
			if(jsMode == 0)
			{
				jsFW[1] = jsFW[1] - linStepSize;
			}
			else if(jsMode == 1)
			{
				jsFW[5] = jsFW[5] + rotStepSize;
			}
			else if(jsMode == 2)
			{
				jsFW[4] = jsFW[4] + rotStepSize;
			}
			moving = true;
		}
		else if(jsRead == 55) //Rght
		{
			if(jsMode == 0)
			{
				jsFW[1] = jsFW[1] + linStepSize;
			}
			else if(jsMode == 1)
			{
				jsFW[5] = jsFW[5] - rotStepSize;

			}
			else if(jsMode == 2)
			{
				jsFW[4] = jsFW[4] - rotStepSize;
			}
			moving = true;
		}
		valid_ik = Robot.InverseKinematics( jsFW, jsIK );
		if(valid_ik && moving == true)
		{
			robot_theta_ik[0] = int(jsIK[0]*10000);
			robot_theta_ik[1] = int(jsIK[1]*10000);
			robot_theta_ik[2] = int(jsIK[2]*10000);
			robot_theta_ik[3] = int(jsIK[3]*10000);
			robot_theta_ik[4] = int(jsIK[4]*10000);
			robot_theta_ik[5] = int(jsIK[5]*10000);
			return 1;
		}
		else
		{
			return 0;
		}
	}
	else if (jsRead==63)
	{
		if(jsReadPast == 47) //spUp
		{
			jsMode ++;
			printf("jsRead: %d\n",jsRead);
			printf("jsMode: %d\n",jsMode);
		}
		else if(jsReadPast == 31) //spDown
		{
			jsMode --;
			printf("jsRead: %d\n",jsRead);
			printf("jsMode: %d\n",jsMode);
		}
		if(jsMode>2)
		{
			jsMode = 0;
			printf("jsMode: %d\n",jsMode);
		}
		else if (jsMode< 0)
		{
			jsMode = 2;
			printf("jsMode: %d\n",jsMode);
		}
		moving = false;
		
		return 0;
	}
}

// Limit detection
void limitBreak(int val1,int val2,int val3,int val4,int val5,int val6)
{
	if(val1<lowLim[0] || val1>upLim[0] ||
	   val2<lowLim[1] || val2>upLim[1] ||
	   val3<lowLim[2] || val3>upLim[2] ||
	   val4<lowLim[3] || val4>upLim[3] ||
	   val5<lowLim[4] || val5>upLim[4] ||
	   val6<lowLim[5] || val6>upLim[5])
	{
		limCounter++;
		if(limCounter >=8)
		{
			STOP_ECAT = TRUE;
		}
		
	}
	else 
	{
		limCounter = 0;
	}
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


// Real Time Thread Block
OSAL_THREAD_FUNC ecatcheck(void *ptr)
{
    int slave;
    printf("Thread on\n");
    // printf("%d\n",ec_group[currentgroup].docheckstate);
    while(STOP_ECAT == FALSE)
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
OSAL_THREAD_FUNC runner(void *ptr)
{
    if (ec_init(ifname))
    {
      if ( ec_config_init(FALSE) > 0 )
      {
         int32_t positionValue; 
         int32_t velocityValue;
         int32_t torqueValue;
         int16_t statusValue;
         int8_t modeValue;
		 int jsRet = 0;
		 boolean STOP_runner = FALSE;
         // Motor initialization
         CubemarsMotor cubemars1((uint16) 2,(int) 2);
		 CubemarsMotor cubemars2((uint16) 3,(int) 3);
		 CubemarsMotor cubemars3((uint16) 4,(int) 4);
		 CubemarsMotor cubemars4((uint16) 5,(int) 5);
		 CubemarsMotor cubemars5((uint16) 6,(int) 6);
		 CubemarsMotor cubemars6((uint16) 7,(int) 7);
         cubemars1.ConfigurePDOs();
		 cubemars2.ConfigurePDOs();
		 cubemars3.ConfigurePDOs();
		 cubemars4.ConfigurePDOs();
		 cubemars5.ConfigurePDOs();
		 cubemars6.ConfigurePDOs();
         CubemarsMotor::ConfigureMotors();
         cout<<"*****Motors_Configured*****"<<endl;
         string command = "";
         dorun = 1;
		 int i = 1;
		 int j = 1;
         while(STOP_runner == FALSE)
		 {
			i++;
			ec_send_processdata();
			wkc = ec_receive_processdata(EC_TIMEOUTRET);
			if(wkc >= expectedWKC)
			{
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
				if(STOP_ECAT == TRUE){
					cubemars1.SetTargetVelocity((int32_t) 0);
					cubemars2.SetTargetVelocity((int32_t) 0);
					cubemars3.SetTargetVelocity((int32_t) 0);
					cubemars4.SetTargetVelocity((int32_t) 0);
					cubemars5.SetTargetVelocity((int32_t) 0);
					cubemars6.SetTargetVelocity((int32_t) 0);
					cubemars1.SetControlWord((uint16_t) 0x07);
					cubemars2.SetControlWord((uint16_t) 0x07);
					cubemars3.SetControlWord((uint16_t) 0x07);
					cubemars4.SetControlWord((uint16_t) 0x07);
					cubemars5.SetControlWord((uint16_t) 0x07);
					cubemars6.SetControlWord((uint16_t) 0x07);
					j++;
				}
				if(j >50){
					STOP_runner = TRUE;
				}
				if(i >= 100 && i <=140)
				{
					cubemars1.SetControlWord((uint16_t) 0x07);
					cubemars2.SetControlWord((uint16_t) 0x07);
					cubemars3.SetControlWord((uint16_t) 0x07);
					cubemars4.SetControlWord((uint16_t) 0x07);
					cubemars5.SetControlWord((uint16_t) 0x07);
					cubemars6.SetControlWord((uint16_t) 0x07);
					cubemars1.SetModeOfOperation((int8_t) 0x08);
					cubemars2.SetModeOfOperation((int8_t) 0x08);
					cubemars3.SetModeOfOperation((int8_t) 0x08);
					cubemars4.SetModeOfOperation((int8_t) 0x08);
					cubemars5.SetModeOfOperation((int8_t) 0x08);
					cubemars6.SetModeOfOperation((int8_t) 0x08);
					cubemars1.SetTargetPosition((int32_t) 0);
					cubemars2.SetTargetPosition((int32_t) 0);
					cubemars3.SetTargetPosition((int32_t) 0);
					cubemars4.SetTargetPosition((int32_t) 0);
					cubemars5.SetTargetPosition((int32_t) 0);
					cubemars6.SetTargetPosition((int32_t) 0);
					cubemars1.SetTargetVelocity((int32_t) 2000);
					cubemars2.SetTargetVelocity((int32_t) 2000);
					cubemars3.SetTargetVelocity((int32_t) 2000);
					cubemars4.SetTargetVelocity((int32_t) 2000);
					cubemars5.SetTargetVelocity((int32_t) 2000);
					cubemars6.SetTargetVelocity((int32_t) 2000);
				}

				if(i >= 150 && i <= 170)
				{
					cubemars1.SetControlWord((uint16_t) 0x1f);
					cubemars2.SetControlWord((uint16_t) 0x1f);
					cubemars3.SetControlWord((uint16_t) 0x1f);
					cubemars4.SetControlWord((uint16_t) 0x1f);
					cubemars5.SetControlWord((uint16_t) 0x1f);
					cubemars6.SetControlWord((uint16_t) 0x1f);
				}
			
				if(i>200)
				{
					if(modeGUI == 1)
					{
						cubemars1.SetTargetPosition((int32_t) targetJointVal[0]);
						cubemars2.SetTargetPosition((int32_t) targetJointVal[1]);
						cubemars3.SetTargetPosition((int32_t) targetJointVal[2]);
						cubemars4.SetTargetPosition((int32_t) targetJointVal[3]);
						cubemars5.SetTargetPosition((int32_t) targetJointVal[4]);
						cubemars6.SetTargetPosition((int32_t) targetJointVal[5]);
						command = "GUI Joint mode";
					}
					else if(modeGUI == 2)
					{
						if(valid_ik == TRUE && ik_sent == TRUE)
						{
							cout<<"ROBOT "<<theta_ik[0]<<" "<<theta_ik[1]<<" "<<theta_ik[2]<<" "<<theta_ik[3]<<" "<<theta_ik[4]<<" "<<theta_ik[5]<<endl;
							cubemars1.SetTargetPosition((int32_t)robot_theta_ik[0]);
							cubemars2.SetTargetPosition((int32_t)robot_theta_ik[1]);
							cubemars3.SetTargetPosition((int32_t)robot_theta_ik[2]);
							cubemars4.SetTargetPosition((int32_t)robot_theta_ik[3]);
							cubemars5.SetTargetPosition((int32_t)robot_theta_ik[4]);
							cubemars6.SetTargetPosition((int32_t)robot_theta_ik[5]);
							ik_sent = FALSE;
							command = "GUI Cartesian mode";
						}
						else
						{
							command = "ERROR: Singularity Triggered";
						}
					}
					else if(modeGUI == 3)
					{
						// SM2_input2_value = 0;
						jsReadPast = jsRead;
						jsRead = 0;
						get_input_int8((int) 1, (uint8_t) 0,&jsRead);
						jsRet = jsModeControl();
						if(jsRet==1)
						{
							cubemars1.SetTargetPosition((int32_t)robot_theta_ik[0]);
							cubemars2.SetTargetPosition((int32_t)robot_theta_ik[1]);
							cubemars3.SetTargetPosition((int32_t)robot_theta_ik[2]);
							cubemars4.SetTargetPosition((int32_t)robot_theta_ik[3]);
							cubemars5.SetTargetPosition((int32_t)robot_theta_ik[4]);
							cubemars6.SetTargetPosition((int32_t)robot_theta_ik[5]);
						}
						jsRet = 0;
						command = "JS Joint mode";
						// fflush(stdout);
					}
				}
				
				// system("clear");
				// cout<<"Last command: "<<command<<endl;
				// cout<<"Cubemars 1 status word: "<<dec<<cubemars1.GetStatusWord()<<endl;
				// cout<<"Cubemars 1 Mode of Operation Display: "<<dec<<cubemars1.GetModeOfOperationDisplay()<<endl;
				// cout<<theta_ik[0]<<" "<<theta_ik[1]<<" "<<theta_ik[2]<<" "<<theta_ik[3]<<" "<<theta_ik[4]<<" "<<theta_ik[5]<<endl;
				
				// printf("cnt %d\n", i);
				theta[0]=cubemars1.GetPositionActualValue()/10000.0;
				theta[1]=cubemars2.GetPositionActualValue()/10000.0;
				theta[2]=cubemars3.GetPositionActualValue()/10000.0;
				theta[3]=cubemars4.GetPositionActualValue()/10000.0;
				theta[4]=cubemars5.GetPositionActualValue()/10000.0;
				theta[5]=cubemars6.GetPositionActualValue()/10000.0;
				thetaD[0]=cubemars1.GetVelocityActualValue()/10000.0;
				thetaD[1]=cubemars2.GetVelocityActualValue()/10000.0;
				thetaD[2]=cubemars3.GetVelocityActualValue()/10000.0;
				thetaD[3]=cubemars4.GetVelocityActualValue()/10000.0;
				thetaD[4]=cubemars5.GetVelocityActualValue()/10000.0;
				thetaD[5]=cubemars6.GetVelocityActualValue()/10000.0;
				torque[0]=cubemars1.GetTorqueActualValue()/10000.0;
				torque[1]=cubemars2.GetTorqueActualValue()/10000.0;
				torque[2]=cubemars3.GetTorqueActualValue()/10000.0;
				torque[3]=cubemars4.GetTorqueActualValue()/10000.0;
				torque[4]=cubemars5.GetTorqueActualValue()/10000.0;
				torque[5]=cubemars6.GetTorqueActualValue()/10000.0;
				limitBreak(thetaD[0],thetaD[1],thetaD[2],thetaD[3],thetaD[4],thetaD[5]);
				dataRecorder();
				needlf = TRUE;
			}
			osal_usleep(ecatSleepInUS);
        }
         ec_slave[0].state = EC_STATE_INIT;
         /* request INIT state for all slaves */
         ec_writestate(0);
         ec_close(); // closes the network
		 STOP_ECAT = TRUE;
      }
   }
   else
   {
      printf("No socket connection on %s\nExecute as root\n",ifname);
   }
   
}

class GTKRobot : public Window
{
	// controls
	public:
		Grid gridLayout;
		Grid gridTargetValues;
		Grid gridCartesianValues;
		Grid gridCartesianButtons;
		VBox vBoxJointLabels;
		VBox vBoxJointValues;
		VBox vBoxJointActualValues;
		VBox vBoxSetup;
		VBox vBoxTarget;
		VBox vBoxCartesian;
		VBox vBoxCartesianLabels;
		VBox vBoxCartesianValues;
		VBox vBoxCartesianActualValues;
		VBox vBoxDataLogger;
		Label labelNetworkInterface;
		Label labelNofSlaves;
		Label labelmodeOfOp;
		Label labelNetworkControl;
		Label labelRobotstatus;
		Label labelTargetJoint;
		Label labelCartesianMode;
		Label labelTargetPosition;
		Label labelTargetPositionCartesian;
		Label labelJoint1;
		Label labelJoint2;
		Label labelJoint3;
		Label labelJoint4;
		Label labelJoint5;
		Label labelJoint6;
		Label labelDX;
		Label labelDY;
		Label labelDZ;
		Label labelRX;
		Label labelRY;
		Label labelRZ;
		Label labelDataLogger;
		Label labelDataLoggerCurrent;
		Entry entryNetworkInterface;
		Entry entryNofslaves;
		Entry entryRTloopTime;
		Entry entryTargetValue1;
		Entry entryTargetValue2;
		Entry entryTargetValue3;
		Entry entryTargetValue4;
		Entry entryTargetValue5;
		Entry entryTargetValue6;
		Entry entryActualValue1;
		Entry entryActualValue2;
		Entry entryActualValue3;
		Entry entryActualValue4;
		Entry entryActualValue5;
		Entry entryActualValue6;
		Entry entryTargetDX;
		Entry entryTargetDY;
		Entry entryTargetDZ;
		Entry entryTargetRX;
		Entry entryTargetRY;
		Entry entryTargetRZ;
		Entry entryActualDX;
		Entry entryActualDY;
		Entry entryActualDZ;
		Entry entryActualRX;
		Entry entryActualRY;
		Entry entryActualRZ;
		Entry entryDataLogger;
		Entry entryDataLoggerCurrent;
		ComboBoxText comboBoxModeofOp;
		ComboBoxText comboBoxTargetJoint;
		ComboBoxText comboBoxTargetCartesian;
		CheckButton checkButtonDataLogger;
		Button buttonStartETC;
		Button buttonStopETC;
		Button buttonModeofOp;
		Button buttonSendTarget;
		Button buttonSendTargetCartesian;
		Button buttonSendTestCartesian;
		Button buttonDataLoggerSave;

	// methods

	public: GTKRobot()
	{
		set_border_width(30);
		// gridLayout
		add(gridLayout);
		gridLayout.set_column_spacing(10);
		gridLayout.set_row_spacing(15);
		gridLayout.show();
		// vBoxSetup
		gridLayout.add(vBoxSetup);
		vBoxSetup.show();
		// vBoxTarget
		gridLayout.add(vBoxTarget);
		vBoxTarget.show();
		// vBoxCartesian
		gridLayout.add(vBoxCartesian);
		vBoxCartesian.show();
		// vBoxDataLogger
		gridLayout.add(vBoxDataLogger);
		vBoxDataLogger.show();

		/*
			vBoxSetup
		*/

		// labelNetworkInterface
		labelNetworkInterface.set_label("  Type Network Interface:  ");
		vBoxSetup.add(labelNetworkInterface);
		labelNetworkInterface.show();

		// entryNetworkInterface
		entryNetworkInterface.set_text("enp3s0");
		vBoxSetup.add(entryNetworkInterface);
		entryNetworkInterface.show();

		// labelNofSlaves
		labelNofSlaves.set_label("Amount of motors:");
		vBoxSetup.add(labelNofSlaves);
		labelNofSlaves.show();

		// entryNofSlaves
		entryNofslaves.set_text("2");
		vBoxSetup.add(entryNofslaves);
		entryNofslaves.show();

		// labelmodeOfOp
		labelmodeOfOp.set_label("Mode of Operation:");
		vBoxSetup.add(labelmodeOfOp);
		labelmodeOfOp.show();

		// comboModeofOp
		comboBoxModeofOp.append("CSP");
		comboBoxModeofOp.append("Cartesian");
		comboBoxModeofOp.append("Joystick");
		comboBoxModeofOp.set_active(0);
		vBoxSetup.add(comboBoxModeofOp);
		comboBoxModeofOp.show();

		// buttonModeofOp
		buttonModeofOp.set_label("Change Mode");
		buttonModeofOp.signal_clicked().connect
		(
			sigc::mem_fun
			(
				*this,
				&GTKRobot::buttonModeofOpClicked
			)
		);
		vBoxSetup.add(buttonModeofOp);
		buttonModeofOp.show();

		// labelNetworkControl
		labelNetworkControl.set_label("Network Control");
		vBoxSetup.add(labelNetworkControl);
		labelNetworkControl.show();

		// buttonStartETC
		buttonStartETC.set_label("Start Network");
		buttonStartETC.signal_clicked().connect
		(
			sigc::mem_fun
			(
				*this,
				&GTKRobot::buttonStartETCClicked
			)
		);
		vBoxSetup.add(buttonStartETC);
		buttonStartETC.show();

		// buttonStopETC
		buttonStopETC.set_label("Stop Network");
		buttonStopETC.signal_clicked().connect
		(
			sigc::mem_fun
			(
				*this,
				&GTKRobot::buttonStopETCClicked
			)
		);
		vBoxSetup.add(buttonStopETC);
		buttonStopETC.show();

		/*
			vBoxTarget
		*/
		// labelTargetJoint
		labelTargetJoint.set_label("  Joint Control  ");
		vBoxTarget.add(labelTargetJoint);
		labelTargetJoint.show();
		// comboTargetJoint
		comboBoxTargetJoint.append("1");
		comboBoxTargetJoint.append("2");
		comboBoxTargetJoint.append("3");
		comboBoxTargetJoint.append("4");
		comboBoxTargetJoint.append("5");
		comboBoxTargetJoint.append("6");
		comboBoxTargetJoint.set_active(0);
		vBoxTarget.add(comboBoxTargetJoint);
		comboBoxTargetJoint.show();
		// labelTargetPosition
		labelTargetPosition.set_label("  Target Position [rad]  ");
		vBoxTarget.add(labelTargetPosition);
		labelTargetPosition.show();

		vBoxTarget.add(gridTargetValues);
		gridTargetValues.set_column_spacing(10);
		gridTargetValues.set_row_spacing(15);
		gridTargetValues.show();
		gridTargetValues.add(vBoxJointLabels);
		vBoxJointLabels.show();
		gridTargetValues.add(vBoxJointValues);
		vBoxJointValues.show();
		gridTargetValues.add(vBoxJointActualValues);
		vBoxJointActualValues.show();

		// labelJoint1
		labelJoint1.set_label("Joint 1");
		vBoxJointLabels.add(labelJoint1);
		labelJoint1.show();
		// labelJoint2
		labelJoint2.set_label("Joint 2");
		vBoxJointLabels.add(labelJoint2);
		labelJoint2.show();
		// labelJoint3
		labelJoint3.set_label("Joint 3");
		vBoxJointLabels.add(labelJoint3);
		labelJoint3.show();
		// labelJoint4
		labelJoint4.set_label("Joint 4");
		vBoxJointLabels.add(labelJoint4);
		labelJoint4.show();
		// labelJoint5
		labelJoint5.set_label("Joint 5");
		vBoxJointLabels.add(labelJoint5);
		labelJoint5.show();
		// labelJoint6
		labelJoint6.set_label("Joint 6");
		vBoxJointLabels.add(labelJoint6);
		labelJoint6.show();

		// entryTargetValue1
		entryTargetValue1.set_text("0.0");
		vBoxJointValues.add(entryTargetValue1);
		entryTargetValue1.show();
		// entryTargetValue2
		entryTargetValue2.set_text("0.0");
		vBoxJointValues.add(entryTargetValue2);
		entryTargetValue2.show();
		// entryTargetValue3
		entryTargetValue3.set_text("0.0");
		vBoxJointValues.add(entryTargetValue3);
		entryTargetValue3.show();
		// entryTargetValue4
		entryTargetValue4.set_text("0.0");
		vBoxJointValues.add(entryTargetValue4);
		entryTargetValue4.show();
		// entryTargetValue5
		entryTargetValue5.set_text("0.0");
		vBoxJointValues.add(entryTargetValue5);
		entryTargetValue5.show();
		// entryTargetValue6
		entryTargetValue6.set_text("0.0");
		vBoxJointValues.add(entryTargetValue6);
		entryTargetValue6.show();

		// entryActualValue1
		entryActualValue1.set_text("");
		vBoxJointActualValues.add(entryActualValue1);
		entryActualValue1.set_editable(FALSE);
		entryActualValue1.show();
		// entryActualValue2
		entryActualValue2.set_text("");
		vBoxJointActualValues.add(entryActualValue2);
		entryActualValue2.set_editable(FALSE);
		entryActualValue2.show();
		// entryActualValue3
		entryActualValue3.set_text("");
		vBoxJointActualValues.add(entryActualValue3);
		entryActualValue3.set_editable(FALSE);
		entryActualValue3.show();
		// entryActualValue4
		entryActualValue4.set_text("");
		vBoxJointActualValues.add(entryActualValue4);
		entryActualValue4.set_editable(FALSE);
		entryActualValue4.show();
		// entryActualValue5
		entryActualValue5.set_text("");
		vBoxJointActualValues.add(entryActualValue5);
		entryActualValue5.set_editable(FALSE);
		entryActualValue5.show();
		// entryActualValue6
		entryActualValue6.set_text("");
		vBoxJointActualValues.add(entryActualValue6);
		entryActualValue6.set_editable(FALSE);
		entryActualValue6.show();

		// buttonSendTarget
		buttonSendTarget.set_label("Send Joint Target");
		buttonSendTarget.signal_clicked().connect
		(
			sigc::mem_fun
			(
				*this,
				&GTKRobot::buttonSendTargetClicked
			)
		);
		vBoxTarget.add(buttonSendTarget);
		buttonSendTarget.show();

		/*
			vBoxCartesian
		*/
		// labelCartesianMode
		labelCartesianMode.set_label("  Cartesian Mode  ");
		vBoxCartesian.add(labelCartesianMode);
		labelCartesianMode.show();
		// comboTargetJoint
		comboBoxTargetCartesian.append("dX");
		comboBoxTargetCartesian.append("dY");
		comboBoxTargetCartesian.append("dZ");
		comboBoxTargetCartesian.append("rX");
		comboBoxTargetCartesian.append("rY");
		comboBoxTargetCartesian.append("rZ");
		comboBoxTargetCartesian.set_active(0);
		vBoxCartesian.add(comboBoxTargetCartesian);
		comboBoxTargetCartesian.show();
		// labelTargetPosition
		labelTargetPositionCartesian.set_label("  Target Position [mm]  ");
		vBoxCartesian.add(labelTargetPositionCartesian);
		labelTargetPositionCartesian.show();

		vBoxCartesian.add(gridCartesianValues);
		gridCartesianValues.set_column_spacing(10);
		gridCartesianValues.set_row_spacing(15);
		gridCartesianValues.show();
		gridCartesianValues.add(vBoxCartesianLabels);
		vBoxCartesianLabels.show();
		gridCartesianValues.add(vBoxCartesianValues);
		vBoxCartesianValues.show();
		gridCartesianValues.add(vBoxCartesianActualValues);
		vBoxCartesianActualValues.show();

		// labelJoint1
		labelDX.set_label("dX");
		vBoxCartesianLabels.add(labelDX);
		labelDX.show();
		// labelJoint2
		labelDY.set_label("dY");
		vBoxCartesianLabels.add(labelDY);
		labelDY.show();
		// labelJoint3
		labelDZ.set_label("dZ");
		vBoxCartesianLabels.add(labelDZ);
		labelDZ.show();
		// labelJoint4
		labelRX.set_label("rX");
		vBoxCartesianLabels.add(labelRX);
		labelRX.show();
		// labelJoint5
		labelRY.set_label("rY");
		vBoxCartesianLabels.add(labelRY);
		labelRY.show();
		// labelJoint6
		labelRZ.set_label("rZ");
		vBoxCartesianLabels.add(labelRZ);
		labelRZ.show();

		// entryTargetValue1
		entryTargetDX.set_text("0.0");
		vBoxCartesianValues.add(entryTargetDX);
		entryTargetDX.show();
		// entryTargetValue2
		entryTargetDY.set_text("0.0");
		vBoxCartesianValues.add(entryTargetDY);
		entryTargetDY.show();
		// entryTargetValue3
		entryTargetDZ.set_text("0.0");
		vBoxCartesianValues.add(entryTargetDZ);
		entryTargetDZ.show();
		// entryTargetValue4
		entryTargetRX.set_text("0.0");
		vBoxCartesianValues.add(entryTargetRX);
		entryTargetRX.show();
		// entryTargetValue5
		entryTargetRY.set_text("0.0");
		vBoxCartesianValues.add(entryTargetRY);
		entryTargetRY.show();
		// entryTargetValue6
		entryTargetRZ.set_text("0.0");
		vBoxCartesianValues.add(entryTargetRZ);
		entryTargetRZ.show();

		// entryActualValue1
		entryActualDX.set_text("");
		vBoxCartesianActualValues.add(entryActualDX);
		entryActualDX.set_editable(FALSE);
		entryActualDX.show();
		// entryActualValue2
		entryActualDY.set_text("");
		vBoxCartesianActualValues.add(entryActualDY);
		entryActualDY.set_editable(FALSE);
		entryActualDY.show();
		// entryActualValue3
		entryActualDZ.set_text("");
		vBoxCartesianActualValues.add(entryActualDZ);
		entryActualDZ.set_editable(FALSE);
		entryActualDZ.show();
		// entryActualValue4
		entryActualRX.set_text("");
		vBoxCartesianActualValues.add(entryActualRX);
		entryActualRX.set_editable(FALSE);
		entryActualRX.show();
		// entryActualValue5
		entryActualRY.set_text("");
		vBoxCartesianActualValues.add(entryActualRY);
		entryActualRY.set_editable(FALSE);
		entryActualRY.show();
		// entryActualValue6
		entryActualRZ.set_text("");
		vBoxCartesianActualValues.add(entryActualRZ);
		entryActualRZ.set_editable(FALSE);
		entryActualRZ.show();

		vBoxCartesian.add(gridCartesianButtons);
		gridCartesianButtons.set_column_spacing(10);
		gridCartesianButtons.set_row_spacing(15);
		gridCartesianButtons.show();

		// buttonSendTargetCartesian
		buttonSendTargetCartesian.set_label("  Send Cartesian Target   ");
		buttonSendTargetCartesian.signal_clicked().connect
		(
			sigc::mem_fun
			(
				*this,
				&GTKRobot::buttonSendCartesianClicked
			)
		);
		gridCartesianButtons.add(buttonSendTargetCartesian);
		buttonSendTargetCartesian.show();

		// buttonSendTestCartesian
		buttonSendTestCartesian.set_label("  Test Cartesian Pos  ");
		buttonSendTestCartesian.signal_clicked().connect
		(
			sigc::mem_fun
			(
				*this,
				&GTKRobot::buttonTestCartesianClicked
			)
		);
		gridCartesianButtons.add(buttonSendTestCartesian);
		buttonSendTestCartesian.show();

		/*
			vBoxDataLogger
		*/
		// labelDataLogger
		labelDataLogger.set_label("Data Logger (.csv)");
		vBoxDataLogger.add(labelDataLogger);
		labelDataLogger.show();

		// entryDataLogger
		entryDataLogger.set_text("dataLog.csv");
		vBoxDataLogger.add(entryDataLogger);
		entryDataLogger.show();

		// checkButtonDataLogger
		checkButtonDataLogger.set_label("Start Recording Data");
		vBoxDataLogger.add(checkButtonDataLogger);
		checkButtonDataLogger.show();

		// labelDataLoggerCurrent
		labelDataLoggerCurrent.set_label("Amount of loops recorded");
		vBoxDataLogger.add(labelDataLoggerCurrent);
		labelDataLoggerCurrent.show();

		// entryDataLoggerCurrent
		entryDataLoggerCurrent.set_text("0");
		vBoxDataLogger.add(entryDataLoggerCurrent);
		entryDataLoggerCurrent.set_editable(FALSE);
		entryDataLoggerCurrent.show();

		// buttonDataLoggerSave
		buttonDataLoggerSave.set_label("Save Data");
		buttonDataLoggerSave.signal_clicked().connect
		(
			sigc::mem_fun
			(
				*this,
				&GTKRobot::buttonDataLoggerSaveClicked
			)
		);
		vBoxDataLogger.add(buttonDataLoggerSave);
		buttonDataLoggerSave.show();
		

	}
	void backgroundRefresh()
    {
	while(STOP_ECAT==FALSE)
	  {
		std::string pos1 = std::to_string(theta[0]);
		std::string pos2 = std::to_string(theta[1]);
		std::string pos3 = std::to_string(theta[2]);
		std::string pos4 = std::to_string(theta[3]);
		std::string pos5 = std::to_string(theta[4]);
		std::string pos6 = std::to_string(theta[5]);
		Robot.ForwardKinematics( theta, FW );
		std::string dx = std::to_string(FW[0]);
		std::string dy = std::to_string(FW[1]);
		std::string dz = std::to_string(FW[2]);
		std::string rx = std::to_string(FW[3]);
		std::string ry = std::to_string(FW[4]);
		std::string rz = std::to_string(FW[5]);
		entryActualValue1.set_text(pos1);
		entryActualValue2.set_text(pos2);
		entryActualValue3.set_text(pos3);
		entryActualValue4.set_text(pos4);
		entryActualValue5.set_text(pos5);
		entryActualValue6.set_text(pos6);
		entryActualDX.set_text(dx);
		entryActualDY.set_text(dy);
		entryActualDZ.set_text(dz);
		entryActualRX.set_text(rx);
		entryActualRY.set_text(ry);
		entryActualRZ.set_text(rz);
		if(checkButtonDataLogger.get_active())
		{
			recordData = TRUE;
			logFileName = entryDataLogger.get_text();
			std::string logCount = std::to_string(logCounter);
			entryDataLoggerCurrent.set_text(logCount);
		}
		else
		{
			recordData = FALSE;
			entryDataLoggerCurrent.set_text("0");
		}
		ik_sent = FALSE;
		sleep(1);
		cout.flush();
      } 
    }
	// event handlers
	protected: void buttonStartETCClicked()
	{
		string networkI = entryNetworkInterface.get_text();
		ifname= networkI.c_str(); // IMPORTANT
		string slaveN = entryNofslaves.get_text();
		slaveNumber = stoi(slaveN); // IMPORTANT
		string RTloop = entryRTloopTime.get_text();
		string modeOfOperation = comboBoxModeofOp.get_active_text();
		if (modeOfOperation == "CSP")
		{
			modeGUI = 1;
		}
		else if (modeOfOperation == "Cartesian")
		{
			modeGUI = 2;
		}
		else if (modeOfOperation == "Joystick")
		{
			modeGUI = 3;
		}
		else
		{
			throw runtime_error("Unrecognized mode of operation!");
		}
		STOP_ECAT = FALSE;
		osal_thread_create(&thread2, 128000, &ecatcheck, NULL);
		// Main excecution acyclic loop
		osal_thread_create(&thread1, 128000, &runner, NULL);
		Glib::Thread::create(sigc::mem_fun(*this, &GTKRobot::backgroundRefresh), true);
		printf("EtherCAT Network Launched");
	}

	protected: void buttonStopETCClicked()
	{
		STOP_ECAT = TRUE;
		printf("EtherCAT Network Stop\n");

	}

	protected: void buttonModeofOpClicked()
	{
		string modeOfOperation = comboBoxModeofOp.get_active_text();
		if (modeOfOperation == "CSP")
		{
			modeGUI = 1;
		}
		else if (modeOfOperation == "Cartesian")
		{
			modeGUI = 2;
		}
		else if (modeOfOperation == "Joystick")
		{
			modeGUI = 3;
			jsMode = 0;
		}
		else
		{
			throw runtime_error("Unrecognized mode of operation!");
		}
	}
	protected: void buttonSendTargetClicked()
	{
		string target1 = entryTargetValue1.get_text();
		string target2 = entryTargetValue2.get_text();
		string target3 = entryTargetValue3.get_text();
		string target4 = entryTargetValue4.get_text();
		string target5 = entryTargetValue5.get_text();
		string target6 = entryTargetValue6.get_text();
		targetJointVal[0] = int(stod(target1)*10000);
		targetJointVal[1] = int(stod(target2)*10000);
		targetJointVal[2] = int(stod(target3)*10000);
		targetJointVal[3] = int(stod(target4)*10000);
		targetJointVal[4] = int(stod(target5)*10000);
		targetJointVal[5] = int(stod(target6)*10000);
	}
	protected: void buttonSendCartesianClicked()
	{
		string target1 = entryTargetDX.get_text();
		string target2 = entryTargetDY.get_text();
		string target3 = entryTargetDZ.get_text();
		string target4 = entryTargetRX.get_text();
		string target5 = entryTargetRY.get_text();
		string target6 = entryTargetRZ.get_text();
		cartesianTarget[0] = stod(target1);
		cartesianTarget[1] = stod(target2);
		cartesianTarget[2] = stod(target3);
		cartesianTarget[3] = stod(target4);
		cartesianTarget[4] = stod(target5);
		cartesianTarget[5] = stod(target6);
		valid_ik = Robot.InverseKinematics( cartesianTarget, theta_ik );
		robot_theta_ik[0] = int(theta_ik[0]*10000);
		robot_theta_ik[1] = int(theta_ik[1]*10000);
		robot_theta_ik[2] = int(theta_ik[2]*10000);
		robot_theta_ik[3] = int(theta_ik[3]*10000);
		robot_theta_ik[4] = int(theta_ik[4]*10000);
		robot_theta_ik[5] = int(theta_ik[5]*10000);
		std::string response = (valid_ik)? "Yes" : "No";
		ik_sent = TRUE;
		cout<<"Inverse kinematics ok? "<<response<<endl;
		cout<<"GUI "<<theta_ik[0]<<" "<<theta_ik[1]<<" "<<theta_ik[2]<<" "<<theta_ik[3]<<" "<<theta_ik[4]<<" "<<theta_ik[5]<<endl;
	}
	protected: void buttonTestCartesianClicked()
	{
		valid_ik = Robot.InverseKinematics( FW, theta_ik );
		std::string response = (valid_ik)? "Yes" : "No";
		cout<<"Inverse kinematics ok? "<<response<<endl;
		cout<<"GUI "<<theta_ik[0]<<" "<<theta_ik[1]<<" "<<theta_ik[2]<<" "<<theta_ik[3]<<" "<<theta_ik[4]<<" "<<theta_ik[5]<<endl;
	}
	protected: void buttonDataLoggerSaveClicked()
	{
		printf("Recording checkbox is: %b\n",checkButtonDataLogger.get_active());
		printf("Saving data...\n");
	}
};

int main (int argc, char *argv[])
{
	Main kit(argc, argv);
	GTKRobot GTKRobot;
	// GTKRobot.entryActualValue1.set_text("10");
	Main::run(GTKRobot);
	return 0;
	printf("non reach");
}