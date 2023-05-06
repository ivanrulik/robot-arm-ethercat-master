#include <gtkmm.h>
// EtherCAT
#include "CubemarsMotor.h"
#include "ethercat.h"

using namespace std;
using namespace Gtk;

class GTKRobot : public Window
{
	// controls

	protected:
		Grid gridLayout;
		Grid gridTargetValues;
		VBox vBoxJointLabels;
		VBox vBoxJointValues;
		VBox vBoxJointActualValues;
		VBox vBoxSetup;
		VBox vBoxTarget;
		VBox vBoxMonitor;
		Label labelNetworkInterface;
		Label labelNofSlaves;
		Label labelmodeOfOp;
		Label labelRTloopTime;
		Label labelRobotstatus;
		Label labelTargetJoint;
		Label labelTargetPosition;
		Label labelJoint1;
		Label labelJoint2;
		Label labelJoint3;
		Label labelJoint4;
		Label labelJoint5;
		Label labelJoint6;
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
		ComboBoxText comboBoxModeofOp;
		ComboBoxText comboBoxTargetJoint;
		CheckButton checkButtonSquareSignal;
		Button buttonStartETC;
		Button buttonModeofOp;
		Button buttonSendTarget;

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
		// vBoxMonitor
		// gridLayout.add(vBoxMonitor);
		// vBoxMonitor.show();

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
		entryNofslaves.set_text("1");
		vBoxSetup.add(entryNofslaves);
		entryNofslaves.show();

		// labelRTloopTime
		labelRTloopTime.set_label("RT loop time in us:");
		vBoxSetup.add(labelRTloopTime);
		labelRTloopTime.show();

		// entryRTloopTime
		entryRTloopTime.set_text("1000");
		vBoxSetup.add(entryRTloopTime);
		entryRTloopTime.show();

		// checkButtonSquareSignal
		checkButtonSquareSignal.set_label("Generate Square Signal");
		vBoxSetup.add(checkButtonSquareSignal);
		checkButtonSquareSignal.show();

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

		// labelmodeOfOp
		labelmodeOfOp.set_label("Mode of Operation:");
		vBoxSetup.add(labelmodeOfOp);
		labelmodeOfOp.show();

		// comboModeofOp
		comboBoxModeofOp.append("CSP");
		comboBoxModeofOp.append("CSV");
		comboBoxModeofOp.append("CST");
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

		/*
			vBoxTarget
		*/
		// labelTargetJoint
		labelTargetJoint.set_label("  Target Joint  ");
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
		entryTargetValue1.set_text("0");
		vBoxJointValues.add(entryTargetValue1);
		entryTargetValue1.show();
		// entryTargetValue2
		entryTargetValue2.set_text("0");
		vBoxJointValues.add(entryTargetValue2);
		entryTargetValue2.show();
		// entryTargetValue3
		entryTargetValue3.set_text("0");
		vBoxJointValues.add(entryTargetValue3);
		entryTargetValue3.show();
		// entryTargetValue4
		entryTargetValue4.set_text("0");
		vBoxJointValues.add(entryTargetValue4);
		entryTargetValue4.show();
		// entryTargetValue5
		entryTargetValue5.set_text("0");
		vBoxJointValues.add(entryTargetValue5);
		entryTargetValue5.show();
		// entryTargetValue6
		entryTargetValue6.set_text("0");
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
		buttonSendTarget.set_label("Send Target");
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
			vBoxMonitor
		*/
		// labelNetworkInterface
		// labelRobotstatus.set_label("  Robot Status  ");
		// vBoxMonitor.add(labelRobotstatus);
		// labelRobotstatus.show();

	}

	// event handlers
	protected: void buttonStartETCClicked()
	{
		string networkI = entryNetworkInterface.get_text();
		string slaveN = entryNofslaves.get_text();
		string RTloop = entryRTloopTime.get_text();
		string modeOfOperation = comboBoxModeofOp.get_active_text();

		if (modeOfOperation != "")
		{
			printf("%s, %d, %d, %s\n", networkI.c_str(),stoi(slaveN),stoi(RTloop), modeOfOperation.c_str());
		}
		else
		{
			throw runtime_error("Unrecognized display mode!");
		}
	}

	protected: void buttonModeofOpClicked()
	{
		string modeOfOperation = comboBoxModeofOp.get_active_text();
		if (modeOfOperation == "CST")
		{
			printf("%x\n", 0x000A);
		}
		else if (modeOfOperation == "CSV")
		{
			printf("%x\n", 0x000B);
		}
		else if (modeOfOperation == "CSP")
		{
			printf("%x\n", 0x000B);
		}
		else
		{
			throw runtime_error("Unrecognized mode of operation!");
		}
	}
	protected: void buttonSendTargetClicked()
	{
		string networkI = entryNetworkInterface.get_text();
		string slaveN = entryNofslaves.get_text();
		string RTloop = entryRTloopTime.get_text();
		string modeOfOperation = comboBoxModeofOp.get_active_text();

		if (modeOfOperation != "")
		{
			printf("%s, %d, %d, %s\n", networkI.c_str(),stoi(slaveN),stoi(RTloop), modeOfOperation.c_str());
		}
		else
		{
			throw runtime_error("Unrecognized display mode!");
		}
	}
};



int main (int argc, char *argv[])
{
	Main kit(argc, argv);
	GTKRobot GTKRobot;
	Main::run(GTKRobot);
	return 0;
	printf("non reach");
}