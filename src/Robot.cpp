#include "WPILib.h"
#include <Counter.h>
#include "Commands/Command.h"
#include <string>
#include <iostream>
#include <TrcDefs.h>
#include <RobotDriveOutput.h>

#define  __PRINT_COMMAND_H__
//#define DRIVE_ENCODER_PPR 	2048

// Joystick Mapping
#define NOODLER_IN				1
#define NOODLER_OUT				2
#define PID_TEST_3				3
#define PID_TEST_4				4
#define ELEVATOR_DOWN			5
#define ELEVATOR_UP				6
#define CLAW_IN					7
#define CLAW_OUT				8
#define PID_TEST_1				9
#define PID_TEST_2				10

#define AUTO_SPEED1 0.5
#define AUTO_SPEED2 0.3

#define AUTO_STAGE1 1.0
#define AUTO_STAGE2 3.4



#define JOYSTICK_DEAD_PERCENTAGE 0.1

using namespace std;

// Robot-state booleans
bool elevatorInUse = false;
bool elevatorUp = true;
bool noodlerInUse = false;
bool noodlerIn = false;

// Speed coefficients for the drive train and elevator control
float driveCoefficient = 0;
float elevatorCoefficient = 0;

class RobotDemo: public IterativeRobot {
	// Drive system motor controllers
	Talon *rightTalon;
	Talon *leftTalon;

	Victor *elevator1;
	Victor *elevator2;
	Victor *leftGrabber;
	Victor *rightGrabber;
//	Victor *noodler;

	//Limit Switches
//	DigitalInput *bottomSwitch;
//	DigitalInput *middleSwitch;
//	DigitalInput *topSwitch;

	// Joysticks
	Joystick *leftController;
	Joystick *rightController;
	Joystick *utilityController;

	RobotDrive *robotDrive;

	Timer *autonomousTimer;

	// Solenoids
	DoubleSolenoid *leftGrabberSolenoid;
	DoubleSolenoid *rightGrabberSolenoid;

	// Flags
	bool firstFlag = false;
public:
	RobotDemo(void) {
		leftTalon = new Talon(0);
		rightTalon = new Talon(1);
		elevator1 = new Victor(2);
		elevator2 = new Victor(7);
		leftGrabber = new Victor(3);
		rightGrabber = new Victor(4);
		//noodler = new Victor(6);

		robotDrive = new RobotDrive(rightTalon, leftTalon);

//		bottomSwitch = new DigitalInput(6);
//		middleSwitch = new DigitalInput(7); //must find the value if used later
//		topSwitch = new DigitalInput(8);

		autonomousTimer = new Timer();

		leftController = new Joystick(0); // Logitech Attack 3
		rightController = new Joystick(1); // Logitech Attack 3
		utilityController = new Joystick(2); // Logitech Gamepad

		leftGrabberSolenoid = new DoubleSolenoid(0, 1);
		rightGrabberSolenoid = new DoubleSolenoid(2, 3);
	}

	/********************************** Init Routines *************************************/

	void RobotInit(void) {}

	void DisabledInit(void) {
		elevator1->SetSpeed(0.0);
		elevator2->SetSpeed(0.0);
		leftTalon->SetSpeed(0.0);
		rightTalon->SetSpeed(0.0);
	}

	void AutonomousInit(void) {
		rightTalon->SetSpeed(0.0);
		leftTalon->SetSpeed(0.0);

		autonomousTimer->Start();
		autonomousTimer->Reset();
	}

	void TeleopInit(void) {
		// Set all motor controllers to be not moving initially.
		//elevator1->SetSpeed(0.0);
		//elevator2->SetSpeed(0.0);
		//leftTalon->SetSpeed(0.0);
		//rightTalon->SetSpeed(0.0);
	}

	/********************************** Periodic Routines *************************************/

	void DisabledPeriodic(void) {}

	void AutonomousPeriodic(void) {
//		cout << "Timer: " << autonomousTimer->Get() << endl;

		if (autonomousTimer->Get() < AUTO_STAGE1) {
			leftTalon->SetSpeed(AUTO_SPEED1);
			rightTalon->SetSpeed(-0.9*AUTO_SPEED1);
		} else if (autonomousTimer->Get() < AUTO_STAGE2) {
			leftTalon->SetSpeed(AUTO_SPEED2);
			rightTalon->SetSpeed(-0.9*AUTO_SPEED2);
			//leftTalon->SetSpeed((AUTO_SPEED - (autonomousTimer->Get()-AUTO_STAGE1)));
			//rightTalon->SetSpeed((-AUTO_SPEED + (autonomousTimer->Get()-AUTO_STAGE1)));
		} else {
			leftTalon->SetSpeed(0.0);
			rightTalon->SetSpeed(0.0);
		}
	}

	void TeleopPeriodic(void) {
		// Initial Stuff
		driveCoefficient = ((-rightController->GetZ() + 1) / 3.125) + 0.36;
		elevatorCoefficient = ((-leftController->GetZ() + 1) / 2.22) + 0.1;

		// Elevator code so that later we can add PID control loops to it
		if (utilityController->GetRawButton(ELEVATOR_DOWN)) {
			elevatorInUse = true;
			elevatorUp = false;
		} else if (utilityController->GetRawButton(ELEVATOR_UP)) {
			elevatorInUse = true;
			elevatorUp = true;
		} else {
			elevatorInUse = false;
		}

		if (utilityController->GetRawButton(NOODLER_IN)) {
			noodlerInUse = true;
			noodlerIn = true;
		} else if (utilityController->GetRawButton(NOODLER_OUT)) {
			noodlerInUse = true;
			noodlerIn = false;
		} else {
			noodlerInUse = false;
		}

		// To Robot Activation

		float rightDriveStickValue = rightController->GetY();
		float leftDriveStickValue = leftController->GetY();

		if (ABS(rightController->GetY()) < JOYSTICK_DEAD_PERCENTAGE) {
			rightDriveStickValue = 0;
		}
		if (ABS(leftController->GetY()) < JOYSTICK_DEAD_PERCENTAGE) {
			leftDriveStickValue = 0;
		}

		robotDrive->TankDrive(driveCoefficient * rightDriveStickValue,
				driveCoefficient * leftDriveStickValue);

		// To add another use an or (||) so that it stops once it reaches a certain point
		if (elevatorInUse) {
			if (elevatorUp) {
				elevator1->SetSpeed(elevatorCoefficient);
				elevator2->SetSpeed(elevatorCoefficient);
			} else {
				elevator1->SetSpeed(-elevatorCoefficient);
				elevator2->SetSpeed(-elevatorCoefficient);
			}
		} else {
			elevator1->SetSpeed(0);
			elevator2->SetSpeed(0);
		}

		// Grabber Solenoids
		if (utilityController->GetRawButton(CLAW_IN)) {
			rightGrabberSolenoid->Set(DoubleSolenoid::kForward);
			leftGrabberSolenoid->Set(DoubleSolenoid::kForward);
		} else if (utilityController->GetRawButton(CLAW_OUT)) {
			rightGrabberSolenoid->Set(DoubleSolenoid::kReverse);
			leftGrabberSolenoid->Set(DoubleSolenoid::kReverse);
		} else {
			rightGrabberSolenoid->Set(DoubleSolenoid::kOff);
			leftGrabberSolenoid->Set(DoubleSolenoid::kOff);
		}

		// Grabber Motor
		leftGrabber->SetSpeed(-utilityController->GetRawAxis(1));
		rightGrabber->SetSpeed(-utilityController->GetRawAxis(3));

		// Noodler
//		if (noodlerInUse
//				|| (rightDriveStickValue > 0 && leftDriveStickValue > 0)) {
//			if (noodlerIn) {
//				noodler->SetSpeed(.8);
//			} else {
//				noodler->SetSpeed(-.8);
//			}
//		} else {
//			noodler->SetSpeed(0);
//		}

		// ===== Debugging Stuff and Testing =====
		// cout << "BottomSwitch: " << bottomSwitch->Get() << " MiddleSwitch: "
		//	<< middleSwitch->Get() << "TopSwitch: "<< topSwitch->Get()<< endl;
	}

	void DisabledContinuous(void) {}
	void AutonomousContinuous(void) {}
	void TeleopContinuous(void) {}
};

START_ROBOT_CLASS(RobotDemo)
