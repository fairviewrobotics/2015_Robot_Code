#include "WPILib.h"
#include <Encoder.h>
#include <Counter.h>
#include "Commands/Command.h"
#include <string>
#include <iostream>
#include <TrcDefs.h>
#include <PIDController.h>
#include <DistanceEncoder.h>
#include <RobotDriveOutput.h>
#include <Movement.h>

#define  __PRINT_COMMAND_H__
#define DRIVE_ENCODER_PPR 	2048

//
// Joystick Mapping
//

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

#define JOYSTICK_DEAD_PERCENTAGE 0.1
#define PID_TEST_TURN_VALUE		 90

//
// Other Constants
//
#define RADIUS_TURN				11.75

using namespace std;

// Robot-state booleans
bool elevatorInUse = false;
bool elevatorUp = true;
bool noodlerInUse = false;
bool noodlerIn = false;

// Flags
bool inProcessFlag = false;
bool pidButtonTurnFlag = true;
bool pidButtonMoveFlag = true;

// Speed coefficients for the drive train and elevator control
float driveCoefficient;
float elevatorCoefficient;

// Autonomous instructions
vector<Movement> instructions;
int currentInstruction = 0;

class RobotDemo: public IterativeRobot {
	// Drive system motor controllers
	Talon *rightTalon;
	Talon *leftTalon;

	Victor *elevator1;
	Victor *elevator2;
	Victor *leftGrabber;
	Victor *rightGrabber;
	Victor *noodler;

	// Encoders
	Encoder *rightEncoder;
	Encoder *leftEncoder;
	Encoder *elevatorEncoder;
	DistanceEncoder *rightDistance;
	DistanceEncoder *leftDistance;

	//Limit Switches
	DigitalInput *bottomSwitch;
	DigitalInput *middleSwitch;
	DigitalInput *topSwitch;

	// Joysticks
	Joystick *leftController;
	Joystick *rightController;
	Joystick *utilityController;

	RobotDrive *robotDrive;

	// PID Controllers
	PIDController *rightPID;
	PIDController *leftPID;

	// Solenoids
	DoubleSolenoid *leftGrabberSolenoid;
	DoubleSolenoid *rightGrabberSolenoid;

	// DigitalInput Line Sensor
	// Black: DIO signal (0)
	// White: DIo Signal (1)
	// Blue: 10-16V supply (PD Board)
	// Brown: Ground

public:
	RobotDemo(void) {
		leftTalon = new Talon(0);
		rightTalon = new Talon(1);
		elevator1 = new Victor(2);
		elevator2 = new Victor(7);
		leftGrabber = new Victor(3);
		rightGrabber = new Victor(4);
		noodler = new Victor(6);

		robotDrive = new RobotDrive(rightTalon, leftTalon);

		bottomSwitch = new DigitalInput(6);
		middleSwitch = new DigitalInput(7);
		topSwitch = new DigitalInput(8);

		leftEncoder = new Encoder(0, 1, true, Encoder::EncodingType::k4X);
		rightEncoder = new Encoder(2, 3, true, Encoder::EncodingType::k4X);
		elevatorEncoder = new Encoder(4, 5, true, Encoder::EncodingType::k4X);

		leftDistance = new DistanceEncoder(leftEncoder);
		rightDistance = new DistanceEncoder(rightEncoder);

		leftController = new Joystick(0); // Logitech Attack 3
		rightController = new Joystick(1); // Logitech Attack 3
		utilityController = new Joystick(2); // Logitech Gamepad

		leftPID = new PIDController(0.0085, 0.0, 0.006, leftDistance,
				leftTalon);
		rightPID = new PIDController(0.0085, 0.0, 0.006, rightDistance,
				rightTalon);

		leftGrabberSolenoid = new DoubleSolenoid(0, 1);
		rightGrabberSolenoid = new DoubleSolenoid(2, 3);
	}

	/********************************** Extra methods *************************************/

	// From an angle to a point which both must drive to
	float AngleToSetpoint(float angle) {
		return (angle * RADIUS_TURN * (PI / 180));
	}

	/********************************** Init Routines *************************************/

	void RobotInit(void) {
	}

	void DisabledInit(void) {
		rightPID->Reset();
		leftPID->Reset();

		rightPID->Disable();
		leftPID->Disable();

		elevator1->SetSpeed(0.0);
		elevator2->SetSpeed(0.0);
		leftTalon->SetSpeed(0.0);
		rightTalon->SetSpeed(0.0);
	}

	void AutonomousInit(void) {

		cout << "PID Enabled party?" << endl;

		instructions.push_back(Movement(false, 40.0, leftPID, rightPID));
		instructions.push_back(Movement(true, 90.0, leftPID, rightPID));
		instructions.push_back(Movement(false, 40.0, leftPID, rightPID));
		instructions.push_back(Movement(true, 90.0, leftPID, rightPID));

		cout << "Added Commands" << endl;
	}

	void TeleopInit(void) {
		rightPID->Disable();
		leftPID->Disable();

		// Set all motor controllers to be not moving initially.
		elevator1->SetSpeed(0.0);
		elevator2->SetSpeed(0.0);
		leftTalon->SetSpeed(0.0);
		rightTalon->SetSpeed(0.0);
	}

	/********************************** Periodic Routines *************************************/

	void DisabledPeriodic(void) {
	}

	void AutonomousPeriodic(void) {
		// cout << "Right error: " << rightPID->GetError() << "  Left error: " << leftPID->GetError() << endl;
		if (!instructions[currentInstruction].IsRunning()) {
			instructions[currentInstruction].DoMovement();
			cout << "Doing "<< currentInstruction-1 << " Inprocess" << endl;

		} else if (instructions[currentInstruction].IsComplete()) {
			currentInstruction++;
			cout << "Instruction "<< currentInstruction-1 << " Complete" << endl;

		} else if (instructions.size() <= ((unsigned int) currentInstruction)) {
			rightPID->Disable();
			leftPID->Disable();

			cout << "Autonomous finished!" << endl;

		} else {
			cout << instructions[currentInstruction].GetError() << endl;
			cout << currentInstruction << endl;
		}
	}

	void TeleopPeriodic(void) {
		// Initial Stuff
		driveCoefficient = ((-rightController->GetZ() + 1) / 3.125) + 0.36;
		elevatorCoefficient = ((-leftController->GetZ() + 1) / 5) + 0.1;

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
		if (noodlerInUse
				|| (rightDriveStickValue > 0 && leftDriveStickValue > 0)) {
			if (noodlerIn) {
				noodler->SetSpeed(.8);
			} else {
				noodler->SetSpeed(-.8);
			}
		} else {
			noodler->SetSpeed(0);
		}

		// Debugging Stuff and testing

		// PID Move TO Test
		if (pidButtonMoveFlag && !inProcessFlag
				&& (utilityController->GetRawButton(PID_TEST_3)
						|| utilityController->GetRawButton(PID_TEST_4))) {

			inProcessFlag = true;
			pidButtonTurnFlag = false;

			Movement* mov = new Movement(false, 100.0, leftPID, rightPID);
			mov->DoMovement();

			pidButtonMoveFlag = true;
			inProcessFlag = false;

		} else if (!inProcessFlag
				&& (!(utilityController->GetRawButton(PID_TEST_3)
						|| utilityController->GetRawButton(PID_TEST_4)))) {
			pidButtonMoveFlag = true;
		}

		// PID Turn TO Test
		if (pidButtonTurnFlag && !inProcessFlag
				&& (utilityController->GetRawButton(PID_TEST_2)
						|| utilityController->GetRawButton(PID_TEST_1))) {

			inProcessFlag = true;
			pidButtonTurnFlag = false;

			Movement* mov = new Movement(true, 90.0, leftPID, rightPID);
			mov->DoMovement();

			pidButtonTurnFlag = true;
			inProcessFlag = false;

		} else if (!inProcessFlag
				&& (!(utilityController->GetRawButton(PID_TEST_2)
						|| utilityController->GetRawButton(PID_TEST_1)))) {
			pidButtonTurnFlag = true;
		}

		cout << "Right error: " << rightPID->GetError() << "  Left error: "
				<< leftPID->GetError() << endl;
		cout << "BottomSwitch: " << bottomSwitch->Get() << " MiddleSwitch: "
				<< middleSwitch->Get() << endl;
	}

	void DisabledContinuous(void) {
	}
	void AutonomousContinuous(void) {
	}
	void TeleopContinuous(void) {
	}
};

START_ROBOT_CLASS(RobotDemo)
