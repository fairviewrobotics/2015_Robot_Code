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

#define __PRINT_COMMAND_H__
#define DRIVE_ENCODER_PPR 	2048


//
//Joystick Mapping
//

#define PID_TEST_FORWARD		9
#define PID_TEST_BACKWARD		10
#define ELEVATOR_DOWN			5
#define ELEVATOR_UP				6
#define CLAW_IN					7
#define CLAW_OUT				8
#define NOODLER_IN				1
#define NOODLER_OUT				2

using namespace std;

// Robot-state booleans
bool elevatorinuse = false;
bool elevatorup = true;
bool noodlerinuse = false;
bool noodlerdir = false;

// Flags
bool pidButtonFlag = true;

// Speed coefficients for the drive train and elevator control
float driveCoefficient;
float elevatorCoefficient;

class RobotDemo : public IterativeRobot {
	// Drive system motor controllers
    Talon *rightTalon;
    Talon *leftTalon;
    Victor *elevator;
    Victor *leftGrabber;
    Victor *rightGrabber;
    Victor *noodler;

    // Encoders
    Encoder *rightEncoder;
    Encoder *leftEncoder;
    Encoder *elevatorEncoder;
    DistanceEncoder *rightDistance;
    DistanceEncoder *leftDistance;

    // Joysticks
    Joystick *leftController;
    Joystick *rightController;
    Joystick *elevatorController;

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
    	leftTalon    = new Talon(0);
  		rightTalon   = new Talon(1);
  		elevator     = new Victor(2);
  		leftGrabber  = new Victor(3);
  		rightGrabber = new Victor(4);
  		noodler      = new Victor(6);

  		robotDrive = new RobotDrive(rightTalon, leftTalon);

  		leftEncoder = new Encoder(0, 1, true, Encoder::EncodingType::k4X);
  		rightEncoder = new Encoder(2, 3, true, Encoder::EncodingType::k4X);
  		elevatorEncoder = new Encoder(4, 5, true, Encoder::EncodingType::k4X);

  		leftDistance = new DistanceEncoder(leftEncoder);
  		rightDistance = new DistanceEncoder(rightEncoder);

  		leftController     = new Joystick(0); // Logitech Attack 3
  		rightController    = new Joystick(1); // Logitech Attack 3
  		elevatorController = new Joystick(2); // Logitech Gamepad

  		leftPID =  new PIDController(0.0085, 0.0, 0.006, leftDistance, leftTalon);
  		rightPID = new PIDController(0.0085, 0.0, 0.006, rightDistance, rightTalon);

  		leftGrabberSolenoid = new DoubleSolenoid(0, 1);
  		rightGrabberSolenoid = new DoubleSolenoid(2, 3);
    }

  	/********************************** Init Routines *************************************/

  	void RobotInit(void) {
  		rightEncoder->SetDistancePerPulse(PI*4/360.0);
  		leftEncoder->SetDistancePerPulse(PI*4/360.0);
  		elevatorEncoder->SetDistancePerPulse(PI*4/360.0); // Needs to be updated with the radius of the elevator coil.
  	}

  	void DisabledInit(void) {
  		rightPID->Reset();
  		leftPID->Reset();
  	}

  	void AutonomousInit(void) {
  		rightPID->Enable();
  		leftPID->Enable();

  		rightPID->SetAbsoluteTolerance(20.0);
  		leftPID->SetAbsoluteTolerance(20.0);

  		rightPID->SetSetpoint(2000);
  		leftPID->SetSetpoint(2000);
  	}

  	void TeleopInit(void) {
  		//rightPID->Enable();
  		//leftPID->Enable();

  		// Set all motor controllers to be not moving initially.
  		elevator->SetSpeed(0.0);
  		leftTalon->SetSpeed(0.0);
  		rightTalon->SetSpeed(0.0);
  	}

  	/********************************** Periodic Routines *************************************/

  	void DisabledPeriodic(void) {}

  	void AutonomousPeriodic(void) {
  		cout << "Right error: " << rightPID->GetError() << "  Left error: " << leftPID->GetError() << endl;
  	}

  	void TeleopPeriodic(void) {
  		// Initial Stuff
  		driveCoefficient = ((-rightController->GetZ() + 1) / 3.125) + 0.36;
  		elevatorCoefficient = ((-leftController->GetZ() + 1) / 5) + 0.1;
  		robotDrive->TankDrive(driveCoefficient * rightController->GetY(), -driveCoefficient * leftController->GetY());

  		// PID Test Driving
  		if (pidButtonFlag && (elevatorController->GetRawButton(PID_TEST_BACKWARD) || elevatorController->GetRawButton(PID_TEST_FORWARD))) {
  			float pidMovement = (elevatorController->GetRawButton(PID_TEST_BACKWARD) ? -2000.0 : 2000.0);

  			leftPID->Enable();
  			rightPID->Enable();

  			leftPID->SetSetpoint(leftPID->GetSetpoint() + pidMovement);
  			rightPID->SetSetpoint(rightPID->GetSetpoint() + pidMovement);

  			while ((rightPID->GetError() > 1.0) && (leftPID->GetError() > 1.0)) {
  				cout << "DRIVING -- Right error: " << rightPID->GetError() << "  Left error: " << leftPID->GetError() << endl;
  			}

  			leftPID->Disable();
  			rightPID->Disable();

  			pidButtonFlag = false;
  		} else if (!(elevatorController->GetRawButton(PID_TEST_BACKWARD) || elevatorController->GetRawButton(PID_TEST_FORWARD))) {
  			pidButtonFlag = true;
  		}

//  		if (elevatorController->GetRawAxis(1)) { // axis 1 & 3
//  			elevatorController
//  			leftGrabber->SetSpeed(1.0); // + is outwards, - in inwards
//  			rightGrabber->SetSpeed(1.0);
//  		} else if (elevatorController->GetRawButton(10)) {
//			leftGrabber->SetSpeed(-1.0); // + is outwards, - in inwards
//			rightGrabber->SetSpeed(-1.0);
//		} else {
  			leftGrabber->SetSpeed(elevatorController->GetRawAxis(1)*4);
  			rightGrabber->SetSpeed(elevatorController->GetRawAxis(3)*4);
//  		}

  		// Elevator code so that later we can add PID control loops to it
		if (elevatorController->GetRawButton(ELEVATOR_DOWN)) {
			elevatorinuse = true;
			elevatorup = false;
		} else if (elevatorController->GetRawButton(ELEVATOR_UP)) {
			elevatorinuse = true;
			elevatorup = true;
		} else {
			elevatorinuse = false;
		}

  		if(elevatorController->GetRawButton(NOODLER_IN)) {
  			noodlerinuse = true;
  			noodlerdir = true;
  		} else if(elevatorController->GetRawButton(NOODLER_OUT)) {
  			noodlerinuse = true;
  			noodlerdir = false;
  		} else {
  			noodlerinuse = false;
  		}

  		// Utility Activation
  		// To add another use an or (||) so that it stops once it reaches a certain point
  		if (elevatorinuse) {
  			if (elevatorup) {
  				elevator->SetSpeed(1);
  			} else {
  				elevator->SetSpeed(1);
  			}
  		} else {
  			elevator->SetSpeed(0);
  		}

  		// Grabber Solenoids
  		if (elevatorController->GetRawButton(CLAW_IN)) {
  			rightGrabberSolenoid->Set(DoubleSolenoid::kForward);
  			leftGrabberSolenoid->Set(DoubleSolenoid::kForward);
  		} else if(elevatorController->GetRawButton(CLAW_OUT)) {
  			rightGrabberSolenoid->Set(DoubleSolenoid::kReverse);
  			leftGrabberSolenoid->Set(DoubleSolenoid::kReverse);
  		} else {
  			rightGrabberSolenoid->Set(DoubleSolenoid::kOff);
  			leftGrabberSolenoid->Set(DoubleSolenoid::kOff);
  		}



  		if (noodlerinuse||(rightController->GetY()<0&&leftController->GetY()<0)) {

  			if (noodlerdir) {
  				noodler->SetSpeed(.8);
  			} else {
  				noodler->SetSpeed(-.8);
  			}
  		} else {
  			noodler->SetSpeed(0);
  		}

  		// Debugging Stuff
  		cout << "Right error: " << rightPID->GetError() << "  Left error: " << leftPID->GetError() << endl;
  	}

  	void DisabledContinuous(void) {}
  	void AutonomousContinuous(void) {}
  	void TeleopContinuous(void) {}
};

START_ROBOT_CLASS(RobotDemo)
