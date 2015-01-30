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

using namespace std;

bool elevatorinuse;
bool elevatorup;
bool noodlerinuse;
bool noodlerdir;
bool driveRight;
bool driveLeft;

// Flags
bool pidButtonFlag = true;

float driveCoefficient;
float elevatorCoefficient;

class RobotDemo : public IterativeRobot {
	// Drive system motor controllers
    Talon *rightTalon;
    Talon *leftTalon;
    Talon *noodler;
    Victor *elevator;

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
    	leftTalon  = new Talon(0);
  		rightTalon = new Talon(1);
  		noodler    = new Talon(2);
  		elevator   = new Victor(2);

  		// robot-state booleans
  		elevatorinuse = false;
  		elevatorup = true;
  		noodlerinuse = false;
  		noodlerdir = false;

  		robotDrive = new RobotDrive(rightTalon, leftTalon);

  		leftEncoder = new Encoder(0, 1, true, Encoder::EncodingType::k4X);
  		rightEncoder = new Encoder(2, 3, true, Encoder::EncodingType::k4X);
  		elevatorEncoder = new Encoder(4, 5, true, Encoder::EncodingType::k4X);

  		leftDistance = new DistanceEncoder(leftEncoder);
  		rightDistance = new DistanceEncoder(rightEncoder);

  		leftController     = new Joystick(0);
  		rightController    = new Joystick(1);
  		elevatorController = new Joystick(2);

  		leftPID =  new PIDController(0.008, 0.001, 0.006, leftDistance, leftTalon);
  		rightPID = new PIDController(0.008, 0.001, 0.006, rightDistance, rightTalon);

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
  		driveCoefficient = ((-rightController->GetZ() + 1) / 3.125) + 0.36;
  		elevatorCoefficient = ((-leftController->GetZ() + 1) / 5) + 0.1;
  		robotDrive->TankDrive(driveCoefficient * rightController->GetY(), driveCoefficient * leftController->GetY());

  		// Elevator code so that later we can add PID control loops to it
  		if (elevatorController->GetRawButton(6)) {
  			elevatorinuse = true;
  			elevatorup = false;
  		} else if (elevatorController->GetRawButton(5)) {
  			elevatorinuse = true;
  			elevatorup = true;
  		} else {
  			elevatorinuse = false;
  		}

  		// PID Test Driving
  		if (pidButtonFlag && (elevatorController->GetRawButton(2) || elevatorController->GetRawButton(4))) {
  			float pidMovement = (elevatorController->GetRawButton(2) ? -100.0 : 100.0);

  			leftPID->Enable();
  			rightPID->Enable();

  			leftPID->SetSetpoint(leftPID->GetSetpoint() + pidMovement);
  			rightPID->SetSetpoint(rightPID->GetSetpoint() + pidMovement);

  			while ((rightPID->GetError() > 1.0) && (leftPID->GetError() > 1.0)) {
  				cout << "DRIVING -- Right error: " << rightPID->GetError() << "  Left error: " << leftPID->GetError() << endl;
  			}

  			pidButtonFlag = false;
  		} else if (!(elevatorController->GetRawButton(2) || elevatorController->GetRawButton(4))) {
  			pidButtonFlag = true;
  		}

  		// Toggle buttons for noodler
  		if(rightController->GetRawButton(4)) {
  			noodlerinuse = true;
  			noodlerdir = true;
  		} else if(leftController->GetRawButton(4)) {
  			noodlerinuse = true;
  			noodlerdir = false;
  		} else {
  			noodlerinuse = false;
  		}

  		// Utility checkers
  		// To add another use an or (||) so that it stops once it reaches a certain point
  		if (elevatorinuse) {
  			if (elevatorup) {
  				elevator->SetSpeed(.3);
  			} else {
  				elevator->SetSpeed(-.3);
  			}
  		} else {
  			elevator->SetSpeed(0);
  		}

//  		if(driving) {
//  			rightPID->Enable();
//  			leftPID->Enable();
//
//  	  		rightPID->SetAbsoluteTolerance(20.0);
//  	  		leftPID->SetAbsoluteTolerance(20.0);
//
//  			if(driveLeft) {
//  				leftPID->SetSetpoint((leftPID->GetSetpoint()+1000));
//  			} else if (driveRight) {
//  				rightPID->SetSetpoint((rightPID->GetSetpoint()+1000));
//  			} else {
//  				//rightPID->SetSetpoint((rightPID->GetSetpoint()+100));
//  				//leftPID->SetSetpoint((leftPID->GetSetpoint()+100));
//  			}
//  		} else {
//  			rightPID->Disable();
//  			leftPID->Disable();
//  		}

  		if (noodlerinuse) {
  			if (noodlerdir) {
  				noodler->SetSpeed(.3);
  			} else {
  				noodler->SetSpeed(-.3);
  			}
  		} else {
  			noodler->SetSpeed(0);
  		}

  		// int32_t rightRate = rightEncoder->GetRate();
  		// int32_t leftRate = leftEncoder->GetRate();

  		//int32_t rightDistance = rightEncoder->GetDistance();
  		//int32_t leftDistance = leftEncoder->GetDistance();

  		//cout << "Right: " << rightDistance << endl;
  		//cout << "Left: " << leftDistance << endl;
		//cout << endl;

  		cout << "Right error: " << rightPID->GetError() << "  Left error: " << leftPID->GetError() << endl;
  	}

  	void DisabledContinuous(void) {}
  	void AutonomousContinuous(void) {}
  	void TeleopContinuous(void) {}
};

START_ROBOT_CLASS(RobotDemo)
