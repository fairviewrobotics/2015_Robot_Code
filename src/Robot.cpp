#include "WPILib.h"
#include <Encoder.h>
#include <Counter.h>
#include "Commands/Command.h"
#include <string>
#include <iostream>
#include <PIDController.h>
#include <TrcDefs.h>
#include <Event.h>
#include <DriveBase.h>
#include <TrcPIDCtrl.h>
#include <TrcPIDDrive.h>
#include <TrcPIDMotor.h>

#define __PRINT_COMMAND_H__
#define DRIVE_ENCODER_PPR 2048

using namespace std;

// const double PI = 3.14159265359;

class RobotDemo : public IterativeRobot {
	// Drive system motor controllers
    Talon *rightTalon;
    Talon *leftTalon;

    Talon *rightElevator;
    Talon *leftElevator;

    Encoder *rightEncoder;
    Encoder *leftEncoder;

    Joystick *controllerLeft;
    Joystick *controllerRight;

    PIDController *rightPID;
    PIDController *leftPID;

    TrcPIDDrive *pidDrive;
    DriveBase *autoDriveBase;

    TrcPIDCtrl *pidCtrlDrive;
    TrcPIDCtrl *pidCtrlTurn;

public:
    RobotDemo(void) {
  		rightTalon = new Talon(8);
  		leftTalon  = new Talon(9);

  		pidCtrlDrive = new TrcPIDCtrl(YDRIVE_KP, YDRIVE_KI, YDRIVE_KD, YDRIVE_KF, YDRIVE_TOLERANCE, YDRIVE_SETTLING);
  		pidCtrlTurn = new TrcPIDCtrl(TURN_KP, TURN_KI, TURN_KD, TURN_KF, TURN_TOLERANCE, TURN_SETTLING);

  		autoDriveBase = new DriveBase(rightTalon, leftTalon, pidCtrlDrive, pidCtrlTurn);
  		autoDriveBase->SetInvertedMotor(RobotDrive::kFrontLeftMotor, true);
  		autoDriveBase->SetInvertedMotor(RobotDrive::kRearLeftMotor, true);

  		rightEncoder = new Encoder(7, 6, true, Encoder::EncodingType::k4X);
  		leftEncoder = new Encoder(9, 8, false, Encoder::EncodingType::k4X);

  		controllerLeft  = new Joystick(0);
  		controllerRight = new Joystick(1);

  		rightElevator = new Talon(6);
  		leftElevator = new Talon(6);

  		// pidCtrlDrive = new TrcPIDCtrl(0.1, 0.001, 0. 0, &rightEncoder);
  		// pidCtrlDriveRight = new TrcPIDCtrl(0.1, 0.001, 0. 0, &rightEncoder);
		// pidCtrlTurnRight = new TrcPIDCtrl(0.1, 0.001, 0. 0, &rightEncoder);
		// pidCtrlDriveLeft = new TrcPIDCtrl(0.1, 0.001, 0. 0, &leftEncoder);
		// pidCtrlTurnLeft = new TrcPIDCtrl(0.1, 0.001, 0. 0, &leftEncoder);

  		// rightPID = new PIDController(0.1, 0.001, 0.0, &rightEncoder, &rightTalon);
  		// leftPID =  new PIDController(0.1, 0.001, 0.0, &leftEncoder, &leftTalon);
    }

  	/********************************** Init Routines *************************************/

  	void RobotInit(void) {
  		rightEncoder->SetDistancePerPulse(PI*4/360.0);
  		leftEncoder->SetDistancePerPulse(PI*4/360.0);
  	}

  	void DisabledInit(void) {}
  	void AutonomousInit(void) {}

  	void TeleopInit(void) {
  		// Set all motor controllers to be not moving initially.
  		leftTalon->SetSpeed(0.0);
  		rightTalon->SetSpeed(0.0);
  	}

  	/********************************** Periodic Routines *************************************/

  	void DisabledPeriodic(void) {}

  	void AutonomousPeriodic(void) {
  		rightPID->SetSetpoint(1000);
  		leftPID->SetSetpoint(1000);

  		pidCtrlDrive->SetOutputRange(-0.5, 0.5);
  		pidCtrlTurn->SetOutputRange(-0.5, 0.5);
//  		autoDriveBase->DriveSetTarget(0.0,
//  		                             -autoDistance * 12.0,
//  		                             0.0,
//  		                             false,
//  		                             &autoDriveEvent);
//  		autoSM->WaitForSingleEvent(&autoDriveEvent, currState + 1);
  	}

  	void TeleopPeriodic(void) {
  		autoDriveBase->TankDrive(controllerLeft, controllerRight);

  		if (controllerRight->GetTrigger()) {
  			leftElevator->SetSpeed(0.3);
			rightElevator->SetSpeed(0.3);
  		} else if (controllerLeft->GetTrigger()) {
  			leftElevator->SetSpeed(0.0);
			rightElevator->SetSpeed(0.0);
  		}

  		// int32_t rightRate = rightEncoder->GetRate();
  		// int32_t leftRate = leftEncoder->GetRate();
  		int32_t rightDistance = rightEncoder->GetDistance();
  		int32_t leftDistance = leftEncoder->GetDistance();

  		cout << "Right: " << rightDistance << endl;
  		cout << "Left: " << leftDistance << endl;
		cout << endl;
  	}

  	void DisabledContinuous(void) {}
  	void AutonomousContinuous(void) {}
  	void TeleopContinuous(void) {}
};

START_ROBOT_CLASS(RobotDemo)
