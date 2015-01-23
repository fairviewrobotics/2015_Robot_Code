#include "WPILib.h"
#include <Encoder.h>
#include <Counter.h>
#include "Commands/Command.h"
#include <string>
#include <iostream>
#include <PIDController.h>
#include <TrcDefs.h>
#include <DistanceEncoder.h>
#include <RobotDriveOutput.h>

#define __PRINT_COMMAND_H__
#define DRIVE_ENCODER_PPR 2048

using namespace std;

// const double PI = 3.14159265359;

class RobotDemo : public IterativeRobot {
	// Drive system motor controllers
    Talon *rightTalon;
    Talon *leftTalon;

    Victor *elevator;

    Encoder *rightEncoder;
    Encoder *leftEncoder;

    Joystick *controllerLeft;
    Joystick *controllerRight;

    RobotDrive *robotDrive;

    // TrcPIDCtrl *pidCtrlDrive;
    // TrcPIDCtrl *pidCtrlTurn;
    PIDController *rightPID;
    PIDController *leftPID;

public:
    RobotDemo(void) {
    	leftTalon  = new Talon(0);
  		rightTalon = new Talon(1);

  		// pidCtrlDrive = new TrcPIDCtrl(YDRIVE_KP, YDRIVE_KI, YDRIVE_KD, YDRIVE_KF, YDRIVE_TOLERANCE, YDRIVE_SETTLING);
  		// pidCtrlTurn = new TrcPIDCtrl(TURN_KP, TURN_KI, TURN_KD, TURN_KF, TURN_TOLERANCE, TURN_SETTLING);

  		robotDrive = new RobotDrive(rightTalon, leftTalon);
  		robotDrive->SetInvertedMotor(RobotDrive::kFrontLeftMotor, true);
  		robotDrive->SetInvertedMotor(RobotDrive::kRearLeftMotor, true);

  		leftEncoder = new Encoder(0, 1, false, Encoder::EncodingType::k4X);
  		rightEncoder = new Encoder(2, 3, false, Encoder::EncodingType::k4X);

  		controllerLeft  = new Joystick(0);
  		controllerRight = new Joystick(1);

  		elevator = new Victor(2);

  		rightPID = new PIDController(0.2, 0.0, 0.0, rightEncoder, rightTalon);
  		leftPID =  new PIDController(0.2, 0.0, 0.0, leftEncoder, leftTalon);
    }

  	/********************************** Init Routines *************************************/

  	void RobotInit(void) {
  		rightEncoder->SetDistancePerPulse(PI*4/360.0);
  		leftEncoder->SetDistancePerPulse(PI*4/360.0);
  	}

  	void DisabledInit(void) {
  		rightPID->Reset();
  		leftPID->Reset();
  	}

  	void AutonomousInit(void) {
  		rightPID->Enable();
  		leftPID->Enable();

  		rightPID->SetSetpoint(1.5);
  		leftPID->SetSetpoint(1.5);
  	}

  	void TeleopInit(void) {
  		// Set all motor controllers to be not moving initially.
  		leftTalon->SetSpeed(0.0);
  		rightTalon->SetSpeed(0.0);
  	}

  	/********************************** Periodic Routines *************************************/

  	void DisabledPeriodic(void) {}

  	void AutonomousPeriodic(void) {
  		cout << "Error: " << rightPID->GetError() << " Setpoint: " << rightPID->GetSetpoint() << endl;
  	}

  	void TeleopPeriodic(void) {
  		robotDrive->TankDrive(controllerLeft, controllerRight);

  		if (controllerRight->GetTrigger()) {
  			elevator->SetSpeed(0.3);
  		} else if (controllerLeft->GetTrigger()) {
  			elevator->SetSpeed(-0.3);
  		} else {
  			elevator->SetSpeed(0.0);
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
