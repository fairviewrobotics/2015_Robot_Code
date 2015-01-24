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
    Talon *noodlerTalon;


    Victor *elevator;

    bool *elevatorinuse;
    bool *elevatorup;
    bool *noodlerinuse;
    bool *noodlerdir;


    Encoder *rightEncoder;
    Encoder *leftEncoder;
    DistanceEncoder *rightDistance;
    DistanceEncoder *leftDistance;

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
  		noodlerTalon = new Talon(2);
  		// pidCtrlDrive = new TrcPIDCtrl(YDRIVE_KP, YDRIVE_KI, YDRIVE_KD, YDRIVE_KF, YDRIVE_TOLERANCE, YDRIVE_SETTLING);
  		// pidCtrlTurn = new TrcPIDCtrl(TURN_KP, TURN_KI, TURN_KD, TURN_KF, TURN_TOLERANCE, TURN_SETTLING);

  		//robot-state booleans
  		elevatorinuse = false;
  		elevatorup = true;
  		noodlerinuse = false;
  		noodlerdir=false;

  		robotDrive = new RobotDrive(rightTalon, leftTalon);
  		robotDrive->SetInvertedMotor(RobotDrive::kFrontLeftMotor, true);
  		robotDrive->SetInvertedMotor(RobotDrive::kRearLeftMotor, true);

  		leftEncoder = new Encoder(0, 1, true, Encoder::EncodingType::k4X);
  		rightEncoder = new Encoder(2, 3, true, Encoder::EncodingType::k4X);

  		leftDistance = new DistanceEncoder(leftEncoder);
  		rightDistance = new DistanceEncoder(rightEncoder);

  		controllerLeft  = new Joystick(0);
  		controllerRight = new Joystick(1);

  		elevator = new Victor(2);
  		//elevator_up = false;
  		rightPID = new PIDController(0.005, 0.0, 0.04, rightDistance, rightTalon);


  		leftPID =  new PIDController(0.005, 0.0, 0.04, leftDistance, leftTalon);
  		rightPID = new PIDController(0.005, 0.0, 0.04, rightDistance, rightTalon);

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

  		rightPID->SetSetpoint(2000);
  		leftPID->SetSetpoint(2000);
  	}

  	void TeleopInit(void) {
  		// Set all motor controllers to be not moving initially.
  		elevator->SetSpeed(0.0);
  		leftTalon->SetSpeed(0.0);
  		rightTalon->SetSpeed(0.0);
  	}

  	/********************************** Periodic Routines *************************************/

  	void DisabledPeriodic(void) {}

  	void AutonomousPeriodic(void) {
  		cout << "Right error: " << rightPID->GetError() << " Setpoint: " << rightPID->GetSetpoint() << endl;
  		cout << "Left error: " << leftPID->GetError() << " Setpoint: " << leftPID->GetSetpoint() << endl;
  	}

  	void TeleopPeriodic(void) {
  		robotDrive->TankDrive(controllerLeft, controllerRight);

  		//UNTESTED
  		//new elevator code so that later we can add PID control loops to it, also toggle-able code, probably not the best way to do this...
  		if (controllerRight->GetTrigger()) {
  			elevatorinuse = true;
  			elevatorup = true;
  		}else if(controllerLeft->GetTrigger()){
  			elevatorinuse = true;
  			elevatorup = false;
  		}else{
  			elevatorinuse = false;
  		}

  		//toggle buttons for noodler,
  		if(controllerRight->GetRawButton(4)){
  			noodlerinuse = -noodlerinuse;
  			noodlerdir = true;
  		}else if(controllerLeft->GetRawButton(4)){
  			noodlerinuse = -noodlerinuse;
  			noodlerdir = false;
  		}

  		//Utility checkers
  		//to add another || so that it stops once it reaches a certain point
  		//
  		if(elevatorinuse){
  			if(elevatorup){
  				elevator->SetSpeed(.03);
  			}else{
  				elevator->SetSpeed(-.03);
  			}
  		}

  		if(noodlerinuse){
  			if(noodlerdir){
  				noodlerTalon->SetSpeed(.3);
  			}else{
  				noodlerTalon->SetSpeed(-.3);
  			}

  		}else{
  			noodlerTalon->SetSpeed(0);
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
