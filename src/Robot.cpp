#include "WPILib.h"

using namespace std;

class RobotDemo : public IterativeRobot {
	// Drive system motor controllers
    Talon *rightTalon;
    Talon *leftTalon;

    Encoder *rightEncoder;
    Encoder *leftEncoder;

    Joystick *controllerLeft;
    Joystick *controllerRight;

public:
    RobotDemo(void) {

  		rightTalon = new Talon(1);
  		leftTalon  = new Talon(0);

  		rightEncoder = new Encoder(2,3,true,Encoder::EncodingType::k4X);
  		leftEncoder = new Encoder(0,1, false,Encoder::EncodingType::k4X);

  		controllerLeft  = new Joystick(0);
  		controllerRight = new Joystick(1);

    }


  	/********************************** Init Routines *************************************/

  	void RobotInit(void) {

  	}

  	void DisabledInit(void) {
  	}

  	void AutonomousInit(void) {
  	}

  	void TeleopInit(void) {
  		// Set all motor controllers to be not moving.
  		leftTalon->SetSpeed(0.0);
  		rightTalon->SetSpeed(0.0);
  	}


  	/********************************** Periodic Routines *************************************/

  	void DisabledPeriodic(void) {}
  	void AutonomousPeriodic(void) {
  		int32_t rightDistance = rightEncoder->GetDistance();
  		int32_t leftDistance = leftEncoder->GetDistance();
  		if(abs(rightDistance)<10240) rightTalon->SetSpeed(1.0);
  		else rightTalon->SetSpeed(0.0);
  		if(abs(leftDistance)<10240) leftTalon->SetSpeed(1.0);
  		else leftTalon->SetSpeed(0.0);

  	}
  	void TeleopPeriodic(void) {

  		float leftStick  = controllerLeft->GetRawAxis(1);  // Drive system
  		float rightStick = controllerRight->GetRawAxis(1); // Drive system


  		leftTalon->SetSpeed(-leftStick);
  		rightTalon->SetSpeed(rightStick);

  		int32_t rightDistance = rightEncoder->GetDistance();
  		int32_t leftDistance = leftEncoder->GetDistance();

  		cout << "Right Encoder: " << rightDistance << "   Left Encoder: " << leftDistance << endl;

  	}
  	void DisabledContinuous(void) {}
  	void AutonomousContinuous(void) {}
  	void TeleopContinuous(void) {}
  };

  START_ROBOT_CLASS(RobotDemo)
