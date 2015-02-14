#include "WPILib.h"
#include "TRCDefs.h"

#ifndef SRC_MOVEMENT_H_
#define SRC_MOVEMENT_H_

class Movement {
	private:
		bool           m_turn;
		float          m_distOrAngle;
		PIDController* m_leftPIDControl;
		PIDController* m_rightPIDControl;

		void DriveTo(void);
		void AngleTo(void);
		float AngleToSetpoint(float angle);

	public:
		bool           m_finished = false;
		bool		   m_isOperating = false;
		Movement(bool turn, float distOrAngle, PIDController* leftPID, PIDController* rightPID);
		bool IsComplete(void);
		bool IsRunning(void);
		void DoMovement(void);
		float GetError(void);
};

// Constructor
Movement::Movement(bool turn, float distOrAngle, PIDController* leftPID, PIDController* rightPID) {
	m_turn = turn;
	m_distOrAngle = distOrAngle;

	m_leftPIDControl = leftPID;
	m_rightPIDControl = rightPID;
}

bool Movement::IsComplete(void) {
	m_finished = (ABS(m_rightPIDControl->GetError()) > 5.0) && (ABS(m_leftPIDControl->GetError()) > 5.0);
	return m_finished;
}

bool Movement::IsRunning(void) {
	return m_isOperating;
}

void Movement::DoMovement(void) {
	m_isOperating = false;
	m_rightPIDControl->Enable();
	m_leftPIDControl->Enable();

	m_rightPIDControl->Reset();
	m_leftPIDControl->Reset();

	printf("PID Reset");

	if(m_turn) {
		AngleTo();
	} else {
		DriveTo();
	}
}

void Movement::DriveTo(void) {
	m_isOperating = true;

	m_rightPIDControl->SetSetpoint(m_distOrAngle);
	m_leftPIDControl->SetSetpoint(m_distOrAngle);
}

void Movement::AngleTo(void) {
	m_isOperating = true;
	float angleDistance = AngleToSetpoint(m_distOrAngle);

	m_rightPIDControl->SetSetpoint(angleDistance);
	m_leftPIDControl->SetSetpoint(-angleDistance);
}

float Movement::AngleToSetpoint(float angle) {
	// Robot Radius Turn is 11.75
	return (angle * 11.75 * (PI / 180));
}

float Movement::GetError(void){
	float rightError = m_rightPIDControl -> GetError();
	return rightError;
}

#endif /* SRC_MOVEMENT_H_ */
