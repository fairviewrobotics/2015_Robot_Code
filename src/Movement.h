#include "WPILib.h"
#include "TRCDefs.h"

#ifndef SRC_MOVEMENT_H_
#define SRC_MOVEMENT_H_

class Movement {
	private:
		bool           		m_turn;
		float          		m_distOrAngle;
		float				m_leftTo;
		float				m_rightTo;
		float				m_leftBegining;
		float				m_rightBegining;
		bool           		m_finished = false;
		bool		   		m_isOperating = false;
		Talon*  	   		m_rightTalon;
		Talon*		   		m_leftTalon;
		DistanceEncoder*	m_leftDistance;
		DistanceEncoder*	m_rightDistance;

		void DriveTo(void);
		void AngleTo(void);
		float AngleToSetpoint(float angle);

	public:
		Movement(bool turn, float distOrAngle, Talon* leftTalon, Talon* rightTalon, DistanceEncoder* leftDistance, DistanceEncoder* rightDistance);
		bool IsComplete(void);
		bool IsRunning(void);
		void DoMovement(void);
		float GetError(void);
};

// Constructor
Movement::Movement(bool turn, float distOrAngle, Talon* leftTalon, Talon* rightTalon, DistanceEncoder* leftDistance, DistanceEncoder* rightDistance) {
	m_turn = turn;
	m_distOrAngle = distOrAngle;
	m_rightTalon = rightTalon;
	m_leftTalon = leftTalon;
	m_leftDistance = leftDistance;
	m_rightDistance = rightDistance;
	m_rightBegining = m_rightDistance->Get();
	m_leftBegining = m_leftDistance->Get();
	//the driving to values start with the Begining then get changed in angle and drive to
	m_rightTo = m_rightDistance->Get();
	m_leftTo = m_leftDistance->Get();
}

bool Movement::IsComplete(void) {
	m_finished = (ABS(m_rightDistance->Get() - m_rightTo) < 5) && (ABS(m_leftDistance->Get() - m_leftTo) < 5);
	if (m_finished) {
		m_isOperating = false;
		m_rightTalon->SetSpeed(0.0);
		m_leftTalon->SetSpeed(0.0);
	}
	return m_finished;
}

bool Movement::IsRunning(void) {
	return m_isOperating;
}

void Movement::DoMovement(void) {
	m_isOperating = false;

	if(m_turn) {
		AngleTo();
	} else {
		DriveTo();
	}
}

void Movement::DriveTo(void) {
	m_isOperating = true;

	m_rightTo += m_distOrAngle;
	m_leftTo += m_distOrAngle;

	m_rightTalon->SetSpeed(1.0);
	m_leftTalon->SetSpeed(1.0);
}

void Movement::AngleTo(void) {
	m_isOperating = true;
	float angleDistance = AngleToSetpoint(m_distOrAngle);

	m_rightTo += angleDistance;
	m_leftTo -= angleDistance;
}

float Movement::AngleToSetpoint(float angle) {
	// Robot Radius Turn is 11.75
	return (angle * 11.75 * (PI / 180));
}

float Movement::GetError(void) {
	return ABS(m_rightDistance->Get()) + ABS(m_leftDistance->Get());
}


#endif /* SRC_MOVEMENT_H_ */
