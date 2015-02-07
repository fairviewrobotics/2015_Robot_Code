#include "WPILib.h"

#ifndef SRC_MOVEMENT_H_
#define SRC_MOVEMENT_H_

class Movement {
	private:
		bool           m_finished;
		bool           m_turn;
		float          m_distOrAngle;
		PIDController* m_leftPIDControl;
		PIDController* m_rightPIDControl;
	public:
		Movement(bool turn, float distOrAngle, PIDController* leftPID, PIDController* rightPID);
		bool IsComplete(void);
};

// Constructor
Movement::Movement(bool turn, float distOrAngle, PIDController* leftPID, PIDController* rightPID) {
	m_turn = turn;
	m_distOrAngle = distOrAngle;
	m_leftPIDControl = leftPID;
	m_rightPIDControl = rightPID;
	m_finished = false;
}

/**
 * This function is called to check if the movement has been finished.
 *
 * @return Returns true if the movement is finished, false otherwise.
 */
bool Movement::IsComplete(void) {
	return m_finished;
}

#endif /* SRC_MOVEMENT_H_ */
