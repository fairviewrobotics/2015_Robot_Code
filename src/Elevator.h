/*
 * Elevator.h
 *
 *  Created on: Feb 10, 2015
 *      Author: FHS Programming
 */

#ifndef SRC_ELEVATOR_H_
#define SRC_ELEVATOR_H_
#include "WPILib.h"
#include "TRCDefs.h"

class Elevator {
	private:
		bool m_isRising = false;
		bool m_isStationary = true;
		bool m_isOperating = false;
	public:

};

// Constructor
Movement::Movement(bool turn, float distOrAngle, PIDController* leftPID, PIDController* rightPID) {

}

#endif /* SRC_ELEVATOR_H_ */
