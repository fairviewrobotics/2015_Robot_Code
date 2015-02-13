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
		// Check to bin decides which Limit Switch it will rise to, or stop to.
		//   True = stop at bin switch, false is to tote switch
		bool m_checkToBin;
		// If true, the elevator will lower to the ground, if false, will bring to max height, or to carry.
		bool m_isOperating = false;
		Victor* m_elevator1;
		Victor* m_elevator2;

		DigitalInput* m_bottomSwitch;
		DigitalInput* m_middleSwitch;



		void downElevatorTote(void);
		void downElevatorBin(void);
		void upOneTote();
		void downOneTote(void);
	public:
		Elevator(bool rising,bool toBin, Victor* el1, Victor* el2, DigitalInput* bottomSwitch, DigitalInput* middleSwitch);

		bool IsComplete();
		bool IsMoving();
		void DoMovement();
};

// Constructor
Elevator::Elevator(bool rising, bool toBin,  Victor* el1, Victor* el2, DigitalInput* bottomSwitch, DigitalInput* middleSwitch) {
	m_checkToBin = toBin;
	m_isRising = rising;
	m_elevator1 = el1;
	m_elevator2 = el2;
	m_bottomSwitch = bottomSwitch;
	m_middleSwitch = middleSwitch;

}
void doMovement(){
	if(){

	}else if(){

	}else if(){

	}
}
void downElevatorTote(){

}
void downElevatorBin(){

}
void upOneTote(){

}
void downOneTote(){

}
#endif /* SRC_ELEVATOR_H_ */
