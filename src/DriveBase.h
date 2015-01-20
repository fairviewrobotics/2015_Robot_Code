#ifndef _DRIVEBASE_H_
#define _DRIVEBASE_H_

class DriveBase;

class DriveBase : public PIDInput {
private:
	float yPos;
	float rotPos;
	TrcPIDCtrl yPidCtrl;
	TrcPIDCtrl turnPidCtrl;

public:
	DriveBase(
		SpeedController *leftMotor,
		SpeedController *rightMotor,
		TrcPIDCtrl *yPidCtrl,
		TrcPIDCtrl *turnPidCtrl
	): RobotDrive(leftMotor, rightMotor) {
		// meh
	}
	~DriveBase();

	float GetInput(TrcPIDCtrl *pidCtrl) {
		float input = 0.0;

		if (pidCtrl == yPidCtrl) {
			input = yPos;
		} else if (pidCtrl == turnPidCtrl) {
			input = rotPos;
		}

		return input;
	}
};

#endif /* SRC_DRIVEBASE_H_ */
