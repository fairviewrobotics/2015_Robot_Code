#ifndef _DRIVEBASE_H_
#define _DRIVEBASE_H_

class DriveBase: public PIDInput, public RobotDrive {
private:
	float yPos;
	float rotPos;
    float yDistPerRev;
    float degPerRev;
	TrcPIDCtrl yPidCtrl;
	TrcPIDCtrl turnPidCtrl;

public:
	DriveBase(
		SpeedController *leftMotor,
		SpeedController *rightMotor,
		TrcPIDCtrl *yPidCtrl,
		TrcPIDCtrl *turnPidCtrl
	): RobotDrive(leftMotor, rightMotor) {
		yPos = 0.0;
		rotPos = 0.0;
		yDistPerRev = 1.0;
		degPerRev = 360.0;

		SetExpiration(1.0);
		SetSafetyEnabled(false);
	}

	// Destructor
    ~DriveBase(void) {
        SetSafetyEnabled(false);
        Stop();
    }

	float GetInput(TrcPIDCtrl *pidCtrl) {
		float input = 0.0;

		if (pidCtrl == yPidCtrl) {
			input = yPos;
		} else if (pidCtrl == turnPidCtrl) {
			input = rotPos;
		}

		return input;
	}

    /**
     * This function sets PID drive target with the given drive distance and
     * turn angle setpoints.
     *
     * @param distYSetPoint Specifies the target distance relative to current
     *        distance.
     * @param angleSetPoint Specifies the target angle relative to current
     *        angle.
     * @param fHoldTarget Optionally specifies if PIDDrive will maintain target
     *        or will stop when target is reached.
     * @param notifyEvent Specifies the event to notifying for completion.
     * @param timeout Specifies the timeout in msec. No timeout if zero.
     */
	void DriveSetTarget(
    	float  distXSetPoint,
    	float  distYSetPoint,
    	float  angleSetPoint,
    	bool   fHoldTarget = false,
	    Event *notifyEvent = NULL,
	    uint32_t timeout = 0
	) {
	    pidDrive->SetTarget(distXSetPoint,
	                        distYSetPoint,
	                        angleSetPoint,
	                        fHoldTarget,
	                        notifyEvent,
	                        timeout);
	    return;
	}

    // This function stops the drive base.
    void Stop(void) {
        pidDrive->Stop();
    }
};

#endif /* SRC_DRIVEBASE_H_ */
