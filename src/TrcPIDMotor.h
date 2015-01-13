#if 0
/// Copyright (c) Titan Robotics Club. All rights reserved.
///
/// <module name="TrcPIDMotor.h" />
///
/// <summary>
///     This module contains the definition and implementation of the
///     TrcPIDMotor class.
/// </summary>
///
/// <remarks>
///     Environment: Wind River C++ for National Instrument cRIO based Robot.
/// </remarks>
#endif

#ifndef _TRCPIDMOTOR_H
#define _TRCPIDMOTOR_H

#ifdef MOD_ID
    #undef MOD_ID
#endif
#define MOD_ID                  MOD_PIDMOTOR
#ifdef MOD_NAME
    #undef MOD_NAME
#endif
#define MOD_NAME                "TrcPIDMotor"

/**
 * This class defines and implements the TrcPIDMotor object. It supports the
 * capability of PID controlled movement by a motor. It drives the motor to
 * the set target position and optionally signals the notification event for
 * notifying completion.
 * The PIDMotor object consists of a motor speed controller, a PID controller
 * object and a PIDInput object to provide feedback.
 */
class TrcPIDMotor: public CoopTask
{
private:
    //
    // Flags
    //
    #define PIDMOTORF_PIDMODE_ON        0x00000001
    #define PIDMOTORF_HOLD_TARGET       0x00000002
    #define PIDMOTORF_STALLED           0x00000004

    #define PIDMOTORO_INVERSE           0x00000001

    SpeedController *m_motor1;
    SpeedController *m_motor2;
    UINT8            m_syncGroup;
    TrcPIDCtrl      *m_pidCtrl;
    PIDInput        *m_pidInput;
    uint32_t           m_pidMotorOptions;
    uint32_t           m_pidMotorFlags;
    Event           *m_notifyEvent;
    float            m_motorPower;
    uint32_t           m_expiredTime;
    float            m_prevInput;
    uint32_t           m_prevTime;
    float            m_calPower;
    float            m_zeroPosition;

public:
    /**
     * This function resets the PID Motor object.
     *
     * @param fStopMotors Optionally specifies if the motors should be stopped.
     */
    void
    Reset(
        bool fStopMotors = true
        )
    {
        TLevel(API);
        TEnter();

        if (fStopMotors)
        {
            if (m_motor1 != NULL)
            {
                m_motor1->Set(0.0);
            }

            if (m_motor2 != NULL)
            {
                m_motor2->Set(0.0);
                ((CANJaguar*)m_motor2)->UpdateSyncGroup(m_syncGroup);
            }
        }

        if (m_pidCtrl != NULL)
        {
            m_pidCtrl->Reset();
        }

        m_pidMotorFlags = 0;
        m_notifyEvent = NULL;
        m_motorPower = 0.0;
        m_expiredTime = 0;
        m_prevInput = 0.0;
        m_prevTime = 0;
        m_calPower = 0.0;

        TExit();
        return;
    }   //Reset

    /**
     * This function is called by TaskMgr to stop the PID motor.
     *
     * @param mode Specifies the CoopTask callback types.
     */
    void
    TaskStopMode(
        uint32_t mode
        )
    {
        TLevel(CALLBK);
        TEnterMsg(("mode=%d", mode));

        Reset(true);

        TExit();
        return;
    }   //TaskStopMode

    /**
     * Constructor: Create an instance of the TrcPIDMotor object that consists
     * of a SpeedController object, a TrcPIDCtrl object and optionally a
     * notification event for signaling completion.
     *
     * @param motor Points to the SpeedController object.
     * @param pidCtrl Points to the TrcPIDCtrl object.
     * @param pidInput Specifies the PIDInput object.
     * @param pidMotorOptions Optionally specifies the motor options.
     */
    TrcPIDMotor(
        SpeedController *motor,
        TrcPIDCtrl      *pidCtrl,
        PIDInput        *pidInput,
        uint32_t           pidMotorOptions = 0
        ): m_motor1(motor)
         , m_motor2(NULL)
         , m_syncGroup(0)
         , m_pidCtrl(pidCtrl)
         , m_pidInput(pidInput)
         , m_pidMotorOptions(pidMotorOptions)
         , m_pidMotorFlags(0)
         , m_notifyEvent(NULL)
         , m_motorPower(0.0)
         , m_expiredTime(0)
         , m_prevInput(0.0)
         , m_prevTime(0)
         , m_calPower(0.0)
         , m_zeroPosition(0.0)
    {
        TLevel(INIT);
        TEnterMsg(("motor=%p,pidCtrl=%p,pidInput=%p,options=%x",
                   motor, pidCtrl, pidInput, pidMotorOptions));

        RegisterTask(MOD_NAME, TASK_STOP_MODE | TASK_POST_PERIODIC);

        TExit();
    }   //TrcPIDMotor

    /**
     * Constructor: Create an instance of the TrcPIDMotor object that consists
     * of two SpeedController objects, a TrcPIDCtrl object and optionally a
     * notification event for signaling completion.
     *
     * @param motor1 Points to the first SpeedController object.
     * @param motor2 Points to the second SpeedController object.
     * @param syncGroup Specifies the sync group the two motors are in.
     * @param pidCtrl Points to the TrcPIDCtrl object.
     * @param pidInput Specifies the PIDInput object.
     * @param pidMotorOptions Optionally specifies the motor options.
     */
    TrcPIDMotor(
        SpeedController *motor1,
        SpeedController *motor2,
        UINT8            syncGroup,
        TrcPIDCtrl      *pidCtrl,
        PIDInput        *pidInput,
        uint32_t           pidMotorOptions = 0
        ): m_motor1(motor1)
         , m_motor2(motor2)
         , m_syncGroup(syncGroup)
         , m_pidCtrl(pidCtrl)
         , m_pidInput(pidInput)
         , m_pidMotorOptions(pidMotorOptions)
         , m_pidMotorFlags(0)
         , m_notifyEvent(NULL)
         , m_motorPower(0.0)
         , m_expiredTime(0)
         , m_prevInput(0.0)
         , m_prevTime(0)
         , m_calPower(0.0)
         , m_zeroPosition(0.0)
    {
        TLevel(INIT);
        TEnterMsg(("motor1=%p,motor2=%p,syncGrp=%d,pidCtrl=%p,pidInput=%p,options=%x",
                   motor1, motor2, syncGroup, pidCtrl, pidInput,
                   pidMotorOptions));

        RegisterTask(MOD_NAME, TASK_STOP_MODE | TASK_POST_PERIODIC);

        TExit();
    }   //TrcPIDMotor

    /**
     * Destructor: Destroy an instance of the TrcPIDMotor object.
     */
    virtual
    ~TrcPIDMotor(
        void
        )
    {
        TLevel(INIT);
        TEnter();

        Reset(true);
        UnregisterTask();

        TExit();
    }   //~TrcPIDMotor

    /**
     * This function sets PID motor power.
     *
     *  @param power Specifies the power level to set the motor.
     *  @param lowerBound Optionally specifies the lower bound of the motor
     *         power.
     *  @param upperBound Optionally specifies the upper bound of the motor
     *         power.
     *  @param stallMinPower Optionally specifies stall detection minimum
     *         power.
     *  @param stallTimeout Optionally specifies stall detection timeout.
     *  @param resetTimeout Optionally specifies stall reset timeout.
     */
    void
    SetPower(
        float power,
        float lowerBound = MOTOR_RANGE_MIN,
        float upperBound = MOTOR_RANGE_MAX,
        float stallMinPower = 0.0,
        uint32_t stallTimeout = 0,
        uint32_t resetTimeout = 0
        )
    {
        TLevel(API);
        TEnterMsg(("power=%f,lowBound=%f,upBound=%f,stallMinPower=%f,stallTimeout=%d,resetTimeout=%d",
                   power, lowerBound, upperBound, stallMinPower, stallTimeout,
                   resetTimeout));

        if (m_pidMotorFlags & PIDMOTORF_PIDMODE_ON)
        {
            //
            // There was a previous unfinished PID operation, cancel it.
            // Don't stop the motor to prevent jerkiness.
            //
            Reset(false);
        }

        if (m_pidMotorOptions & PIDMOTORO_INVERSE)
        {
            power *= -1.0;
        }
        power = BOUND(power, lowerBound, upperBound);
        if (m_pidMotorFlags & PIDMOTORF_STALLED)
        {
            if (power == 0.0)
            {
                //
                // Clear the stall condition if power is zero.
                //
                if ((resetTimeout == 0) ||
                    (GetMsecTime() - m_prevTime > resetTimeout))
                {
                    m_prevInput = m_pidInput->GetInput(m_pidCtrl);
                    m_prevTime = GetMsecTime();
                    m_pidMotorFlags &= ~PIDMOTORF_STALLED;
                }
            }
            else
            {
                m_prevTime = GetMsecTime();
            }
        }
        else
        {
            m_motorPower = power;
            if ((stallMinPower > 0) && (stallTimeout > 0))
            {
                //
                // Stall protection is ON, check for stall condition.
                // - power is above stallMinPower
                // - motor has not moved for at least stallTimeout
                //
                float currInput = m_pidInput->GetInput(m_pidCtrl);
                if ((fabs(power) < fabs(stallMinPower)) ||
                    (currInput != m_prevInput))
                {
                    m_prevInput = currInput;
                    m_prevTime = GetMsecTime();
                }

                if (GetMsecTime() - m_prevTime > stallTimeout)
                {
                    //
                    // We have detected a stalled condition for at least
                    // stallMinTime. Kill the power to protect the motor.
                    //
                    m_motorPower = 0.0;
                    m_pidMotorFlags |= PIDMOTORF_STALLED;
                }
            }

            if (m_motor1 != NULL)
            {
                m_motor1->Set(m_motorPower);
            }

            if (m_motor2 != NULL)
            {
                m_motor2->Set(m_motorPower);
                ((CANJaguar*)m_motor2)->UpdateSyncGroup(m_syncGroup);
            }
        }

        TExit()
        return;
    }   //SetPower

    /**
     *  This function performs zero point calibration.
     *
     *  @param calPower Specifies the motor power for the calibration.
     */
    void
    ZeroCalibrate(
        float calPower
        )
    {
        TLevel(API);
        TEnterMsg(("calPower=%f", calPower));

        m_calPower = calPower;

        TExit();
        return;
    }   //ZeroCalibrate

    /**
     * This function sets PID motor target with the given setpoint.
     *
     * @param setPoint Specifies the target setPoint.
     * @param fHoldTarget If true, maintain target. Otherwise, will stop when
     *        target is reached.
     * @param notifyEvent Specifies the event to notifying for completion.
     * @param timeout Specifies the timeout in msec. No timeout if zero.
     */
    void
    SetTarget(
        float  setPoint,
        bool   fHoldTarget = false,
        Event *notifyEvent = NULL,
        uint32_t timeout = 0
        )
    {
        TLevel(API);
        TEnterMsg(("setPoint=%f,fHoldTarget=%x,event=%p,timeout=%d",
                   setPoint, fHoldTarget, notifyEvent, timeout));

        if (m_pidMotorFlags & PIDMOTORF_PIDMODE_ON)
        {
            //
            // Previous SetTarget has not been completed, cancel it.
            //
            Reset(false);
        }

        m_notifyEvent = notifyEvent;
        m_expiredTime = (timeout != 0)? GetMsecTime() + timeout: 0;
        m_pidCtrl->SetTarget(setPoint, m_pidInput->GetInput(m_pidCtrl));
        m_pidMotorFlags = PIDMOTORF_PIDMODE_ON;
        if (fHoldTarget)
        {
            m_pidMotorFlags |= PIDMOTORF_HOLD_TARGET;
        }

        TExit();
        return;
    }   //SetTarget

    /**
     * This function is called by the TaskMgr to update the PIDMotor state
     * and check for completion.
     *
     * @param mode Specifies the CoopTask callback types.
     */
    void
    TaskPostPeriodic(
        uint32_t mode
        )
    {
        TLevel(TASK);
        TEnterMsg(("mode=%d", mode));

        if ((m_motor1 != NULL) && (m_calPower != 0.0))
        {
            //
            // We are in zero calibration mode.
            //
            if ((m_calPower < 0.0) &&
                ((CANJaguar *)m_motor1)->GetReverseLimitOK() ||
                (m_calPower > 0.0) &&
                ((CANJaguar *)m_motor1)->GetForwardLimitOK())
            {
                m_motor1->Set(m_calPower);
                if (m_motor2 != NULL)
                {
                    m_motor2->Set(m_calPower);
                    ((CANJaguar*)m_motor2)->UpdateSyncGroup(m_syncGroup);
                }
            }
            else
            {
                //
                // Done calibrating.
                //
                m_calPower = 0.0;
                m_motor1->Set(0.0);
                if (m_motor2 != NULL)
                {
                    m_motor2->Set(0.0);
                    ((CANJaguar*)m_motor2)->UpdateSyncGroup(m_syncGroup);
                }
                m_zeroPosition = ((CANJaguar *)m_motor1)->GetPosition();
            }
        }
        else if (m_pidMotorFlags & PIDMOTORF_PIDMODE_ON)
        {
            if (!(m_pidMotorFlags & PIDMOTORF_HOLD_TARGET) &&
                m_pidCtrl->OnTarget() ||
                (m_expiredTime != 0) && (GetMsecTime() >= m_expiredTime))
            {
                Reset(true);
                if (m_notifyEvent != NULL)
                {
                    m_notifyEvent->SetEvent();
                }
            }
            else
            {
                m_motorPower = m_pidCtrl->CalcPIDOutput(
                                        m_pidInput->GetInput(m_pidCtrl));
                if (m_pidMotorOptions & PIDMOTORO_INVERSE)
                {
                    m_motorPower *= -1.0;
                }
                m_motor1->Set(m_motorPower);
                if (m_motor2 != NULL)
                {
                    m_motor2->Set(m_motorPower);
                    ((CANJaguar*)m_motor2)->UpdateSyncGroup(m_syncGroup);
                }
                TSampling(("MotorOutput: %f", m_motorPower));
            }
        }

        TExit();
        return;
    }   //TaskPostPeriodic

};  //class TrcPIDMotor

#endif  //ifndef _TRCPIDMOTOR_H
