#if 0
/// Copyright (c) Titan Robotics Club. All rights reserved.
///
/// <module name="TrcPIDDrive.h" />
///
/// <summary>
///     This module contains the definition and implementation of the
///     TrcPIDDrive class.
/// </summary>
///
/// <remarks>
///     Environment: Wind River C++ for National Instrument cRIO based Robot.
/// </remarks>
#endif

#ifndef _TRCPIDDRIVE_H
#define _TRCPIDDRIVE_H

#ifdef MOD_ID
    #undef MOD_ID
#endif
#define MOD_ID                  MOD_PIDDRIVE
#ifdef MOD_NAME
    #undef MOD_NAME
#endif
#define MOD_NAME                "TrcPIDDrive"

/**
 * This class defines and implements the TrcPIDDrive object. It supports the
 * capability of PID controlled drive by X and Y distance and/or PID controlled
 * turn by angle so that it can work with the mecanum drive train. After the
 * operation is completed, it has an option of notifying the caller by setting
 * an event. Therefore, it can work with the state machine to do autonomous
 * drive.
 * The PID Drive object consists of a drive object, 3 PID controllers (one
 * for X, one for Y and one for rotation) and a PID input object to provide
 * feedback.
 */
class TrcPIDDrive
{
private:
    //
    // Flags
    //
    #define PIDDRIVEF_PIDDRIVE_ON       0x00000001
    #define PIDDRIVEF_HOLD_TARGET       0x00000002
    #define PIDDRIVEF_TURN_ONLY         0x00000004
    #define PIDDRIVEF_ABORTED           0x00000008
    #define PIDDRIVEF_MANUAL_DRIVE      0x00000010

    RobotDrive *m_drive;
    TrcPIDCtrl *m_pidCtrlXDrive;
    TrcPIDCtrl *m_pidCtrlYDrive;
    TrcPIDCtrl *m_pidCtrlTurn;
    PIDInput   *m_pidInput;
    uint32_t      m_pidDriveFlags;
    Event      *m_notifyEvent;
    uint32_t      m_expiredTime;
    float       m_xPower;
    float       m_yPower;

public:
    /**
     * This function stops the PID Drive object.
     */
    void
    Stop(
        void
        )
    {
        m_drive->StopMotor();

        if (m_pidCtrlXDrive != NULL)
        {
            m_pidCtrlXDrive->Reset();
        }

        if (m_pidCtrlYDrive != NULL)
        {
            m_pidCtrlYDrive->Reset();
        }

        if (m_pidCtrlTurn != NULL)
        {
            m_pidCtrlTurn->Reset();
        }

        m_pidDriveFlags = 0;

        return;
    }   //Stop

    /**
     * This function is called by TaskMgr to stop the PID drive task.
     *
     * @param mode Specifies the CoopTask callback types.
     */
    void
    TaskStopMode(
        uint32_t mode
        )
    {
        Stop();

        return;
    }   //TaskStopMode

    /**
     * Constructor: Create an instance of the TrcPIDDrive object that consists
     * of a RobotDrive object, a TrcPIDCtrl object for drive, a TrcPIDCtrl
     * object for turn and optionally a notification object for the callback.
     *
     * @param drive Points to the RobotDrive object.
     * @param pidCtrlXDrive Points to the TrcPIDCtrl object for driving
     *        sideway (only use if drive train is mecanum).
     * @param pidCtrlYDrive Points to the TrcPIDCtrl object for driving
     *        forward and backward.
     * @param pidCtrlTurn Points to the TrcPIDCtrl object for turning.
     * @param pidInput Specifies the PIDInput object.
     */
    TrcPIDDrive(
        RobotDrive *drive,
        TrcPIDCtrl *pidCtrlXDrive,
        TrcPIDCtrl *pidCtrlYDrive,
        TrcPIDCtrl *pidCtrlTurn,
        PIDInput   *pidInput
        ): m_drive(drive)
         , m_pidCtrlXDrive(pidCtrlXDrive)
         , m_pidCtrlYDrive(pidCtrlYDrive)
         , m_pidCtrlTurn(pidCtrlTurn)
         , m_pidInput(pidInput)
         , m_pidDriveFlags(0)
         , m_notifyEvent(NULL)
         , m_expiredTime(0)
         , m_xPower(0.0)
         , m_yPower(0.0)
    {}   //TrcPIDDrive

    /**
     * Destructor: Destroy an instance of the TrcPIDDrive object.
     */
    virtual
    ~TrcPIDDrive(
        void
        )
    {
        Stop();
    }   //~TrcPIDDrive

    /**
     * This function sets PID drive target with the given drive distance and
     * turn angle setpoints.
     *
     * @param distXSetPoint Specifies the target distance relative to current
     *        distance (only use if drive train is mecanum).
     * @param distYSetPoint Specifies the target distance relative to current
     *        distance.
     * @param angleSetPoint Specifies the target angle relative to current
     *        angle.
     * @param fHoldTarget Optionally specifies if PIDDrive will maintain target
     *        or will stop when target is reached.
     * @param notifyEvent Specifies the event to notifying for completion.
     * @param timeout Specifies the timeout in msec. No timeout if zero.
     */
    void
    SetTarget(
        float  distXSetPoint,
        float  distYSetPoint,
        float  angleSetPoint,
        bool   fHoldTarget = false,
        Event *notifyEvent = NULL,
        uint32_t timeout = 0
        )
    {
        if (m_pidCtrlXDrive != NULL)
        {
            m_pidCtrlXDrive->SetTarget(distXSetPoint,
                                       m_pidInput->GetInput(m_pidCtrlXDrive));

        }
        if (m_pidCtrlYDrive != NULL)
        {
            m_pidCtrlYDrive->SetTarget(distYSetPoint,
                                       m_pidInput->GetInput(m_pidCtrlYDrive));
        }
        if (m_pidCtrlTurn != NULL)
        {
            m_pidCtrlTurn->SetTarget(angleSetPoint,
                                     m_pidInput->GetInput(m_pidCtrlTurn));
        }
        m_notifyEvent = notifyEvent;
        m_expiredTime = (timeout != 0)? GetMsecTime() + timeout: 0;

        m_pidDriveFlags = PIDDRIVEF_PIDDRIVE_ON;
        if (fHoldTarget)
        {
            m_pidDriveFlags |= PIDDRIVEF_HOLD_TARGET;
        }

        if ((distXSetPoint == 0.0) && (distYSetPoint == 0.0) &&
            (angleSetPoint != 0.0))
        {
            m_pidDriveFlags |= PIDDRIVEF_TURN_ONLY;
        }

        return;
    }   //SetTarget

    /**
     * This function sets PID drive angle target with specifies X and Y drive
     * powers. In other words, it allows manual drive with automatic PID
     * controlled heading. This only make sense for Mecanum drive train. If it
     * is not mecanum, this function does nothing.
     *
     * @param xPower Specifies the drive power of the X direction.
     * @param yPower Specifies the drive power of the Y direction.
     * @param angleSetPoint Specifies the target angle relative to current
     *        angle.
     */
    void
    SetAngleTarget(
        float xPower,
        float yPower,
        float angleSetPoint
        )
    {
        if (m_pidCtrlXDrive != NULL)
        {
            m_xPower = xPower;
            m_yPower = yPower;
            if (m_pidCtrlTurn != NULL)
            {
                m_pidCtrlTurn->SetTarget(angleSetPoint,
                                         m_pidInput->GetInput(m_pidCtrlTurn));
            }
            m_pidDriveFlags = PIDDRIVEF_PIDDRIVE_ON |
                              PIDDRIVEF_MANUAL_DRIVE;
        }

        return;
    }   //SetAngleTarget

    /**
     * This function is called to abort a PID controlled drive in progress.
     * This makes PID controlled drive interruptible by external condition.
     */
    void
    Abort(
        void
        )
    {
        if (m_pidDriveFlags & PIDDRIVEF_PIDDRIVE_ON)
        {
            Stop();
            m_pidDriveFlags |= PIDDRIVEF_ABORTED;
            if (m_notifyEvent != NULL)
            {
                m_notifyEvent->SetEvent();
            }
        }

        return;
    }   //Abort

    /**
     * This function is called by the TaskMgr to update the PIDDrive state
     * and check for completion.
     *
     * @param mode Specifies the CoopTask callback types.
     */
    void
    TaskPostPeriodic(
        uint32_t mode
        )
    {
        if (m_pidDriveFlags & PIDDRIVEF_PIDDRIVE_ON)
        {
            float xPower = ((m_pidDriveFlags & PIDDRIVEF_TURN_ONLY) ||
                            (m_pidCtrlXDrive == NULL))?
                                0.0:
                                m_pidCtrlXDrive->CalcPIDOutput(
                                    m_pidInput->GetInput(m_pidCtrlXDrive));
            float yPower = ((m_pidDriveFlags & PIDDRIVEF_TURN_ONLY) ||
                            (m_pidCtrlYDrive == NULL))?
                                0.0:
                                m_pidCtrlYDrive->CalcPIDOutput(
                                    m_pidInput->GetInput(m_pidCtrlYDrive));
            float turnPower = (m_pidCtrlTurn == NULL)?
                                0.0:
                                m_pidCtrlTurn->CalcPIDOutput(
                                    m_pidInput->GetInput(m_pidCtrlTurn));
            bool expired = (m_expiredTime != 0) &&
                           (GetMsecTime() >= m_expiredTime);
            bool xOnTarget = (m_pidCtrlXDrive == NULL) ||
                             m_pidCtrlXDrive->OnTarget();
            bool yOnTarget = (m_pidCtrlYDrive == NULL) ||
                             m_pidCtrlYDrive->OnTarget();
            bool turnOnTarget = (m_pidCtrlTurn == NULL) ||
                                m_pidCtrlTurn->OnTarget();

            if (m_pidDriveFlags & PIDDRIVEF_MANUAL_DRIVE)
            {
                m_drive->MecanumDrive_Polar(MAGNITUDE(m_xPower, m_yPower),
                                            DIR_DEGREES(m_xPower, m_yPower),
                                            turnPower);
            }
            else if (expired ||
                     (turnOnTarget &&
                      ((m_pidDriveFlags & PIDDRIVEF_TURN_ONLY) ||
                       (xOnTarget && yOnTarget))))
            {
                if (!(m_pidDriveFlags & PIDDRIVEF_HOLD_TARGET))
                {
                    Stop();
                    if (m_notifyEvent != NULL)
                    {
                        m_notifyEvent->SetEvent();
                    }
                }
                else if (m_pidCtrlXDrive != NULL)
                {
                    m_drive->MecanumDrive_Polar(0.0, 0.0, 0.0);
                }
                else
                {
                    m_drive->Drive(0.0, 0.0);
                }
            }
            else if (m_pidCtrlXDrive != NULL)
            {
                m_drive->MecanumDrive_Polar(MAGNITUDE(xPower, yPower),
                                            DIR_DEGREES(xPower, yPower),
                                            turnPower);
            }
            else
            {
                m_drive->ArcadeDrive(yPower, turnPower);
            }
        }

        return;
    }   //TaskPostPeriodic

};  //class TrcPIDDrive

#endif  //ifndef _TRCPIDDRIVE_H
