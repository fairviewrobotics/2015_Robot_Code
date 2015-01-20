/**
 * This module defines and implements the Motivator subsystem. The Motivator
 * subsystem consists of a pair of motors on Jaguar controllers and a pair
 * of pneumatic deployers.
 */
class Motivator
    : public CoopTask
{
private:
    CanJag              m_leftMotor;
    CanJag              m_rightMotor;
    TrcSol              m_deployer;
    TrcSol              m_ballStopper;
    float               m_motorSpeed;
    bool                m_inAction;
    StateMachine        m_shootSM;
    TrcTimer            m_shootTimer;
    Event               m_shootTimerEvent;

public:
    void StopShooter(void)
    {
        m_shootTimer.CancelTimer();
        m_shootTimerEvent.ClearEvent();
        m_shootSM.Stop();
    }   //StopShooter

    void SetSpeed(float speed)
    {
        m_motorSpeed = speed;
    }   //SetSpeed

    void Open(void)
    {
        m_deployer.Set(MOTIVATOR_EXTEND, MOTIVATOR_RETRACT);
    }   //Extend

    void Close(void)
    {
        m_deployer.Set(MOTIVATOR_RETRACT, MOTIVATOR_EXTEND);
    }   //Retract

    void ExtendBallStopper(void)
    {
        m_ballStopper.Set(BALLSTOPPER_EXTEND, BALLSTOPPER_RETRACT);
    }   //ExtendBallStopper

    void RetractBallStopper(void)
    {
        m_ballStopper.Set(BALLSTOPPER_RETRACT, BALLSTOPPER_EXTEND);
    }   //RetractBallStopper

    bool IsInAction(void)
    {
        return m_inAction;
    }   //IsInAction
    
    void MotorOverride(bool fInaction)
    {
        m_inAction = fInaction;
    }   //MotorOverride

    void Shoot(bool fPressed)
    {
        if (fPressed)
        {
            if (m_shootSM.IsEnabled())
            {
                //
                // Abort previous operation.
                //
                StopShooter();
            }

            Close();
            m_motorSpeed = 0.0;
            m_inAction = true;
        }
        else
        {
            m_motorSpeed = 1.0;
            RetractBallStopper();
            m_shootSM.Start();
        }
    }   //Shoot

    /**
     *  Constructor.
     */
    Motivator(void)
        : m_leftMotor(CANID_LEFTMOTIVATOR_JAG)
        , m_rightMotor(CANID_RIGHTMOTIVATOR_JAG)
        , m_deployer(SOL_MOTIVATOR_EXTEND, SOL_MOTIVATOR_RETRACT)
        , m_ballStopper(SOL_BALLSTOPPER_EXTEND, SOL_BALLSTOPPER_RETRACT)
        , m_motorSpeed(0.0)
        , m_inAction(false)
        , m_shootSM()
        , m_shootTimer()
        , m_shootTimerEvent()
    {
        m_leftMotor.SetSafetyEnabled(false);
        m_rightMotor.SetSafetyEnabled(false);

        m_leftMotor.EnableControl();
        m_rightMotor.EnableControl();
        
        Close();
        ExtendBallStopper();
        
        RegisterTask("Shooter", TASK_STOP_MODE | TASK_POST_PERIODIC);
    }   //Motivator

    /**
     *  Destructor.
     */
    virtual
    ~Motivator(void)
    {
        if (m_shootSM.IsEnabled())
        {
            //
            // Abort previous operation.
            //
            StopShooter();
        }
        RetractBallStopper();
        Close();
        m_motorSpeed = 0.0;
        m_inAction = false;
        UnregisterTask();
    }   //~Motivator
        
    void TaskStopMode(UINT32 mode)
    {
        if (mode != MODE_DISABLED)
        {
            if (m_shootSM.IsEnabled())
            {
                //
                // Abort previous operation.
                //
                StopShooter();
            }
            RetractBallStopper();
            Close();
            m_motorSpeed = 0.0;
            m_inAction = false;
        }
    }   //TaskStopMode

    void TaskPostPeriodic(UINT32 mode)
    {
        if (mode != MODE_DISABLED)
        {
            if (m_shootSM.IsReady())
            {
                UINT32 currState = m_shootSM.GetCurrentState();

                switch (currState)
                {
                    case SMSTATE_STARTED:
                        m_shootTimer.SetTimer(SHOOT_DELAY, &m_shootTimerEvent);
                        m_shootSM.WaitForSingleEvent(&m_shootTimerEvent,
                                                     currState + 1);
                        break;

                    default:
                        m_motorSpeed = 0.0;
                        ExtendBallStopper();
                        m_inAction = false;
                        m_shootSM.Stop();
                        break;
                }
            }

            m_leftMotor.Set(m_motorSpeed, JAG_SYNCGROUP_MOTIVATOR);
            m_rightMotor.Set(-m_motorSpeed, JAG_SYNCGROUP_MOTIVATOR);
            m_leftMotor.UpdateSyncGroup(JAG_SYNCGROUP_MOTIVATOR);
        }
    }   //TaskPostPeriodic

};  //class Motivator
