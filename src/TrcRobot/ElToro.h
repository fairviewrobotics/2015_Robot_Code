/**
 * This module defines and implements the El Toro subsytem. The El Toro
 * subsystem consists of a pair of motors on Talon controllers and a
 * pneumatic deployer.
 */
class ElToro
    : public CoopTask
{
private:
    Talon           m_leftMotor;
    Talon           m_rightMotor;
    TrcSol          m_deployer;
    StateMachine    m_pickupBallSM;
    TrcTimer        m_pickupBallTimer;
    Event           m_pickupBallTimerEvent;

public:
    void StopBallPickup(void)
    {
        m_pickupBallTimer.CancelTimer();
        m_pickupBallTimerEvent.ClearEvent();
        m_pickupBallSM.Stop();
    }   //StopShooter

    void SetSpeed(float speed)
    {
        m_leftMotor.Set(speed);
        m_rightMotor.Set(-speed);
    }   //SetSpeed

    void Extend(void)
    {
        m_deployer.Set(ELTORO_EXTEND, ELTORO_RETRACT);
    }   //Extend

    void Retract(void)
    {
        m_deployer.Set(ELTORO_RETRACT, ELTORO_EXTEND);
    }   //Retract

    void PickupBall(bool fPressed)
    {
        if (fPressed)
        {
            if (m_pickupBallSM.IsEnabled())
            {
                //
                // Abort previous operation.
                //
                StopBallPickup();
            }

            SetSpeed(ELTORO_POWER);
            Extend();
        }
        else
        {
            Retract();
            m_pickupBallSM.Start();
        }
    }   //PickupBall

    /**
     *  Constructor. 
     */
    ElToro(void)
        : m_leftMotor(1, PWM_LEFT_ELTORO_MOTOR)
        , m_rightMotor(1, PWM_RIGHT_ELTORO_MOTOR)
        , m_deployer(SOL_ELTORO_EXTEND, SOL_ELTORO_RETRACT)
        , m_pickupBallSM()
        , m_pickupBallTimer()
        , m_pickupBallTimerEvent()
    {
        Retract();
        RegisterTask("ElToro", TASK_STOP_MODE | TASK_POST_PERIODIC);
    }   //ElToro

    /**
     *  Destructor.
     */
    virtual
    ~ElToro(void)
    {
        if (m_pickupBallSM.IsEnabled())
        {
            //
            // Abort previous operation.
            //
            StopBallPickup();
        }
        SetSpeed(0.0);
        Retract();
        UnregisterTask();
    }   //~ElToro

    void TaskStopMode(UINT32 mode)
    {
        if (mode != MODE_DISABLED)
        {
            if (m_pickupBallSM.IsEnabled())
            {
                //
                // Abort previous operation.
                //
                StopBallPickup();
            }
            SetSpeed(0.0);
            Retract();
        }
    }   //TaskStopMode

    void TaskPostPeriodic(UINT32 mode)
    {
        if (mode != MODE_DISABLED && m_pickupBallSM.IsReady())
        {
            UINT32 currState = m_pickupBallSM.GetCurrentState();

            switch (currState)
            {
                case SMSTATE_STARTED:
                    m_pickupBallTimer.SetTimer(ELTORO_DELAY,
                                               &m_pickupBallTimerEvent);
                    m_pickupBallSM.WaitForSingleEvent(&m_pickupBallTimerEvent,
                                                      currState + 1);
                    break;

                default:
                    SetSpeed(0.0);
                    m_pickupBallSM.Stop();
                    break;
            }
        }
    }   //TaskPostPeriodic

};  //class ElToro
