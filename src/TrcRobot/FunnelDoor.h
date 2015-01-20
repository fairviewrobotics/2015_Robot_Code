/**
 * This module defines and implements the FunnelDoor subsytem. The FunelDoor
 * subsystem consists of one motor swinging the funnel door open and close
 * within a certain angle defined by two limit switches.
 */
class FunnelDoor
    : public CoopTask
{
private:
    #define FUNNELDOOR_STATE_OPENING    SMSTATE_STARTED
    #define FUNNELDOOR_STATE_CLOSING    (SMSTATE_STARTED + 1)
    #define FUNNELDOOR_STATE_OFF        (SMSTATE_STARTED + 2)

    #define FUNNELDOOR_TIMEOUT          3000    //msevc

    CanJag          m_motor;
    float           m_motorSpeed;
    StateMachine    m_funnelDoorSM;
    UINT32          m_timeout;

public:
    void SetSpeed(float speed)
    {
        m_motorSpeed = speed;
    }   //SetSpeed

    void Stop(void)
    {
        if (m_funnelDoorSM.IsEnabled())
        {
            //
            // There is a pending opening/closing operation,
            // canel it.
            //
            m_funnelDoorSM.Stop();
        }
        m_motorSpeed = 0.0;
    }   //Stop
    
    void Open(void)
    {
        if (m_funnelDoorSM.IsEnabled())
        {
            //
            // There is a pending opening/closing operation,
            // canel it.
            //
            m_funnelDoorSM.Stop();
        }

        m_timeout = GetMsecTime() + FUNNELDOOR_TIMEOUT;
        m_funnelDoorSM.Start(FUNNELDOOR_STATE_OPENING);
    }   //Open
    
    void Close(void)
    {
        if (m_funnelDoorSM.IsEnabled())
        {
            //
            // There is a pending opening/closing operation,
            // canel it.
            //
            m_funnelDoorSM.Stop();
        }

        m_timeout = GetMsecTime() + FUNNELDOOR_TIMEOUT;
        m_funnelDoorSM.Start(FUNNELDOOR_STATE_CLOSING);
    }   //Close
    
    /**
     *  Constructor.
     */
    FunnelDoor(void)
        : m_motor(CANID_FUNNELDOOR_JAG)
        , m_motorSpeed(0.0)
        , m_funnelDoorSM()
        , m_timeout(0)
    {
        RegisterTask("FunnelDoor", TASK_STOP_MODE | TASK_POST_PERIODIC);
    }   //FunnelDoor
    
    /**
     *  Destructor.
     */
    virtual
    ~FunnelDoor(void)
    {
        Stop();
        UnregisterTask();
    }   //~FunnelDoor

    void TaskStopMode(UINT32 mode)
    {
        if (mode != MODE_DISABLED)
        {
            Stop();
        }
    }   //TaskStopMode

    void TaskPostPeriodic(UINT32 mode)
    {
        if (mode != MODE_DISABLED)
        {
            if (m_funnelDoorSM.IsReady())
            {
                UINT32 currState = m_funnelDoorSM.GetCurrentState();

                switch (currState)
                {
                    case FUNNELDOOR_STATE_OPENING:
                        if ((GetMsecTime() < m_timeout) &&
                            m_motor.GetReverseLimitOK())
                        {
                            m_motorSpeed = -FUNNELDOOR_POWER;
                        }
                        else
                        {
                            m_funnelDoorSM.SetCurrentState(
                                FUNNELDOOR_STATE_OFF);
                        }
                        break;

                    case FUNNELDOOR_STATE_CLOSING:
                        if ((GetMsecTime() < m_timeout) &&
                            m_motor.GetForwardLimitOK())
                        {
                            m_motorSpeed = FUNNELDOOR_POWER;
                        }
                        else
                        {
                            m_funnelDoorSM.SetCurrentState(
                                FUNNELDOOR_STATE_OFF);
                        }
                        break;

                    default:
                        m_timeout = 0;
                        m_motorSpeed = 0.0;
                        m_funnelDoorSM.Stop();
                        break;
                }
            }

            m_motor.Set(m_motorSpeed);
#ifdef _DEBUG_FUNNELDOOR
            LCDPrintf((LCD_LINE5, "fdPower=%5.2f", m_motorSpeed));
            LCDPrintf((LCD_LINE6, "FSw=%d,RSw=%d",
                       m_motor.GetForwardLimitOK(),
                       m_motor.GetReverseLimitOK()));
#endif
        }
    }   //TaskPostPeriodic

};  //class FunnelDoor
