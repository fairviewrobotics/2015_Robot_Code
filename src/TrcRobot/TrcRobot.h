/**
 *  This module defines and implements the main robot object. The main robot
 *  object consists of the DriveBase and various subsystems. It also contains
 *  callback methods for the competition modes: Disabled, Autonomous and
 *  TeleOp.
 */
class TrcRobot
    : public CoopMTRobot
    , public ButtonNotify
    , public EIODinNotify
    , public MotorPosition
{
private:
    //
    // Miscellaneous
    //
    DashboardDataFormat m_dashboardDataFormat;
    Compressor          m_compressor;
    //
    // Input subsystem.
    //
    TrcJoystick         m_leftDriveStick;
    TrcJoystick         m_rightDriveStick;
    TrcJoystick         m_operatorStick;
    //
    // Drive subsystem.
    //
    Gyro                m_gyro;
    CanJag              m_leftFrontMotor;
    CanJag              m_leftRearMotor;
    CanJag              m_rightFrontMotor;
    CanJag              m_rightRearMotor;
    TrcPIDCtrl          m_xPidCtrl;
    TrcPIDCtrl          m_yPidCtrl;
    TrcPIDCtrl          m_turnPidCtrl;
    DriveBase           m_driveBase;
    //
    // ElToro subsystem.
    //
    ElToro              m_elToro;
    //
    // Motivator subsystem.
    //
    Motivator           m_motivator;
    bool                m_motivatorManualPower;
    bool                m_motivatorOpened;
    bool                m_ballStopperRetracted;
    //
    // FunnelDoor subsystem.
    //
    FunnelDoor          m_funnelDoor;
    bool                m_funnelDoorManualControl;
    bool                m_funnelDoorOpened;
#ifndef _DISABLE_VISION
    //
    // VisionTarget subsystem.
    //
    VisionTarget        m_visionTarget;
    TargetReport        m_targetReport;
#endif
    //
    // RGBLights subsystem.
    //
    SolLight            m_rgbLights;
    UINT                m_colorIdx;
    bool                m_flashing;
    //
    // Autonomous mode.
    //
    DSEnhDin            m_dsSwitches;
    UINT8               m_autoStrategy;
    UINT8               m_autoDistance;
    StateMachine        m_autoSM;
    Event               m_autoDriveEvent;
    TrcTimer            m_autoTimer;
    Event               m_autoTimerEvent;
    UINT32              m_autoTimeout;
    float               m_lfCurrent;
    float               m_lrCurrent;
    float               m_rfCurrent;
    float               m_rrCurrent;
    //
    // Test mode.
    //
    UINT8               m_testOp;
    UINT8               m_testParam;
    StateMachine        m_testSM;
    TrcTimer            m_testTimer;
    Event               m_testEvent;

public:
    /**
     *  This is a callback function for the DriveBase object to obtain position
     *  information for each motor.
     */
    float
    GetMotorPosition(
        SpeedController *motor
        )
    {
        return ((CANJaguar*)motor)->GetPosition();
    }   //GetMotorPosition

    /**
     *  Constructor.
     */
    TrcRobot(void)
        : m_dashboardDataFormat()
        , m_compressor(DIN_COMPRESSOR_PRESSURE_SWITCH, RELAY_COMPRESSOR_POWER)
        , m_leftDriveStick(JOYSTICK_LEFT_DRIVE, this)
        , m_rightDriveStick(JOYSTICK_RIGHT_DRIVE, this)
        , m_operatorStick(JOYSTICK_OPERATOR, this)
        , m_gyro(AIN_GYRO)
        , m_leftFrontMotor(CANID_LEFTFRONT_JAG)
        , m_leftRearMotor(CANID_LEFTREAR_JAG)
        , m_rightFrontMotor(CANID_RIGHTFRONT_JAG)
        , m_rightRearMotor(CANID_RIGHTREAR_JAG)
        , m_xPidCtrl("XDrive",
                     XDRIVE_KP, XDRIVE_KI, XDRIVE_KD, XDRIVE_KF,
                     XDRIVE_TOLERANCE, XDRIVE_SETTLING)
        , m_yPidCtrl("YDrive",
                     YDRIVE_KP, YDRIVE_KI, YDRIVE_KD, YDRIVE_KF,
                     YDRIVE_TOLERANCE, YDRIVE_SETTLING)
        , m_turnPidCtrl("Turn",
                        TURN_KP, TURN_KI, TURN_KD, TURN_KF,
                        TURN_TOLERANCE, TURN_SETTLING)
        , m_driveBase(&m_gyro,
                      &m_leftFrontMotor, &m_leftRearMotor,
                      &m_rightFrontMotor, &m_rightRearMotor,
//                      NULL, &m_yPidCtrl, &m_turnPidCtrl,
                      &m_xPidCtrl, &m_yPidCtrl, &m_turnPidCtrl,
                      this)
        , m_elToro()
        , m_motivator()
        , m_motivatorManualPower(false)
        , m_motivatorOpened(false)
        , m_ballStopperRetracted(false)
        , m_funnelDoor()
        , m_funnelDoorManualControl(false)
        , m_funnelDoorOpened(false)
#ifndef _DISABLE_VISION
        , m_visionTarget(&m_dashboardDataFormat)
#endif
        , m_rgbLights(SOL_RED_LED, SOL_GREEN_LED, SOL_BLUE_LED, 2)
        , m_colorIdx(0)
        , m_flashing(false)
        , m_dsSwitches()
        , m_autoStrategy(AUTO_STRATEGY_INVALID)
        , m_autoDistance(0)
        , m_autoSM()
        , m_autoDriveEvent()
        , m_autoTimer()
        , m_autoTimerEvent()
        , m_autoTimeout(0)
        , m_lfCurrent(0.0)
        , m_lrCurrent(0.0)
        , m_rfCurrent(0.0)
        , m_rrCurrent(0.0)
        , m_testOp(TESTOP_INVALID)
        , m_testParam(0)
        , m_testSM()
        , m_testTimer()
        , m_testEvent()
    {
        TraceInit(TRACE_MODULES, TRACE_LEVEL, MSG_LEVEL);
        //
        // Compressor takes a long time to charge up the air tanks,
        // so start this at the earliest possible time.
        //
        m_compressor.Start();

        //
        // Initialize motor controllers.
        //
        m_leftFrontMotor.SetSpeedReference(CANJaguar::kSpeedRef_QuadEncoder);
        m_leftRearMotor.SetSpeedReference(CANJaguar::kSpeedRef_QuadEncoder);
        m_rightFrontMotor.SetSpeedReference(CANJaguar::kSpeedRef_QuadEncoder);
        m_rightRearMotor.SetSpeedReference(CANJaguar::kSpeedRef_QuadEncoder);
    
        m_leftFrontMotor.SetPositionReference(CANJaguar::kPosRef_QuadEncoder);
        m_leftRearMotor.SetPositionReference(CANJaguar::kPosRef_QuadEncoder);
        m_rightFrontMotor.SetPositionReference(CANJaguar::kPosRef_QuadEncoder);
        m_rightRearMotor.SetPositionReference(CANJaguar::kPosRef_QuadEncoder);

        m_leftFrontMotor.ConfigEncoderCodesPerRev(DRIVE_ENCODER_PPR);
        m_leftRearMotor.ConfigEncoderCodesPerRev(DRIVE_ENCODER_PPR);
        m_rightFrontMotor.ConfigEncoderCodesPerRev(DRIVE_ENCODER_PPR);
        m_rightRearMotor.ConfigEncoderCodesPerRev(DRIVE_ENCODER_PPR);

        m_leftFrontMotor.SetSafetyEnabled(false);
        m_leftRearMotor.SetSafetyEnabled(false);
        m_rightFrontMotor.SetSafetyEnabled(false);
        m_rightRearMotor.SetSafetyEnabled(false);

        m_leftFrontMotor.SetExpiration(0.5);
        m_leftRearMotor.SetExpiration(0.5);
        m_rightFrontMotor.SetExpiration(0.5);
        m_rightRearMotor.SetExpiration(0.5);

        m_leftFrontMotor.EnableControl();
        m_leftRearMotor.EnableControl();
        m_rightFrontMotor.EnableControl();
        m_rightRearMotor.EnableControl();

        m_driveBase.SetInvertedMotor(RobotDrive::kFrontLeftMotor,
                                     MOTOR_LEFT_FRONT_REVERSE);
        m_driveBase.SetInvertedMotor(RobotDrive::kRearLeftMotor,
                                     MOTOR_LEFT_REAR_REVERSE);
        m_driveBase.SetInvertedMotor(RobotDrive::kFrontRightMotor,
                                     MOTOR_RIGHT_FRONT_REVERSE);
        m_driveBase.SetInvertedMotor(RobotDrive::kRearRightMotor,
                                     MOTOR_RIGHT_REAR_REVERSE);
        m_driveBase.SetEncoderPolarities(MOTOR_LEFT_FRONT_REVERSE,
                                         MOTOR_LEFT_REAR_REVERSE,
                                         MOTOR_RIGHT_FRONT_REVERSE,
                                         MOTOR_RIGHT_REAR_REVERSE);
        m_driveBase.SetEncoderScale(DRIVE_XDISTANCE_PER_REV,
                                    DRIVE_YDISTANCE_PER_REV,
                                    TURN_DEGREE_PER_REV);
        
        SetPeriod(0.1);
    }   //TrcRobot

    /**
     *  Destructor.
     */
    ~TrcRobot(void)
    {
        m_compressor.Stop();
    }   //~TrcRobot

    //
    // The following functions are in disabled.h
    //
    void DisabledStart(void);
    void DisabledStop(void);
    void DisabledPeriodic(void);

    //
    // The following functions are in auto.h
    //
    void AutonomousStart(void);
    void AutonomousStop(void);
    void AutonomousPeriodic(void);

    //
    // The following functions are in teleop.h
    //
    void TeleOpStart(void);
    void TeleOpStop(void);
    void TeleOpPeriodic(void);
    void NotifyButton(UINT32 port, UINT16 maskButton, bool fPressed);
    void NotifyEIODin(UINT32 channel, UINT32 state);

    //
    // The following functions are in test.h
    //
    void TestStart(void);
    void TestStop(void);
    void TestPeriodic(void);

};  //class TrcRobot

