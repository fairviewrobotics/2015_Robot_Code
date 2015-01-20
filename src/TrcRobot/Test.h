/**
 *  This function is called before test period starts.
 */
void TrcRobot::TestStart(void)
{
    UINT32 startState;

    m_driveBase.ResetPosition();
    m_testOp = m_dsSwitches.GetBCDDigit(DSCHN_BCD_LOW_DIGIT);
    m_testParam = m_dsSwitches.GetBCDDigit(DSCHN_BCD_HIGH_DIGIT);
    switch (m_testOp)
    {
        case TESTOP_DRIVE_ENCODERS:
            startState = SMSTATE_TEST_DRIVE_ENCODERS;
            break;

        case TESTOP_TIMED_DRIVE_X:
            startState = SMSTATE_TEST_TIMED_DRIVE_X;
            break;

        case TESTOP_TIMED_DRIVE_Y:
            startState = SMSTATE_TEST_TIMED_DRIVE_Y;
            break;

        case TESTOP_DRIVE_X:
            startState = SMSTATE_TEST_DRIVE_X;
            break;

        case TESTOP_DRIVE_Y:
            startState = SMSTATE_TEST_DRIVE_Y;
            break;

        case TESTOP_TURN:
            startState = SMSTATE_TEST_TURN;
            break;
            
        default:
            TErr(("Invalid test operation %d", m_testOp));
            m_testOp = TESTOP_INVALID;
    }

    if (m_testOp != TESTOP_INVALID)
    {
        m_testSM.Start(startState);
    }
}   //TestStart

/**
 *  This function is called after test period ends.
 */
void TrcRobot::TestStop(void)
{
    if (m_testSM.IsEnabled())
    {
        m_driveBase.Stop();
        m_testSM.Stop();
    }
}   //TestStop

/**
 *  This function is called periodically during the test period.
 */
void TrcRobot::TestPeriodic(void)
{
    UINT32 currState = m_testSM.GetCurrentState();
    bool fReady = m_testSM.IsReady();
    float xPos, yPos, rotPos, heading;

    LCDPrintf((LCD_LINE2, "TestState=%d %s",
               currState - SMSTATE_STARTED, fReady? "[Ready]": ""));

    m_driveBase.GetPosition(&xPos, &yPos, &rotPos, &heading);
    LCDPrintf((LCD_LINE3, "lf=%5.1f,rf=%5.1f",
               m_leftFrontMotor.GetPosition(),
               m_rightFrontMotor.GetPosition()));
    LCDPrintf((LCD_LINE4, "lr=%5.1f,rr=%5.1f",
               m_leftRearMotor.GetPosition(),
               m_rightRearMotor.GetPosition()));
    LCDPrintf((LCD_LINE5, "x=%5.1f,y=%5.1f", xPos, yPos));
    LCDPrintf((LCD_LINE6, "r=%5.1f,h=%5.1f", rotPos, heading));
    //
    // Process the state machine if it is ready.
    //
    if (fReady)
    {
        switch (currState)
        {
            case SMSTATE_TEST_DRIVE_ENCODERS:
                //
                // Never exits until disabled.
                //
                break;

            case SMSTATE_TEST_TIMED_DRIVE_X:
                m_driveBase.MecanumDrive_Cartesian(0.2, 0.0, 0.0);
                m_testTimer.SetTimer(m_testParam, &m_testEvent);
                m_testSM.WaitForSingleEvent(&m_testEvent, currState + 1);
                break;

            case SMSTATE_TEST_TIMED_DRIVE_X + 1:
                m_driveBase.MecanumDrive_Cartesian(0.0, 0.0, 0.0);
                m_testSM.SetCurrentState(SMSTATE_TEST_DONE);
                break;

            case SMSTATE_TEST_TIMED_DRIVE_Y:
                m_driveBase.MecanumDrive_Cartesian(0.0, 0.2, 0.0);
                m_testTimer.SetTimer(m_testParam, &m_testEvent);
                m_testSM.WaitForSingleEvent(&m_testEvent, currState + 1);
                break;

            case SMSTATE_TEST_TIMED_DRIVE_Y + 1:
                m_driveBase.MecanumDrive_Cartesian(0.0, 0.0, 0.0);
                m_testSM.SetCurrentState(SMSTATE_TEST_DONE);
                break;

            case SMSTATE_TEST_DRIVE_X:
                m_xPidCtrl.SetOutputRange(-0.2, 0.2);
                m_driveBase.DriveSetTarget(TEST_DRIVE_DISTANCE,
                                           0.0,
                                           0.0,
                                           false,
                                           &m_testEvent);
                m_testSM.WaitForSingleEvent(&m_testEvent, currState + 1);
                break;

            case SMSTATE_TEST_DRIVE_X + 1:
                m_xPidCtrl.SetOutputRange(MOTOR_RANGE_MIN, MOTOR_RANGE_MAX);
                m_testSM.SetCurrentState(SMSTATE_TEST_DONE);
                break;

            case SMSTATE_TEST_DRIVE_Y:
                m_yPidCtrl.SetOutputRange(-0.2, 0.2);
                m_driveBase.DriveSetTarget(0.0,
                                           TEST_DRIVE_DISTANCE,
                                           0.0,
                                           false,
                                           &m_testEvent);
                m_testSM.WaitForSingleEvent(&m_testEvent, currState + 1);
                break;

            case SMSTATE_TEST_DRIVE_Y + 1:
                m_yPidCtrl.SetOutputRange(MOTOR_RANGE_MIN, MOTOR_RANGE_MAX);
                m_testSM.SetCurrentState(SMSTATE_TEST_DONE);
                break;

            case SMSTATE_TEST_TURN:
                m_turnPidCtrl.SetOutputRange(-0.3, 0.3);
                m_driveBase.DriveSetTarget(0.0, 0.0, 360.0, false, &m_testEvent);
                m_testSM.WaitForSingleEvent(&m_testEvent, currState + 1);
                break;
             
            case SMSTATE_TEST_TURN + 1:
                m_turnPidCtrl.SetOutputRange(MOTOR_RANGE_MIN, MOTOR_RANGE_MAX);
                m_testSM.SetCurrentState(SMSTATE_TEST_DONE);
                break;

            case SMSTATE_TEST_DONE:
            default:
                m_driveBase.Stop();
                m_testSM.Stop();
                break;
        }
    }
}   //TestPeriodic
