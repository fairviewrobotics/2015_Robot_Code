/**
 *  This function is called before autonomous period starts.
 */
void TrcRobot::AutonomousStart(void)
{
    UINT32 startState;
    
    m_driveBase.ResetPosition();
    m_autoStrategy = m_dsSwitches.GetBCDDigit(DSCHN_BCD_LOW_DIGIT);
    m_autoDistance = m_dsSwitches.GetBCDDigit(DSCHN_BCD_HIGH_DIGIT);

    switch (m_autoStrategy)
    {
        case AUTO_STRATEGY_MOVE_FWD:
            startState = SMSTATE_MOVE_FWD;
            break;

        case AUTO_STRATEGY_LOW_GOAL:
            startState = SMSTATE_LOW_GOAL;
            break;
            
        case AUTO_STRATEGY_LEFT_LOW_GOAL:
        case AUTO_STRATEGY_RIGHT_LOW_GOAL:
            startState = SMSTATE_LOW_GOAL_VISION;
            break;

        case AUTO_STRATEGY_TWO_LOW_GOALS:
            startState = SMSTATE_TWO_GOALS;
            break;
        default:
            TErr(("Invalid autonomous strategy %d", m_autoStrategy));
            m_autoStrategy = AUTO_STRATEGY_INVALID;
    }

    if (m_autoStrategy != AUTO_STRATEGY_INVALID)
    {
        m_autoSM.Start(startState);
    }
}   //AutonomousStart

/**
 *  This function is called after autonomous period ends.
 */
void TrcRobot::AutonomousStop(void)
{
    if (m_autoSM.IsEnabled())
    {
        m_autoSM.Stop();
    }
}   //AutonomousStop

/**
 *  This function is called periodically during the autonomous period.
 */
void TrcRobot::AutonomousPeriodic(void)
{
    float lfCurrent = m_leftFrontMotor.GetOutputCurrent();
    float lrCurrent = m_leftRearMotor.GetOutputCurrent();
    float rfCurrent = m_rightFrontMotor.GetOutputCurrent();
    float rrCurrent = m_rightRearMotor.GetOutputCurrent();
    bool lfStalled = (m_lfCurrent != 0.0) && (lfCurrent - m_lfCurrent >= 10.0);
    bool lrStalled = (m_lrCurrent != 0.0) && (lrCurrent - m_lrCurrent >= 10.0);
    bool rfStalled = (m_rfCurrent != 0.0) && (rfCurrent - m_rfCurrent >= 10.0);
    bool rrStalled = (m_rrCurrent != 0.0) && (rrCurrent - m_rrCurrent >= 10.0);

#if 0
    printf("lf=%5.2f, rf=%5.2f, lr=%5.2f, rr=%5.2f\n",
           lfCurrent, rfCurrent, lrCurrent, rrCurrent);
    printf("lfStalled=%d, rfStalled=%d, lrStalled=%d, rrStalled=%d\n",
           lfStalled, rfStalled, lrStalled, rrStalled);
#endif

    m_lfCurrent = lfCurrent;
    m_lrCurrent = lrCurrent;
    m_rfCurrent = rfCurrent;
    m_rrCurrent = rrCurrent;

    if (lfStalled || lrStalled || rfStalled || rrStalled)
    {
        m_driveBase.AbortPidDrive();
    }

    UINT32 currState = m_autoSM.GetCurrentState();
    bool fReady = m_autoSM.IsReady();

    LCDPrintf((LCD_LINE2, "AutoState=%d %s",
               currState - SMSTATE_STARTED, fReady? "[Ready]": ""));
    //
    // Process the state machine if it is ready.
    //
    if (fReady)
    {
        switch (currState)
        {
            //
            // Strategy: Move Forward.
            //
            case SMSTATE_MOVE_FWD:
                //
                // Move forward into our zone and stop.
                //
                if (m_autoDistance == 0)
                {
                    m_autoDistance = AUTO_DEF_DISTANCE;
                }
                m_xPidCtrl.SetOutputRange(0.0, 0.0);
                m_yPidCtrl.SetOutputRange(-0.5, 0.5);
                m_turnPidCtrl.SetOutputRange(-0.5, 0.5);
                m_driveBase.DriveSetTarget(0.0,
                                           -m_autoDistance*12.0,
                                           0.0,
                                           false,
                                           &m_autoDriveEvent);
                m_autoSM.WaitForSingleEvent(&m_autoDriveEvent, currState + 1);
                break;

            case SMSTATE_MOVE_FWD + 1:
                //
                // Done.
                //
                m_autoSM.SetCurrentState(SMSTATE_AUTO_DONE);
                break;

            //
            // Strategy: Score Low Goal either left or right without vision.
            //
            case SMSTATE_LOW_GOAL:
                //
                // Drop the ball into motivator and wait for half a second.
                //
                m_motivator.Open();
                m_elToro.Extend();
                m_autoTimer.SetTimer(0.5, &m_autoTimerEvent);
                m_autoSM.WaitForSingleEvent(&m_autoTimerEvent, currState + 1);
                break;

            case SMSTATE_LOW_GOAL + 1:
                //
                // Retract El Toro, clamp down motivator and move towards
                // goal (15ft + adjustment in inches)..
                //
                m_elToro.Retract();
                m_motivator.Shoot(true);
                m_xPidCtrl.SetOutputRange(0.0, 0.0);
                m_yPidCtrl.SetOutputRange(-0.5, 0.5);
                m_turnPidCtrl.SetOutputRange(-0.5, 0.5);
                m_driveBase.DriveSetTarget(0.0,
                                           -(AUTO_GOAL_DISTANCE*12.0 +
                                             m_autoDistance),
                                           0.0,
                                           false,
                                           &m_autoDriveEvent);
                m_autoSM.WaitForSingleEvent(&m_autoDriveEvent, currState + 1, 2000);
                break;

            case SMSTATE_LOW_GOAL + 2:
                //
                // Shoot the ball, done.
                //
                m_motivator.Shoot(false);
                m_autoSM.SetCurrentState(SMSTATE_AUTO_DONE);
                break;

#ifndef _DISABLE_VISION
            //
            // Stategy: Score Low Goal either left or right with vision.
            //
            case SMSTATE_LOW_GOAL_VISION:
                //
                // Drop the ball into motivator and wait for half a second.
                //
                if (m_autoDistance == 0)
                {
                    m_autoDistance = AUTO_DEF_DISTANCE;
                }
                m_motivator.Open();
                m_elToro.Extend();
                m_autoTimer.SetTimer(0.5, &m_autoTimerEvent);
                m_autoSM.WaitForSingleEvent(&m_autoTimerEvent, currState + 1);
                break;

            case SMSTATE_LOW_GOAL_VISION + 1:
                //
                // Retract El Toro, clamp down motivator and move closer to
                // the goal before engaging vision targeting.
                //
                m_elToro.Retract();
                m_motivator.Shoot(true);
                m_xPidCtrl.SetOutputRange(0.0, 0.0);
                m_yPidCtrl.SetOutputRange(-0.5, 0.5);
                m_turnPidCtrl.SetOutputRange(-0.5, 0.5);
                m_driveBase.DriveSetTarget(0.0,
                                           -m_autoDistance*12.0,
                                           0.0,
                                           false,
                                           &m_autoDriveEvent,
                                           1000);
                m_autoTimeout = GetMsecTime() + 2000;
                m_autoSM.WaitForSingleEvent(&m_autoDriveEvent, currState + 1);
                break;

            case SMSTATE_LOW_GOAL_VISION + 2:
                //
                // Acquire target.
                // Prepare motivator to shoot.
                // Move toward target.
                //
                if (GetMsecTime() >= m_autoTimeout)
                {
                    m_driveBase.DriveSetTarget(
                            0.0,
                            -(AUTO_GOAL_DISTANCE - m_autoDistance)*12.0,
                            0.0,
                            false,
                            &m_autoDriveEvent);
                    m_autoSM.WaitForSingleEvent(&m_autoDriveEvent,
                                                currState + 1);
                }
                else if (m_visionTarget.GetTargetInfo(m_targetReport))
                {
                    m_driveBase.DriveSetTarget(
//                            (m_autoStrategy == AUTO_STRATEGY_LEFT_LOW_GOAL)?
//                                -m_targetReport.offset + ???:
//                                -m_targetReport.offset - ???,
                            0.0,
                            -(m_targetReport.distance - 65.0), //???
                            0.0,
                            false,
                            &m_autoDriveEvent);
                    m_autoSM.WaitForSingleEvent(&m_autoDriveEvent,
                                                currState + 1);
                }
                break;

            case SMSTATE_LOW_GOAL_VISION + 3:
                //
                // Set hot target timeout.
                //
                m_autoTimeout = GetMsecTime() + 4000;  //???
                m_autoSM.SetCurrentState(currState + 1);
                break;

            case SMSTATE_LOW_GOAL_VISION + 4:
                //
                // Wait for hot target or until timeout.
                // Shoot into low goal.
                //
                if ((GetMsecTime() > m_autoTimeout) ||
                    (m_visionTarget.GetTargetInfo(m_targetReport) &&
                     m_targetReport.Hot))
                {
                    m_motivator.Shoot(false);
                    m_autoSM.SetCurrentState(SMSTATE_AUTO_DONE);
                }
                break;
#endif

            case SMSTATE_TWO_GOALS:
                //
                // Drop the ball into motivator and wait for half a second.
                //
                m_motivator.Open();
                m_elToro.Extend();
                m_autoTimer.SetTimer(0.5, &m_autoTimerEvent);
                m_autoSM.WaitForSingleEvent(&m_autoTimerEvent, currState + 1);
                break;

            case SMSTATE_TWO_GOALS + 1:
                //
                // Clamp down on the ball and spin the El Toro to pick up
                // the second ball.
                //
                m_motivator.Close();
                m_elToro.SetSpeed(ELTORO_POWER);
                m_autoTimer.SetTimer(1.0, &m_autoTimerEvent);
                m_autoSM.WaitForSingleEvent(&m_autoTimerEvent, currState + 1);
                break;

            case SMSTATE_TWO_GOALS + 2:
                //
                // Stop spinning the El Toro and start moving forward.
                //
                m_elToro.SetSpeed(0.0);
                m_xPidCtrl.SetOutputRange(0.0, 0.0);
                m_yPidCtrl.SetOutputRange(-0.5, 0.5);
                m_turnPidCtrl.SetOutputRange(-0.5, 0.5);
                m_driveBase.DriveSetTarget(0.0,
                                           -(AUTO_GOAL_DISTANCE*12.0 +
                                             m_autoDistance),
                                           0.0,
                                           false,
                                           &m_autoDriveEvent);
                m_autoSM.WaitForSingleEvent(&m_autoDriveEvent, currState + 1);
                break;

            case SMSTATE_TWO_GOALS + 3:
                m_motivator.SetSpeed(1.0);
                m_autoTimer.SetTimer(2.0, &m_autoTimerEvent);
                m_autoSM.WaitForSingleEvent(&m_autoTimerEvent, currState + 1);
                break;

            case SMSTATE_TWO_GOALS + 4:
                m_elToro.SetSpeed(ELTORO_POWER);
                m_elToro.Retract();
                m_autoTimer.SetTimer(2.0, &m_autoTimerEvent);
                m_autoSM.WaitForSingleEvent(&m_autoTimerEvent, currState + 1);
                break;

            case SMSTATE_TWO_GOALS + 5:
                m_motivator.SetSpeed(0.0);
                m_elToro.SetSpeed(0.0);
                m_autoSM.SetCurrentState(SMSTATE_AUTO_DONE);
                break;
            
            case SMSTATE_AUTO_DONE:
            default:
                m_autoSM.Stop();
                m_xPidCtrl.SetOutputRange(MOTOR_RANGE_MIN, MOTOR_RANGE_MAX);
                m_yPidCtrl.SetOutputRange(MOTOR_RANGE_MIN, MOTOR_RANGE_MAX);
                m_turnPidCtrl.SetOutputRange(MOTOR_RANGE_MIN, MOTOR_RANGE_MAX);
                break;
        }
    }
}   //AutonomousPeriodic

