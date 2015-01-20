/**
 *  This function is called before teleop period starts.
 */
void TrcRobot::TeleOpStart(void)
{
    m_dsSwitches.RegisterNotification(this, 0xff00);
    m_driveBase.ResetPosition();
}   //TeleOpStart

/**
 *  This function is called after teleop period ends.
 */
void TrcRobot::TeleOpStop(void)
{
    m_dsSwitches.UnregisterNotification(this);
}   //TeleOpStop

/**
 *  This function is called during the teleop period.
 */
void TrcRobot::TeleOpPeriodic(void)
{
#ifdef _DEBUG_TELEOP
    float xPos, yPos, rotPos, heading;

    m_driveBase.GetPosition(&xPos, &yPos, &rotPos, &heading);
    LCDPrintf((LCD_LINE3, "lf=%5.1f,rf=%5.1f",
               m_leftFrontMotor.GetPosition(),
               m_rightFrontMotor.GetPosition()));
    LCDPrintf((LCD_LINE4, "lr=%5.1f,rr=%5.1f",
               m_leftRearMotor.GetPosition(),
               m_rightRearMotor.GetPosition()));
    LCDPrintf((LCD_LINE5, "x=%5.1f,y=%5.1f", xPos, yPos));
    LCDPrintf((LCD_LINE6, "r=%5.1f,h=%5.1f", rotPos, heading));
#endif

    float speed;
    //
    // DriveBase operation.
    //
    float x = m_leftDriveStick.GetXWithDeadband(true);
    float y = m_rightDriveStick.GetYWithDeadband(true);
    float rot = m_rightDriveStick.GetZWithDeadband(true);

    m_driveBase.MecanumDrive_Cartesian(x, y, rot);

    //
    // Motivator operation.
    //
    if (!m_motivator.IsInAction())
    {
        speed = m_motivatorManualPower?
                    m_operatorStick.GetThrottleWithDeadband(true): 0.0;
        m_motivator.SetSpeed(speed);
    }

    //
    // FunnelDoor operation.
    //
    speed = m_operatorStick.GetYWithDeadband(true)/3.0;
    if (speed != 0.0)
    {
        m_funnelDoorManualControl = true;
        m_funnelDoor.SetSpeed(speed);
    }
    else if (m_funnelDoorManualControl)
    {
        m_funnelDoorManualControl = false;
        m_funnelDoor.SetSpeed(0.0);
    }
    
}   //TeleOpPeriodic

/**
 *  This is a callback function for the Joystick object to inform us of
 *  any joystick button events.
 */
void TrcRobot::NotifyButton(
    UINT32 port,
    UINT16 maskButton,
    bool   fPressed)
{
    static UINT8 s_LEDColors[8] =
    {
        LED_COLOR_BLACK,
        LED_COLOR_RED,
        LED_COLOR_GREEN,
        LED_COLOR_YELLOW,
        LED_COLOR_BLUE,
        LED_COLOR_MAGENTA,
        LED_COLOR_CYAN,
        LED_COLOR_WHITE
    };

    if (port == JOYSTICK_LEFT_DRIVE)
    {
        switch (maskButton)
        {
            case Logitech_Trigger:
                break;
        }
    }
    else if (port == JOYSTICK_RIGHT_DRIVE)
    {
        switch (maskButton)
        {
            case Logitech_Trigger:
                break;
        }
    }
    else if (port == JOYSTICK_OPERATOR)
    {
        switch (maskButton)
        {
            case Logitech_Trigger:
                //
                // Perform shoot ball sequence.
                //
                m_motivator.Shoot(fPressed);
                break;
                
            case Logitech_Btn2:
                //
                // Perform pickup ball sequence.
                //
                m_motivator.Open();
                m_elToro.PickupBall(fPressed);
                break;

            case Logitech_Btn3:
                //
                // Reverse spin El Toro.
                //
                if (fPressed)
                {
                    m_motivator.MotorOverride(true);
                    m_elToro.SetSpeed(-ELTORO_POWER);
                    m_motivator.SetSpeed(-MOTIVATOR_POWER);
                    m_motivator.Close();
                }
                else
                {
                    m_motivator.Open();
                    m_motivator.SetSpeed(0.0);
                    m_elToro.SetSpeed(0.0);
                    m_motivator.MotorOverride(false);
                }
                break;

            case Logitech_Btn4:
                //
                // Toggle FunnelDoor OPEN/CLOSE.
                //
                if (fPressed)
                {
                    m_funnelDoorOpened = !m_funnelDoorOpened;
                    if (m_funnelDoorOpened)
                    {
                        m_funnelDoor.Open();
                    }
                    else
                    {
                        m_funnelDoor.Close();
                    }
                }
                break;

            case Logitech_Btn6:
                //
                // Toggle Motivator ON/OFF
                //
                if (fPressed)
                {
                    m_motivatorManualPower = !m_motivatorManualPower;
                }
                break;

            case Logitech_Btn7:
                //
                // Toggle LED flashing.
                //
                if (fPressed)
                {
                    m_flashing = !m_flashing;
                    if (m_flashing)
                    {
                        m_rgbLights.Set(0.5, 0.5, s_LEDColors[m_colorIdx]);
                    }
                    else
                    {
                        m_rgbLights.Set(true, s_LEDColors[m_colorIdx]);
                    }
                }
                break;

            case Logitech_Btn8:
                //
                // Change RGB lights color.
                //
                if (fPressed)
                {
                    if (m_flashing)
                    {
                        m_rgbLights.Set(0.5, 0.5, s_LEDColors[m_colorIdx]);
                    }
                    else
                    {
                        m_rgbLights.Set(true, s_LEDColors[m_colorIdx]);
                    }
                    m_colorIdx++;
                    if (m_colorIdx >= ARRAYSIZE(s_LEDColors))
                    {
                        m_colorIdx = 0;
                    }
                }
                break;

#ifndef _DISABLE_VISION
            //
            // Testing Vision Target.
            //
            case Logitech_Btn9:
                if (fPressed && m_visionTarget.GetTargetInfo(m_targetReport))
                {
#ifdef _DEBUG_VISIONTARGET
                    LCDPrintf((LCD_LINE3, "%d/%d:D=%5.1f,O=%5.1f",
                               m_targetReport.verticalIndex,
                               m_targetReport.horizontalIndex,
                               m_targetReport.distance,
                               m_targetReport.offset));
                    LCDPrintf((LCD_LINE4, "Hot=%d,Score=%5.1f",
                               m_targetReport.Hot,
                               m_targetReport.totalScore));
                    LCDPrintf((LCD_LINE5, "lS=%5.1f,rS=%5.1f",
                               m_targetReport.leftScore,
                               m_targetReport.rightScore));
                    LCDPrintf((LCD_LINE6, "wS=%5.1f,vS=%5.1f",
                               m_targetReport.tapeWidthScore,
                               m_targetReport.verticalScore));
#endif
                }
                break;
#endif

            case Logitech_Btn10:
                if (fPressed)
                {
                    m_motivatorOpened = !m_motivatorOpened;
                    if (m_motivatorOpened)
                    {
                        m_motivator.Open();
                    }
                    else
                    {
                        m_motivator.Close();
                    }
                }
                break;

            case Logitech_Btn11:
                if (fPressed)
                {
                    m_ballStopperRetracted = !m_ballStopperRetracted;
                    if (m_ballStopperRetracted)
                    {
                        m_motivator.RetractBallStopper();
                    }
                    else
                    {
                        m_motivator.ExtendBallStopper();
                    }
                }
                break;
        }
    }
}   //NotifyButton

/**
 *  This is a callback function for the Driver Station button events.
 */
void TrcRobot::NotifyEIODin(UINT32 channel, UINT32 state)
{
    switch (channel)
    {
        case DSSW_MOTIVATOR_RETRACT:
            if (state == 1)
            {
                m_motivatorOpened = false;
                m_motivator.Close();
            }
            else
            {
                m_motivatorOpened = true;
                m_motivator.Open();
            }
            break;

        case DSSW_BALLSTOPPER_RETRACT:
            if (state == 1)
            {
                m_ballStopperRetracted = true;
                m_motivator.RetractBallStopper();
            }
            else
            {
                m_ballStopperRetracted = false;
                m_motivator.ExtendBallStopper();
            }
            break;

        case DSSW_MOTIVATOR_POWERON:
            m_motivatorManualPower = (state == 1);
            break;

        case DSSW_FUNNELDOOR_OPEN:
            if (state == 1)
            {
                m_funnelDoorOpened = true;
                m_funnelDoor.Open();
            }
            else
            {
                m_funnelDoorOpened = false;
                m_funnelDoor.Close();
            }
            break;
    }
}   //NotifyEIODin
