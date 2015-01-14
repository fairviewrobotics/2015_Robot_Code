#if 0
/// Copyright (c) Titan Robotics Club. All rights reserved.
///
/// <module name="TrcPIDCtrl.h" />
///
/// <summary>
///     This module contains the definition and implementation of the
///     TrcPIDCtrl class.
/// </summary>
///
/// <remarks>
///     Environment: Wind River C++ for National Instrument cRIO based Robot.
/// </remarks>
#endif

#ifndef _TRCPIDCTRL_H
#define _TRCPIDCTRL_H

#ifdef MOD_ID
    #undef MOD_ID
#endif
#define MOD_ID                  MOD_PIDCTRL
#ifdef MOD_NAME
    #undef MOD_NAME
#endif
#define MOD_NAME                "TrcPIDCtrl"

//
// PIDCtrl options.
//
#define PIDCTRLO_INVERSE        0x00000001
#define PIDCTRLO_ABS_SETPT      0x00000002
#define PIDCTRLO_SPEED_CTRL     0x00000004
#define PIDCTRLO_NO_OSCILLATE   0x00000008

class TrcPIDCtrl;

/**
 * This abstract class defines the PIDInput object. The object is
 * a callback interface. It is not meant to be created as an object.
 * Instead, it should be inherited by a subclass who needs to provide
 * input data to a PID controller.
 */
class PIDInput
{
public:
    /**
     * This function is provided by the subclass to provide input to the PID
     * controller object.
     *
     * @param pidCtrl Points to the PIDCtrl object that requires input.
     *
     * @return Returns the input data for the PID controller.
     */
    virtual
    float
    GetInput(
        TrcPIDCtrl *pidCtrl
        ) = 0;
};  //class PIDInput

/**
 * This class defines and implements the TrcPIDCtrl object. This object
 * replaces the PIDController object from the WPI library. The PIDController
 * object in the WPI library is not flexible enough because it dealt with
 * PID input and PID output directly. In contrast, the TrcPIDCtrl object is
 * a simple primitive that calculates the PID output according to the given
 * PID input. So it is agnostic to the kind of input, output or even how the
 * calculated output is used. This is especially important because the
 * TrcPIDDrive object is going to combine three PID controllers to
 * calculate the drive output. One PID controller is for driving straight
 * distance in the X direction, one PID controller is for driving straight
 * distance in the Y direction, and one for rotation. In addition, the WPI
 * PIDController is creating a periodic loop per controller. In the TRC
 * library, we will do periodic loop at a higher level in the TrcPIDDrive
 * object instead.
 */
class TrcPIDCtrl
{
private:
    double  m_Kp;
    double  m_Ki;
    double  m_Kd;
    double  m_Kf;
    float   m_tolerance;
    uint32_t  m_settlingTime;
    uint32_t  m_pidCtrlOptions;
    float   m_minInput;
    float   m_maxInput;
    float   m_minOutput;
    float   m_maxOutput;
    float   m_prevError;
    float   m_totalError;
    uint32_t  m_startSettling;
    float   m_setPoint;
    float   m_output;
    uint32_t  m_timeStamp;

public:
    /**
     * Constructor: Create an instance of the TrcPIDCtrl object.
     *
     * @param idString Identifying the PID controller, mainly for logging
     *        purpose.
     * @param Kp Specifies the proportional coefficient.
     * @param Ki Specifies the integral coefficient.
     * @param Kd Specifies the derivative coefficient.
     * @param Kf Specifies the feed forward coefficient.
     * @param tolerance Specifies the on-target tolerance.
     * @param settlingTime Specifes the on-target settling time in msec.
     * @param pidCtrlOptions Specifies the option flags.
     */
    TrcPIDCtrl(
        char  *idString,
        double Kp,
        double Ki = 0.0,
        double Kd = 0.0,
        double Kf = 0.0,
        float  tolerance = 0.0,
        uint32_t settlingTime = 0,
        uint32_t pidCtrlOptions = 0
        ): m_Kp(Kp)
         , m_Ki(Ki)
         , m_Kd(Kd)
         , m_Kf(Kf)
         , m_tolerance(tolerance)
         , m_settlingTime(settlingTime)
         , m_pidCtrlOptions(pidCtrlOptions)
         , m_minInput(0.0)
         , m_maxInput(0.0)
         , m_minOutput(-1.0)
         , m_maxOutput(1.0)
         , m_prevError(0.0)
         , m_totalError(0.0)
         , m_startSettling(0)
         , m_setPoint(0.0)
         , m_output(0.0)
         , m_timeStamp(GetMsecTime())
    {

#ifdef _LOGDATA_PIDCTRL
        DataLogger *dataLogger = DataLogger::GetInstance();
        dataLogger->AddDataPoint(MOD_NAME, idString, "setPt", "%f",
                                 DataFloat, &m_setPoint);
        dataLogger->AddDataPoint(MOD_NAME, idString, "output", "%f",
                                 DataFloat, &m_output);
        dataLogger->AddDataPoint(MOD_NAME, idString, "error", "%f",
                                 DataFloat, &m_prevError);
        dataLogger->AddDataPoint(MOD_NAME, idString, "totalError", "%f",
                                 DataFloat, &m_totalError);
#endif

    }   //TrcPIDCtrl

    /**
     * Constructor: Create an instance of the TrcPIDCtrl object.
     *
     * @param Kp Specifies the proportional coefficient.
     * @param Ki Specifies the integral coefficient.
     * @param Kd Specifies the derivative coefficient.
     * @param Kf Specifies the feed forward coefficient.
     * @param tolerance Specifies the on-target tolerance.
     * @param settlingTime Specifes the on-target settling time in msec.
     * @param pidCtrlOptions Specifies the option flags.
     */
    TrcPIDCtrl(
        double Kp,
        double Ki = 0.0,
        double Kd = 0.0,
        double Kf = 0.0,
        float  tolerance = 0.0,
        uint32_t settlingTime = 0,
        uint32_t pidCtrlOptions = 0
        ): m_Kp(Kp)
         , m_Ki(Ki)
         , m_Kd(Kd)
         , m_Kf(Kf)
         , m_tolerance(tolerance)
         , m_settlingTime(settlingTime)
         , m_pidCtrlOptions(pidCtrlOptions)
         , m_minInput(0.0)
         , m_maxInput(0.0)
         , m_minOutput(-1.0)
         , m_maxOutput(1.0)
         , m_prevError(0.0)
         , m_totalError(0.0)
         , m_startSettling(0)
         , m_setPoint(0.0)
         , m_output(0.0)
    {}   //TrcPIDCtrl

    /**
     * Destructor: Destroy an instance of the TrcPIDCtrl object.
     */
    ~TrcPIDCtrl(
        void
        )
    {}   //~TrcPIDCtrl

    /**
     * This function resets the PID controller.
     */
    void
    Reset(
        void
        )
    {
        m_prevError = 0.0;
        m_totalError = 0.0;
        m_output = 0.0;

        return;
    }   //Reset

    /**
     * This function gets the proportional constant.
     *
     * @return Proportional coefficient constant.
     */
    float
    GetKp(
        void
        )
    {
        return m_Kp;
    }   //GetKp

    /**
     * This function gets the integral constant.
     *
     * @return Integral coefficient constant.
     */
    float
    GetKi(
        void
        )
    {
        return m_Ki;
    }   //GetKi

    /**
     * This function gets the differential constant.
     *
     * @return Differential coefficient constant.
     */
    float
    GetKd(
        void
        )
    {
        return m_Kd;
    }   //GetKd

    /**
     * This function gets the feed forward constant.
     *
     * @return Feed forward coefficient constant.
     */
    float
    GetKf(
        void
        )
    {
        return m_Kf;
    }   //GetKf

    /**
     * This function gets the PID controller constants.
     *
     * @param pKp Points to the variable to hold the proportional constant.
     * @param pKi Points to the variable to hold the integral constant.
     * @param pKd Points to the variable to hold the differential constant.
     * @param pKf Points to the variable to hold the feed forward constant.
     */
    void
    GetPID(
        double *pKp = NULL,
        double *pKi = NULL,
        double *pKd = NULL,
        double *pKf = NULL
        )
    {
        if (pKp != NULL)
        {
            *pKp = m_Kp;
        }

        if (pKi != NULL)
        {
            *pKi = m_Ki;
        }

        if (pKd != NULL)
        {
            *pKd = m_Kd;
        }

        if (pKf != NULL)
        {
            *pKf = m_Kf;
        }

        return;
    }   //GetPID

    /**
     * This function sets the PID controller constants.
     *
     * @param Kp Specifies the proportional constant.
     * @param Ki Specifies the integral constant.
     * @param Kd Specifies the differential constant.
     * @param Kf Specifies the feed forward constant.
     */
    void
    SetPID(
        double Kp,
        double Ki = 0.0,
        double Kd = 0.0,
        double Kf = 0.0
        )
    {
        m_Kp = Kp;
        m_Ki = Ki;
        m_Kd = Kd;
        m_Kf = Kf;

        return;
    }   //SetPID

    /**
     * This function gets the last error.
     *
     * @return the last error.
     */
    float
    GetError(
        void
        )
    {
        return m_prevError;
    }   //GetError

    /**
     * This function gets the PID controller setpoint.
     *
     * @return the current setpoint.
     */
    float
    GetTarget(
        void
        )
    {
        return m_setPoint;
    }   //GetTarget

    /**
     * This function sets the PID controller setpoint.
     *
     * @param setPoint Specifies the target setpoint.
     * @param currInput Specifies the current input value.
     */
    void
    SetTarget(
        float setPoint,
        float currInput
        )
    {
        if (!(m_pidCtrlOptions & PIDCTRLO_ABS_SETPT))
        {
            setPoint += currInput;
        }

        if (m_maxInput > m_minInput)
        {
            if (setPoint > m_maxInput)
            {
                m_setPoint = m_maxInput;
            }
            else if (setPoint < m_minInput)
            {
                m_setPoint = m_minInput;
            }
            else
            {
                m_setPoint = setPoint;
            }
        }
        else
        {
            m_setPoint = setPoint;
        }
        m_prevError = m_setPoint - currInput;
        m_totalError = 0.0;
        m_startSettling = GetMsecTime();
        m_timeStamp = GetMsecTime();

        return;
    }   //SetTarget

    /**
     * This function determines if we are on target by checking if the
     * previous error is within target tolerance and remain within tolerance
     * for at least the settling period.
     *
     * @return True if we are on target, false otherwise.
     */
    bool
    OnTarget(
        void
        )
    {
        bool fOnTarget = false;

        if (m_pidCtrlOptions & PIDCTRLO_NO_OSCILLATE)
        {
            if (fabs(m_prevError) <= m_tolerance)
            {
                fOnTarget = true;
            }
        }
        else if (fabs(m_prevError) > m_tolerance)
        {
            m_startSettling = GetMsecTime();
        }
        else if (GetMsecTime() - m_startSettling >= m_settlingTime)
        {
            fOnTarget = true;
        }

        return fOnTarget;
    }   //OnTarget

    /**
     * This function sets the minimum and maximum values expected from the
     * input.
     *
     * @param minInput Specifies the minimum value.
     * @param maxInput Specifies the maximum value.
     */
    void
    SetInputRange(
        float minInput,
        float maxInput
        )
    {
        m_minInput = minInput;
        m_maxInput = maxInput;

        return;
    }   //SetInputRange

    /**
     * This function limits the minimum and maximum values of the output.
     *
     * @param minOutput Specifies the minimum value.
     * @param maxOutput Specifies the maximum value.
     */
    void
    SetOutputRange(
        float minOutput,
        float maxOutput
        )
    {
        m_minOutput = minOutput;
        m_maxOutput = maxOutput;

        return;
    }   //SetOutputRange

    /**
     * This function returns the calculated PID output according to the input
     * value from the input source.
     *
     * @param currInput Specifies the current input value.
     *
     * @return Returns the PID control source input value.
     */
    float
    CalcPIDOutput(
        float currInput
        )
    {
        float output;
        float error;
        float adjTotalError;
        uint32_t currTime;
        float deltaTime;

        currTime = GetMsecTime();
        deltaTime = (currTime - m_timeStamp)/1000.0;
        m_timeStamp = currTime;

        error = m_setPoint - currInput;
        if (m_pidCtrlOptions & PIDCTRLO_INVERSE)
        {
            error = -error;
        }

        if (m_Ki != 0.0)
        {
            adjTotalError = m_Ki*(m_totalError + error*deltaTime);
            if (adjTotalError < m_maxOutput)
            {
                if (adjTotalError > m_minOutput)
                {
                    m_totalError += error*deltaTime;
                }
                else
                {
                    m_totalError = m_minOutput/m_Ki;
                }
            }
            else
            {
                m_totalError = m_maxOutput/m_Ki;
            }
        }

        output = m_Kp*error +
                 m_Ki*m_totalError +
                 m_Kd*(error - m_prevError)/deltaTime +
                 m_Kf*m_setPoint;

        output = BOUND(output, m_minOutput, m_maxOutput);

        m_prevError = error;
        m_output = output;

        return output;
    }   //CalcPIDOutput

};  //class TrcPIDCtrl

#endif  //ifndef _TRCPIDCTRL_H
