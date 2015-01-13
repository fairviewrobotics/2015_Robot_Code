#if 0
/// Copyright (c) Titan Robotics Club. All rights reserved.
///
/// <module name="DbgTrace.h" />
///
/// <summary>
///     This module contains the definitions of the DbgTrace class.
/// </summary>
///
/// <remarks>
///     Environment: Wind River C++ for National Instrument cRIO based Robot.
/// </remarks>
#endif

#ifndef _DBGTRACE_H
#define _DBGTRACE_H

//
// Module ID.
//
#define MOD_CONSOLE             0x00000001
#define MOD_DATALOGGER          0x00000002
#define MOD_PERFDATA            0x00000004
#define MOD_TASK                0x00000008
#define MOD_COOPMTROBOT         0x00000010
#define MOD_EVENT               0x00000020
#define MOD_TIMER               0x00000040
#define MOD_SM                  0x00000080
#define MOD_FILTER              0x00000100
#define MOD_JOYSTICK            0x00000200
#define MOD_DIGITALIN           0x00000400
#define MOD_ANALOGIN            0x00000800
#define MOD_DSENHDIN            0x00001000
#define MOD_ACCEL               0x00002000
#define MOD_VISION              0x00004000
#define MOD_SOLENOID            0x00008000
#define MOD_SOLLIGHT            0x00010000
#define MOD_CANJAG              0x00020000
#define MOD_SERVO               0x00040000
#define MOD_PIDCTRL             0x00080000
#define MOD_PIDMOTOR            0x00100000
#define MOD_PIDSERVO            0x00200000
#define MOD_PIDDRIVE            0x00400000
#define MOD_DRIVEBASE           0x00800000
#define MOD_LNFOLLOWER          0x01000000
#define MOD_MAIN                0x80000000
#define MOD_ALL                 0xffffffff

#define QUIET                   0
#define INIT                    1
#define API                     2
#define CALLBK                  3
#define EVENT                   4
#define FUNC                    5
#define TASK                    6
#define UTIL                    7
#define HIFREQ                  8

#define FATAL                   1
#define ERR                     2
#define WARN                    3
#define INFO                    4
#define VERBOSE                 5

#ifndef TRACE_PERIOD
  #define TRACE_PERIOD          1000    //in msec
#endif

#ifndef SAMPLING_PERIOD
  #define SAMPLING_PERIOD       500     //in msec
#endif

#define TPrintf                 printf

//
// Trace macros.
//
#ifdef _DBGTRACE_ENABLED
    #define TModEnterMsg(m,p)   if (g_Trace.m_fTraceEnabled && \
                                    ((g_Trace.m_traceModules & (m)) != 0) && \
                                    (_traceLevel <= g_Trace.m_traceLevel)) \
                                { \
                                    g_Trace.FuncPrefix(MOD_NAME, \
                                                       __FUNCTION__, \
                                                       true, \
                                                       false); \
                                    TPrintf p; \
                                    TPrintf(")\n" ESC_NORMAL); \
                                }
    #define TModEnter(m)        if (g_Trace.m_fTraceEnabled && \
                                    ((g_Trace.m_traceModules & (m)) != 0) && \
                                    (_traceLevel <= g_Trace.m_traceLevel)) \
                                { \
                                    g_Trace.FuncPrefix(MOD_NAME, \
                                                       __FUNCTION__, \
                                                       true, \
                                                       true); \
                                }
    #define TModExitMsg(m,p)    if (g_Trace.m_fTraceEnabled && \
                                    ((g_Trace.m_traceModules & (m)) != 0) && \
                                    (_traceLevel <= g_Trace.m_traceLevel)) \
                                { \
                                    g_Trace.FuncPrefix(MOD_NAME, \
                                                       __FUNCTION__, \
                                                       false, \
                                                       false); \
                                    TPrintf p; \
                                    TPrintf("\n" ESC_NORMAL); \
                                }
    #define TModExit(m)         if (g_Trace.m_fTraceEnabled && \
                                    ((g_Trace.m_traceModules & (m)) != 0) && \
                                    (_traceLevel <= g_Trace.m_traceLevel)) \
                                { \
                                    g_Trace.FuncPrefix(MOD_NAME, \
                                                       __FUNCTION__, \
                                                       false, \
                                                       true); \
                                }
    #define TModMsg(m,e,p)      if (((g_Trace.m_traceModules & (m)) != 0) && \
                                    ((e) <= g_Trace.m_msgLevel)) \
                                { \
                                    g_Trace.MsgPrefix(MOD_NAME, \
                                                      __FUNCTION__, \
                                                      e); \
                                    TPrintf p; \
                                    TPrintf("\n" ESC_NORMAL); \
                                }
    #define TMsg(e,p)           if ((e) <= g_Trace.m_msgLevel) \
                                { \
                                    g_Trace.MsgPrefix(MOD_NAME, \
                                                      __FUNCTION__, \
                                                      e); \
                                    TPrintf p; \
                                    TPrintf("\n" ESC_NORMAL); \
                                }
    #define TEnable(b)          g_Trace.m_fTraceEnabled = (b)
    #define TraceInit(m,l,e)    g_Trace.Initialize(m, l, e)
    #define TLevel(l)           uint32_t _traceLevel = l
    #define TEnterMsg(p)        TModEnterMsg(MOD_ID, p)
    #define TEnter()            TModEnter(MOD_ID)
    #define TExitMsg(p)         TModExitMsg(MOD_ID, p)
    #define TExit()             TModExit(MOD_ID)
    #define TFatal(p)           TMsg(FATAL, p)
    #define TErr(p)             TMsg(ERR, p)
    #define TWarn(p)            TMsg(WARN, p)
    #define TInfo(p)            TModMsg(MOD_ID, INFO, p)
    #define TVerbose(p)         TModMsg(MOD_ID, VERBOSE, p)
    #define TMsgPeriod(t,p)     { \
                                    static uint32_t _usecNextTime = 0; \
                                    if (GetFPGATime() >= _usecNextTime) \
                                    { \
                                        _usecNextTime = GetFPGATime() + \
                                                        (uint32_t)(t)*1000; \
                                        TModMsg(MOD_ID, INFO, p); \
                                    } \
                                }
    #define TSampling(p)        TMsgPeriod(SAMPLING_PERIOD, p)
    #define TAssertPeriod(t,p,r) { \
                                    uint32_t _time = GetMsecTime(); \
                                    float _period = (float)(_time - (t))/ \
                                                    1000.0; \
                                    t = _time; \
                                    if (fabs(_period - (p)) > (r)) \
                                    { \
                                        TWarn(("Period variance exceeding " \
                                               "tolerance (period=%f)", \
                                               _period)); \
                                    } \
                                }
    #define TPeriodStart()      if (GetFPGATime() >= g_Trace.m_traceTime) \
                                { \
                                    g_Trace.m_traceTime = GetFPGATime() + \
                                                          TRACE_PERIOD; \
                                    TEnable(true); \
                                }
    #define TPeriodEnd()        TEnable(false)
#else
    #define TEnable(b)
    #define TraceInit(m,l,e)
    #define TLevel(l)
    #define TEnterMsg(p)
    #define TEnter()
    #define TExitMsg(p)
    #define TExit()
    #define TMsg(e,p)
    #define TFatal(p)
    #define TErr(p)
    #define TWarn(p)
    #define TInfo(p)
    #define TVerbose(p)
    #define TMsgPeriod(t,p)
    #define TSampling(p)
    #define TAssertPeriod(t,p,r)
    #define TPeriodStart()
    #define TPeriodEnd()
#endif  //ifdef _DBGTRACE_ENABLED

/**
 * This class implements the debug tracing object. It provides two facilities.
 * One allows the functions to trace the enter and exit conditions of the call
 * by dumping the calling parameters of function entry and the return value
 * of function exit. The other one allows the function to print out different
 * level of messages such as fatal message, error message, warning message,
 * info message and verbose message etc.
 */
class DbgTrace
{
public:
    bool   m_fTraceEnabled;
    uint32_t m_traceModules;
    uint32_t m_traceLevel;
    uint32_t m_msgLevel;
    uint32_t m_traceTime;

private:
    int32_t  m_indentLevel;

public:
    /**
     * Constructor for the DbgTrace object.
     */
    DbgTrace(
        void
        )
    {
        m_fTraceEnabled = false;
        m_traceModules = 0;
        m_traceLevel = 0;
        m_msgLevel = 0;
        m_traceTime = 0;
        m_indentLevel = 0;
    }   //DbgTrace

    /**
     * Destructor for the DbgTrace object.
     */
    virtual
    ~DbgTrace(
        void
        )
    {
    }   //~DbgTrace

    /**
     * This function initializes the tracing module with the specified
     * module IDs and trace levels.
     *
     * @param traceModules Bit mask specifying which modules to enable tracing
     *        with. Each module is assigned a bit ID in the bit mask.
     * @param traceLevel Specifies the function trace level at or below which
     *        function tracing is enabled.
     * @param msgLevel Specifies the message trace level at or below which
     *        message tracing is enabled.
     */
    void
    Initialize(
        uint32_t traceModules,
        uint32_t traceLevel,
        uint32_t msgLevel
        )
    {
        m_traceModules = traceModules;
        m_traceLevel = traceLevel;
        m_msgLevel = msgLevel;
        m_indentLevel = 0;
    }   //Initialize

    /**
     * This method generates the function trace prefix string. The prefix
     * contains the indentation, the module name and the function name.
     *
     * @param pszMod Specifies the name of the module.
     * @param pszFunc Specifies the name of the function.
     * @param fEnter Specifies whether we are entering or exiting the
     *        function.
     * @param fNewLine Specifies whether we will print a new line.
     */
    void
    FuncPrefix(
        const char *pszMod,
        const char *pszFunc,
        bool        fEnter,
        bool        fNewLine
        )
    {
        if (fEnter)
        {
            m_indentLevel++;
        }

#ifdef _USE_COLORFONT
        TPrintf(ESC_FGB_MAGENTA);
#endif
        for (int32_t i = 0; i < m_indentLevel; i++)
        {
            TPrintf("| ");
        }

        TPrintf("%s.%s", pszMod, pszFunc);

        if (fEnter)
        {
            TPrintf("%s", fNewLine? "()\n": "(");
        }
        else
        {
            TPrintf("%s", fNewLine? "!\n" ESC_NORMAL: "");
            m_indentLevel--;
        }
    }   //FuncPrefix

    /**
     * This method generates the message trace prefix string. The prefix
     * contains the module and function names as well as message level info
     * in which the message is printed.
     *
     * @param pszMod Specifies the name of the module.
     * @param pszFunc Specifies the name of the function.
     * @param msgLevel Specifies message level.
     */
    void
    MsgPrefix(
        const char *pszMod,
        const char *pszFunc,
        uint32_t      msgLevel
        )
    {
        char *pszPrefix = "_Unk: ";
#ifdef _USE_COLORFONT
        char *pszColor = ESC_NORMAL;
#endif

        switch (msgLevel)
        {
        case FATAL:
            pszPrefix = "_Fatal: ";
#ifdef _USE_COLORFONT
            pszColor = ESC_PREFIX SGR_FG_YELLOW
                       ESC_SEP SGR_BRIGHT
                       ESC_SEP SGR_BG_RED
                       ESC_SUFFIX;
#endif
            break;

        case ERR:
            pszPrefix = "_Err: ";
#ifdef _USE_COLORFONT
            pszColor = ESC_FGB_RED;
#endif
            break;

        case WARN:
            pszPrefix = "_Warn: ";
#ifdef _USE_COLORFONT
            pszColor = ESC_FGB_YELLOW;
#endif
            break;

        case INFO:
            pszPrefix = "_Info: ";
#ifdef _USE_COLORFONT
            pszColor = ESC_FGB_GREEN;
#endif
            break;

        case VERBOSE:
            pszPrefix = "_Verbose: ";
#ifdef _USE_COLORFONT
            pszColor = ESC_FGB_WHITE;
#endif
            break;
        }

#ifdef _USE_COLORFONT
        TPrintf("%s%s.%s%s", pszColor, pszMod, pszFunc, pszPrefix);
#else
        TPrintf("%s.%s%s", pszMod, pszFunc, pszPrefix);
#endif
    }   //MsgPrefix
};	//class DbgTrace

#ifdef _DBGTRACE_ENABLED
    DbgTrace g_Trace;
#endif

#endif  //ifndef _DBGTRACE_H
