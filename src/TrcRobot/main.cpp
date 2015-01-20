//
// Program switches.
//
#define _ENABLE_COMPETITION
#define _USE_COLORFONT
#define _DISABLE_VISION

#ifndef _ENABLE_COMPETITION
#define _DBGTRACE_ENABLED
//#define _LOGDATA_DRIVEBASE

//#define _DEBUG_TELEOP
//#define _DEBUG_DRIVEBASE
//#define _DEBUG_MOTIVATOR
//#define _DEBUG_FUNNELDOOR
#define _DEBUG_VISIONTARGET
#endif

#define PROGRAM_NAME            "Aerial Assist"

//
// Include libraries here.
//
#include "WPILib.h"
#include "RobotInfo.h"          //Robot configurations.
#include "..\frclib\TrcLib.h"

//
// Tracing info.
//
#ifdef MOD_ID
    #undef MOD_ID
#endif
#define MOD_ID                  MOD_MAIN
#ifdef MOD_NAME
    #undef MOD_NAME
#endif
#define MOD_NAME                "Main"

#define TRACE_MODULES           (MOD_MAIN)
#define TRACE_LEVEL             QUIET
#define MSG_LEVEL               INFO

//
// Include project files here.
//

// Include subsystems here.
#include "DashboardDataFormat.h"
#include "ElToro.h"
#include "Motivator.h"
#include "FunnelDoor.h"
#ifndef _DISABLE_VISION
#include "VisionTarget.h"
#endif

// Include main files here.
#include "TrcRobot.h"
#include "Disabled.h"
#include "Auto.h"
#include "TeleOp.h"
#include "Test.h"
//
// Specifies main robot object.
//
START_ROBOT_CLASS(TrcRobot);

