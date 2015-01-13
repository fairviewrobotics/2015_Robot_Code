#if 0
/// Copyright (c) Michael Tsang. All rights reserved.
///
/// <module name="common.h" />
///
/// <summary>
///   This module contains common definitions that can be used anywhere.
/// </summary>
///
/// <remarks>
///   Environment: Wind River C++ for National Instrument cRIO based Robot.
/// </remarks>
#endif

#ifndef _COMMON_H
#define _COMMON_H

//
// Constants.
//
#define PI                      3.141592653589793
#define FEET_PER_METER          3.280839895
#define METERS_PER_FOOT         (1.0/FEET_PER_METER)
#define GRAVITY_CONSTANT        9.80665 // meter/sq(sec)
#define GYRO_SENSITIVITY_2009   0.0125  // volts/degree/sec
#define GYRO_SENSITIVITY_2010   0.007   // volts/degree/sec

//
// Macros.
//
#define __in
#define __out
#define __inout

#define ARRAYSIZE(a)            (sizeof(a)/sizeof(a[0]))
#define SAFE_DELETE(p)          if (p != NULL) \
                                { \
                                    delete p; \
                                    p = NULL; \
                                }
#define MAGNITUDE(x,y)          sqrt(pow(x, 2) + pow(y, 2))
#define RADIANS_TO_DEGREES(n)   ((n)*180.0/PI)
// Forward is 0-radian
#define DIR_RADIANS(x,y)        (((x == 0.0) && (y == 0.0))? 0.0: atan2(x, y))
#define DIR_DEGREES(x,y)        RADIANS_TO_DEGREES(DIR_RADIANS(x, y))

#define GetMsecTime()           (GetFPGATime()/1000)

//
// The BOUND macro limits the value (n) within the bounds between the given
// low (l) and high (h).
//
#define BOUND(n,l,h)            (((n) < (l))? (l): ((n) > (h))? (h): (n))

//
// The NORMALIZE macro transforms a value (n) in the range between (sl) and
// (sh) to the range between (tl) and (th).
//
#define NORMALIZE(n,sl,sh,tl,th) (((n) - (sl))*((th) - (tl))/((sh) - (sl)) + (tl))
#define NORMALIZE_DRIVE(n,m)    NORMALIZE(n, -1.0, 1.0, -(m), (m))

//
// This macro ignores input value (n) that is within the DEADBAND_THRESHOLD.
// This is necessary because analog joysticks do not always centered at zero.
// So if the joystick is at the rest position, we will consider it zero even
// though the value is non-zero but within DEADBAND_THRESHOLD.
//
#define ABS(n)                  (((n) < 0)? -(n): (n))
#define DEADBAND(n,t)           ((ABS(n) > (t))? (n): 0)

//
// This macro limits the input value (n) to the range between -128 and 127.
// This is useful when calculations on the input value may bring the result
// outside of the valid range. This macro will make sure the result is within
// bounds.
//
#define BOUND_INPUT(n)          BOUND(n, -1.0, 1.0)

#endif  //ifndef _COMMON_H
