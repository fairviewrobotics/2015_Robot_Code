/**
 *  This module contains the definitions of all implementation constants of
 *  the robot.
 */

//
// CAN IDs.
//
#define CANID_LEFTFRONT_JAG             2       //Orange
#define CANID_LEFTREAR_JAG              3       //Yellow
#define CANID_RIGHTFRONT_JAG            4       //Green
#define CANID_RIGHTREAR_JAG             5       //Blue
#define CANID_LEFTMOTIVATOR_JAG         6       //Purple
#define CANID_RIGHTMOTIVATOR_JAG        7       //Gray
#define CANID_FUNNELDOOR_JAG            8       //White

#define JAG_SYNCGROUP_MOTIVATOR         1

//
// Analog input channels.
//
#define AIN_GYRO                        1

//
// Digital input channels.
//
#define DIN_COMPRESSOR_PRESSURE_SWITCH  1       //Green

//
// PWM channels.
//
#define PWM_LEFT_ELTORO_MOTOR           1       //Purple
#define PWM_RIGHT_ELTORO_MOTOR          2       //Gray

//
// Relay channels.
//
#define RELAY_COMPRESSOR_POWER          1       //Green
#define RELAY_RINGLIGHT_POWER           2       //Blue

//
// Solenoid channels.
//

// Solenoid module 1.
#define SOL_MOTIVATOR_EXTEND            ((UINT32)1)
#define SOL_MOTIVATOR_RETRACT           ((UINT32)2)
#define SOL_ELTORO_EXTEND               ((UINT32)3)
#define SOL_ELTORO_RETRACT              ((UINT32)4)
#define SOL_BALLSTOPPER_EXTEND          ((UINT32)5)
#define SOL_BALLSTOPPER_RETRACT         ((UINT32)6)

// Solenoid module 2.
#define SOL_RED_LED                     ((UINT32)6)
#define SOL_GREEN_LED                   ((UINT32)7)
#define SOL_BLUE_LED                    ((UINT32)8)

//
// Input subsystem.
//
#define JOYSTICK_LEFT_DRIVE             1
#define JOYSTICK_RIGHT_DRIVE            2
#define JOYSTICK_OPERATOR               3

//
// Driver Station Digital Inputs.
//
#define DSSW_MOTIVATOR_RETRACT          DSCHN_SW1
#define DSSW_BALLSTOPPER_RETRACT        DSCHN_SW2
#define DSSW_MOTIVATOR_POWERON          DSCHN_SW3
#define DSSW_FUNNELDOOR_OPEN            DSCHN_SW4

//
// Drive subsystem.
//
#define MOTOR_LEFT_FRONT_REVERSE        false
#define MOTOR_LEFT_REAR_REVERSE         false
#define MOTOR_RIGHT_FRONT_REVERSE       true
#define MOTOR_RIGHT_REAR_REVERSE        true

#define DRIVE_ENCODER_PPR               360
#define DRIVE_XDISTANCE_PER_REV         23.10       //Wheel circumference (6*PI=18.85) 26.4
#define DRIVE_YDISTANCE_PER_REV         21.8675     //19.5508
#define TURN_DEGREE_PER_REV             60.0000     //60.0000

#define XDRIVE_KP                       0.022       //0.022
#define XDRIVE_KI                       0.0
#define XDRIVE_KD                       0.0
#define XDRIVE_KF                       0.0
#define XDRIVE_TOLERANCE                1.0
#define XDRIVE_SETTLING                 200

#define YDRIVE_KP                       0.022        //0.022
#define YDRIVE_KI                       0.0
#define YDRIVE_KD                       0.0
#define YDRIVE_KF                       0.0
#define YDRIVE_TOLERANCE                1.0
#define YDRIVE_SETTLING                 200

#define TURN_KP                         0.012       //0.012[3/2/14]
#define TURN_KI                         0.0
#define TURN_KD                         0.0
#define TURN_KF                         0.0
#define TURN_TOLERANCE                  1.0
#define TURN_SETTLING                   200

//
// ElToro subsystem.
//
#define ELTORO_POWER                    1.0
#define ELTORO_EXTEND                   SolID(SOL_ELTORO_EXTEND)
#define ELTORO_RETRACT                  SolID(SOL_ELTORO_RETRACT)

//
// Motivator subsystem.
//
#define MOTIVATOR_EXTEND                SolID(SOL_MOTIVATOR_EXTEND)
#define MOTIVATOR_RETRACT               SolID(SOL_MOTIVATOR_RETRACT)
#define MOTIVATOR_POWER                 1.0

//
// FunnelDoor subsystem.
//
#define FUNNELDOOR_POWER                0.2

//
// BallStopper subsystem.
//
#define BALLSTOPPER_EXTEND              SolID(SOL_BALLSTOPPER_EXTEND)
#define BALLSTOPPER_RETRACT             SolID(SOL_BALLSTOPPER_RETRACT)

//
// VisionTarget subsystem.
//
#define CAMERA_IP                       "10.4.92.11"

//
// RGBLights subsystem.
//
#define LED_RED                         SolID(SOL_RED_LED)
#define LED_GREEN                       SolID(SOL_GREEN_LED)
#define LED_BLUE                        SolID(SOL_BLUE_LED)
#define LED_ALL                         ((UINT8)(LED_RED | LED_GREEN | LED_BLUE))

#define LED_COLOR_BLACK                 0
#define LED_COLOR_RED                   (LED_RED)
#define LED_COLOR_GREEN                 (LED_GREEN)
#define LED_COLOR_BLUE                  (LED_BLUE)
#define LED_COLOR_YELLOW                ((UINT8)(LED_RED | LED_GREEN))
#define LED_COLOR_MAGENTA               ((UINT8)(LED_RED | LED_BLUE))
#define LED_COLOR_CYAN                  ((UINT8)(LED_GREEN | LED_BLUE))
#define LED_COLOR_WHITE                 ((UINT8)(LED_RED | LED_GREEN | LED_BLUE))

//
// Autonomous.
//
#define AUTO_STRATEGY_INVALID           0
#define AUTO_STRATEGY_MOVE_FWD          1
#define AUTO_STRATEGY_LOW_GOAL          2
#define AUTO_STRATEGY_LEFT_LOW_GOAL     3
#define AUTO_STRATEGY_RIGHT_LOW_GOAL    4
#define AUTO_STRATEGY_TWO_LOW_GOALS     5

#define AUTO_DEF_DISTANCE               5       //in ft
#define AUTO_GOAL_DISTANCE              14.5    //in ft

#define SMSTATE_MOVE_FWD                (SMSTATE_STARTED + 100)
#define SMSTATE_LOW_GOAL                (SMSTATE_STARTED + 200)
#define SMSTATE_LOW_GOAL_VISION         (SMSTATE_STARTED + 300)
#define SMSTATE_TWO_GOALS               (SMSTATE_STARTED + 400)
#define SMSTATE_AUTO_DONE               (SMSTATE_STARTED + 999)

//
// TeleOp.
//
#define ELTORO_DELAY                    1.0
#define SHOOT_DELAY                     1.5

//
// Test mode.
//
#define TESTOP_INVALID                  0
#define TESTOP_DRIVE_ENCODERS           1
#define TESTOP_TIMED_DRIVE_X            2
#define TESTOP_TIMED_DRIVE_Y            3
#define TESTOP_DRIVE_X                  4
#define TESTOP_DRIVE_Y                  5
#define TESTOP_TURN                     6

#define SMSTATE_TEST_DRIVE_ENCODERS     (SMSTATE_STARTED + 100)
#define SMSTATE_TEST_TIMED_DRIVE_X      (SMSTATE_STARTED + 200)
#define SMSTATE_TEST_TIMED_DRIVE_Y      (SMSTATE_STARTED + 300)
#define SMSTATE_TEST_DRIVE_X            (SMSTATE_STARTED + 400)
#define SMSTATE_TEST_DRIVE_Y            (SMSTATE_STARTED + 500)
#define SMSTATE_TEST_TURN               (SMSTATE_STARTED + 600)
#define SMSTATE_TEST_DONE               (SMSTATE_STARTED + 999)

#define TEST_DRIVE_DISTANCE             48.0
