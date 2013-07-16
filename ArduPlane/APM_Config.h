// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

// This file is just a placeholder for your configuration file.  If
// you wish to change any of the setup parameters from their default
// values, place the appropriate #define statements here.

#define CONFIG_APM_HARDWARE APM_HARDWARE_APM2

// Ordinary users should please ignore the following define.
// APM2_BETA_HARDWARE is used to support early (September-October 2011) APM2
// hardware which had the BMP085 barometer onboard. Only a handful of
// developers have these boards.
//#define APM2_BETA_HARDWARE

// The following are the recommended settings for Xplane
// simulation. Remove the leading "/* and trailing "*/" to enable:

#define HIL_MODE            HIL_MODE_DISABLED
//#define HIL_MODE            HIL_MODE_ATTITUDE

/*
 *  // HIL_MODE SELECTION
 *  //
 *  // Mavlink supports
 *  // 1. HIL_MODE_ATTITUDE : simulated position, airspeed, and attitude
 *  // 2. HIL_MODE_SENSORS: full sensor simulation
 *  //#define HIL_MODE            HIL_MODE_ATTITUDE
 *
 */

// Allows debug messages to be sent through GPS terminal (automatically turned off when HIL mode is disabled)
#define DEBUGGER ENABLED

// Switches between computer controlled throttle commands and manual pass through
#define HOVER_THROTTLE ENABLED
//#define HOVER_THROTTLE DISABLED

// Used limit pitch target for hover modes during testing
//#define PITCH_LIMITER ENABLED
#define PITCH_LIMITER DISABLED 


/////// Hard coded variables used in control algorithms///////////////////////////////////////////////////////////////

// Preset throttle settings used with either yaw or pitch divergence criteria are met
#define DIVERGENCE_THROTTLE_MAX 0.75
#define DIVERGENCE_THROTTLE_MIN 0.4

// Angle (deg) at which divergence criteria is met when in hover
#define DIVERGENCE_ANGLE 5.0

// Maximum magnitude of commanded sink/climb rate (m/s)
#define SINK_RATE_MAX 2

// Fequency cut off (Hz) for discrete low pass filter on altitude derivative
#define _fCut_alt 20   // Set at 1 Hz originally bit I think that might have been hurting throttle controller performance so moved it back to standard 20 Hz