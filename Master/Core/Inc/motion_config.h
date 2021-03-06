#pragma once

#if INTECH_TARGET == 2

#define MOTION_LEFT_KP 20
#define MOTION_LEFT_KI 0.0001
#define MOTION_LEFT_KD 2e-6

#define MOTION_RIGHT_KP 15
#define MOTION_RIGHT_KI 1.9e-5
#define MOTION_RIGHT_KD 0

#define MOTION_TRANSLATION_KP 5.01
#define MOTION_TRANSLATION_KI 3
#define MOTION_TRANSLATION_KD 30000

#define MOTION_ROTATION_KP 12
#define MOTION_ROTATION_KI 1e-05
#define MOTION_ROTATION_KD 10000

#else // INTECH_TARGET == 1, INTECH_TARGET == 2

#define MOTION_LEFT_KP 1e1
#define MOTION_LEFT_KD 1
#define MOTION_LEFT_KI 1e-6

#define MOTION_RIGHT_KP 1e1
#define MOTION_RIGHT_KD 1
#define MOTION_RIGHT_KI 1

#define MOTION_TRANSLATION_KP 1
#define MOTION_TRANSLATION_KD 2e-1
#define MOTION_TRANSLATION_KI 1e-1

#define MOTION_ROTATION_KP 5e-1
#define MOTION_ROTATION_KD 1e-1
#define MOTION_ROTATION_KI 1e-1

#endif // INTECH_TARGET == 1, INTECH_TARGET == 2

#define MOTION_POWER_BASE 0.015
#define MOTION_POWER_MAX  0.6
#define MOTION_STARTUP_DISTANCE 500

#define MOTION_JOYSTICK_ANGLE_OUTREACH 3
