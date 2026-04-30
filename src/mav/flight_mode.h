#ifndef __FLIGHT_MODE_H__
#define __FLIGHT_MODE_H__

enum FlightMode {
    Manual          = 0,
    CIRCLE          = 1,
    STABILIZE       = 2,
    TRAINING        = 3,
    ACRO            = 4,
    FBWA            = 5,
    FBWB            = 6,
    CRUISE          = 7,
    AUTOTUNE        = 8,
    Auto            = 10,
    RTL             = 11,
    Loiter          = 12,
    TAKEOFF         = 13,
    AVOID_ADSB      = 14,
    Guided          = 15,
    QSTABILIZE      = 17,
    QHOVER          = 18,
    QLOITER         = 19,
    QLAND           = 20,
    QRTL            = 21,
    QAUTOTUNE       = 22,
    QACRO           = 23,
    THERMAL         = 24,
    Loiter_to_QLand = 25,
};

#endif
