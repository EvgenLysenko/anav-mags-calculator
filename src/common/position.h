#ifndef __POSITION_H__
#define __POSITION_H__

#include "location.h"
#include "mavlink/common/mavlink.h"

struct Position: public Location
{
    Position(): alt(0), frame(MAV_FRAME::MAV_FRAME_GLOBAL) {}
    Position(const Location& location, float altitude, MAV_FRAME frame): Location(location), alt(altitude), frame(frame) {}
    Position(double lat, double lon, float altitude, MAV_FRAME frame): Location(lat, lon), alt(altitude), frame(frame) {}
    float alt = 0;
    MAV_FRAME frame = MAV_FRAME::MAV_FRAME_GLOBAL;
};

#endif
