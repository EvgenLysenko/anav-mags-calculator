#ifndef __MAV_UTILS_H__
#define __MAV_UTILS_H__

#include "mavlink/common/mavlink.h"
#include "mavlink/mavlink_types.h"
#include <cmath>

namespace MavUtils {
    typedef mavlink_mission_item_int_t MissionItem;    
    const float DEGE7_TO_DEG = 0.0000001;
    const float DEG_TO_DEGE7 = 10000000;
    const double DEG_TO_RAD = 0.0174532925199433;
    const double RAD_TO_DEG = 57.29577951308231 ;
    
    static double get_lat_deg (const MissionItem& mission_item) { return mission_item.x * DEGE7_TO_DEG; }
    static double get_lon_deg (const MissionItem& mission_item) { return mission_item.y * DEGE7_TO_DEG; }
    static double get_alt (const MissionItem& mission_item) { return mission_item.z; }
    static double get_lat (const MissionItem& mission_item) { return mission_item.x * DEGE7_TO_DEG * DEG_TO_RAD; }
    static double get_lon (const MissionItem& mission_item) { return mission_item.y * DEGE7_TO_DEG * DEG_TO_RAD; }

    static void set_lat (MissionItem& mission_item, double lat_rad) {
        mission_item.x = (int32_t)(lat_rad * RAD_TO_DEG * DEG_TO_DEGE7);
    }
    static void set_lon (MissionItem& mission_item, double lon_rad) {
        mission_item.y = (int32_t)(lon_rad * RAD_TO_DEG * DEG_TO_DEGE7);
    }
    static void set_lat_deg (MissionItem& mission_item, double lat_deg) {
        mission_item.x = (int32_t)(lat_deg * DEG_TO_DEGE7);
    }
    static void set_lon_deg (MissionItem& mission_item, double lon_deg) {
        mission_item.y = (int32_t)(lon_deg * DEG_TO_DEGE7);
    }

    static double rad_to_deg (double rad) { return rad * RAD_TO_DEG; }
    static double deg_to_rad (double deg) { return deg * DEG_TO_RAD; }

    static int32_t deg_to_int (double deg) { return deg * DEG_TO_DEGE7; }
    static double int_to_deg (int32_t deg_int) { return deg_int * DEGE7_TO_DEG; }
}

#endif
