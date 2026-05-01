#ifndef __LOCATION_H__
#define __LOCATION_H__

struct Location
{
    Location() {}
    Location(double lat, double lon): lat(lat), lon(lon) {}
    double lat = 0;
    double lon = 0;

    void inverse(const Location& loc, double* azimuth, double* distance) const {
        inverse(lat, lon, loc.lat, loc.lon, azimuth, distance);
    }

    void move(double azimuth, double distance) { fwd(lat, lon, azimuth, distance, &lat, &lon); }
    Location fwd(double azimuth, double distance) const {
        Location location;
        fwd(lat, lon, azimuth, distance, &location.lat, &location.lon);
        return  location;
    }

    static void fwd(double lat1, double lon1, double azimuth, double distance, double* lat2, double* lon2);
    static void inverse(double lat1, double lon1, double lat2, double lon2, double* azimuth, double* distance);
};

#endif
