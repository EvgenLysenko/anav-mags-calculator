#include "location.h"
#include <cmath>

static const double a = 6378137.0000;
static const double b = 6356752.3141;
static const double PI = 3.1415926535897932384626433832795;

void Location::fwd(double lat, double lon, double azimuth, double distance, double* lat2, double* lon2)
{
    if (distance == 0) {
        *lat2 = lat;
        *lon2 = lon;
        return;
    }

    double e1 = sqrt((a * a - b * b) / (a * a));
    double e2 = sqrt((a * a - b * b) / (b * b));

    double w1 = sqrt(1.0 - pow(e1, 2) * sin(lat) * sin(lat));
    double sin_u1 = (sin(lat) * sqrt(1.0 - pow(e1, 2.))) / w1;
    double cos_u1 = cos(lat) / w1;
    double sin_A0 = cos_u1 * sin(azimuth);
    int inSa0 = (int)(sin_A0);

    double A0 = 0;
    switch (inSa0)
    {
    case 1: A0 = PI / 2.; break;
    case -1: A0 = -1. * PI / 2.; break;
    default: A0 = asin(sin_A0);
    }

    double cooa0 = pow(cos(A0), 2);
    double si1 = (sin_u1 == 0) ? 0 : atan(sin_u1 / (cos_u1 * cos(distance)));
    double k = e2 * cos(A0);
    double AAAA = b * (1.0 + pow(k, 2.0) / 4.0 - 3.0 * pow(k, 4.0) / 64.0 + 5.0 * pow(k, 6.0) / 256.0);
    double BBBB = b * ((k * k) / 8.0 - pow(k, 4.0) / 32.0 + (15.0 * pow(k, 6.0)) / 1024.0);
    double CCCC = b * (pow(k, 4.0) / 128.0 - (3 * pow(k, 6.0)) / 512.0);
    double si0 = (distance - (BBBB + CCCC * cos(2.0 * si1)) * sin(2.0 * si1)) / AAAA;
    double A_s = sin(2.0 * si1) * cos(2.0 * si0) + cos(2.0 * si1) * sin(2.0 * si0);
    double C_s = cos(2.0 * si1) * cos(2.0 * si0) - sin(2.0 * si1) * sin(2.0 * si0);
    double si = si0 + (BBBB + 5.0 * CCCC * C_s) * A_s / AAAA;
    double alfa = (pow(e1, 2.0) / 2.0 + pow(e1, 4.0) / 8.0 + pow(e1, 6.0) / 16.0) -
        (pow(e1, 4.0) / 16.0 + pow(e1, 6.0) / 16.0) * cooa0 +
        (3 * pow(e1, 6.0) / 128.0) * pow(cooa0, 2.0);
    double bet = (pow(e1, 4.0) / 32.0 + pow(e1, 6.0) / 32.0) * cooa0 - pow(e1, 6.0) * pow(cooa0, 2.0) / 64.0;
    double ga = (alfa * si + bet * (sin(2.0 * (si1 + si0)) - sin(2.0 * si1))) * sin(A0);
    double sin_u2 = sin_u1 * cos(si) + cos_u1 * cos(distance) * sin(si);
    //	latitude2 = asin(sin_u2)*180/PI;
    *lat2 = atan(sin_u2 / (sqrt(1.0 - pow(e1, 2.0)) * sqrt(1.0 - pow(sin_u2, 2.0))));
    //	double ssi=sin(si);

    double la1 = sin(azimuth) * sin(si) / (cos_u1 * cos(si) - sin_u1 * sin(si) * cos(azimuth));
    double la = atan(la1);

    if ((sin(azimuth) > 0) && (la1 > 0)) la = fabs(la);
    if ((sin(azimuth) > 0) && (la1 < 0)) la = PI - fabs(la);
    if ((sin(azimuth) < 0) && (la1 < 0)) la = -1 * fabs(la);
    if ((sin(azimuth) < 0) && (la1 > 0)) la = fabs(la) - PI;

    *lon2 = lon + la - ga;
}

void Location::inverse(double la1, double lo1, double la2, double lo2, double* azimuth, double* distance)
{
    *distance = 0;
    *azimuth = 0;

    if (la1 == la2 && lo1 == lo2)
        return;

    double e1 = sqrt((a * a - b * b) / (a * a));
    double e2 = sqrt((a * a - b * b) / (b * b));
    //	double e2 = Math.sqrt((a*a-b*b)/(a*a)); //original src ?????

    double w1 = sqrt(1 - e1 * e1 * sin(la1) * sin(la1));
    double w2 = sqrt(1 - e1 * e1 * sin(la2) * sin(la2));
    double su1 = (sin(la1) * sqrt(1 - e1 * e1)) / w1;
    double su2 = (sin(la2) * sqrt(1 - e1 * e1)) / w2;
    double cu1 = cos(la1) / w1;
    double cu2 = cos(la2) / w2;
    double l = lo2 - lo1;
    double ga = 0;
    double a11 = su1 * su2;
    double a12 = cu1 * cu2;
    double b11 = cu1 * su2;
    double b12 = su1 * cu2;

    double ugol = 0, a0 = 0, si = 0, x = 0;
    for (int i = 0; i < 4; ++i)
    {
        double la = l + ga;
        double p = cu2 * sin(la);
        double q = b11 - b12 * cos(la);
        //		if (q == 0) a1 = 0;
        double a10 = atan(p / q);

        if ((p > 0) && (q > 0)) ugol = fabs(a10);
        if ((p == 0) && (q > 0)) ugol = fabs(a10);
        if ((p > 0) && (q < 0)) ugol = PI - fabs(a10);
        if ((p == 0) && (q < 0)) ugol = PI - fabs(a10);
        if ((p < 0) && (q < 0)) ugol = PI + fabs(a10);
        if ((p < 0) && (q > 0)) ugol = 2. * PI - fabs(a10);

        double ssi = p * sin(ugol) + q * cos(ugol);
        double csi = a11 + a12 * cos(la);
        double si1 = atan(ssi / csi);

        if (si1 == 0) si = 0;
        if (si1 > 0) si = fabs(si1);
        if (si1 < 0) si = PI - fabs(si1);

        double tempVal = (cu1 * sin(ugol)) / (sqrt(1. - (cu1 * sin(ugol)) * (cu1 * sin(ugol))));
        a0 = atan(tempVal);
        x = 2.0 * a11 - cos(a0) * cos(a0) * cos(si);
        double alfa = ((e1 * e1) / 2.0 + pow(e1, 4.0) / 8.0 + pow(e1, 4.0) / 16.0) -
            (pow(e1, 4.0) / 16.0 + pow(e1, 6.0) / 16.0) * cos(a0) * cos(a0) +
            ((3.0 * pow(e1, 6.0)) / 128.0) * cos(a0) * cos(a0) * cos(a0) * cos(a0);
        double bet = (28189.0 - 94.0 * cos(a0) * cos(a0)) * 0.0000000001;
        ga = (alfa * si - bet * x * ssi) * sin(a0);
    }

    double k = e2 * cos(a0);
    double AAAA = b * (1.0 + (k * k) / 4.0 - (3.0 * k * k * k * k) / 64.0 + (5. * k * k * k * k * k * k) / 256.);
    double BBBB = b * e2 * e2 / 4.0 - b * (e2 * e2 * e2 * e2 / 32.0) * cos(a0) * cos(a0);
    double CCCC = b * ((k * k * k * k) / 128. - (3. * k * k * k * k * k * k) / 512.);
    double y = (cos(a0) * cos(a0) * cos(a0) * cos(a0) - 2. * x * x) * cos(si);

    *distance = AAAA * si + (BBBB * x + CCCC * y) * sin(si);

    if (ugol < 0)
        ugol += 2. * PI;
    else
        if (ugol > 2. * PI) ugol -= 2. * PI;

    *azimuth = ugol;
}
