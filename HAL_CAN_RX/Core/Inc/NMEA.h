#ifndef __NMEA_H
#define __NMEA_H

#include "stdint.h"
#include "stdbool.h"

/* Maximum NMEA sentence length */
#define NMEA_MAX_LEN 128

/* Structure to hold parsed GPS data */
typedef struct
{
    char utc_time[11];      // HHMMSS.SS
    double latitude;        // Decimal degrees
    char lat_dir;           // N/S
    double longitude;       // Decimal degrees
    char lon_dir;           // E/W
    int fix_quality;        // 0 = invalid, 1 = GPS fix, 2 = DGPS fix
    int num_satellites;     // Number of satellites
    double hdop;            // Horizontal dilution of precision
    double altitude;        // Altitude in meters
    bool valid;             // True if sentence parsed successfully
} GPS_Data_t;

/* Function prototypes */
void NMEA_Parse(char* nmea, GPS_Data_t* data);
double NMEA_ConvertToDecimal(char* val, char dir);

#endif /* __NMEA_H */
