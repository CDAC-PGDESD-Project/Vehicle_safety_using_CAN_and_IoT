#include "NMEA.h"
#include <string.h>
#include <stdlib.h>
#include <stdio.h>

/* Convert NMEA coordinate (DDMM.MMMM) to decimal degrees */
double NMEA_ConvertToDecimal(char* val, char dir)
{
    if (val == NULL || strlen(val) < 3)
        return 0.0;

    double d = atof(val);
    int degrees = (int)(d / 100);
    double minutes = d - (degrees * 100);
    double decimal = degrees + (minutes / 60.0);

    if (dir == 'S' || dir == 'W') decimal *= -1.0;

    return decimal;
}

/* Parse NMEA sentence (GPGGA) */
void NMEA_Parse(char* nmea, GPS_Data_t* data)
{
    if (!nmea || !data) return;

    memset(data, 0, sizeof(GPS_Data_t));
    data->valid = false;

    /* Only parse GPGGA sentences */
    if (strncmp(nmea, "$GPGGA", 6) != 0)
        return;

    char* token;
    char* rest = nmea;
    int field = 0;

    while ((token = strtok_r(rest, ",", &rest)))
    {
        switch (field)
        {
            case 1: // UTC Time
                strncpy(data->utc_time, token, sizeof(data->utc_time)-1);
                break;
            case 2: // Latitude
                data->latitude = atof(token); // temporarily store raw
                break;
            case 3: // N/S
                data->lat_dir = token[0];
                data->latitude = NMEA_ConvertToDecimal(token, data->lat_dir);
                break;
            case 4: // Longitude
                data->longitude = atof(token); // temporarily store raw
                break;
            case 5: // E/W
                data->lon_dir = token[0];
                data->longitude = NMEA_ConvertToDecimal(token, data->lon_dir);
                break;
            case 6: // Fix Quality
                data->fix_quality = atoi(token);
                break;
            case 7: // Number of Satellites
                data->num_satellites = atoi(token);
                break;
            case 8: // HDOP
                data->hdop = atof(token);
                break;
            case 9: // Altitude
                data->altitude = atof(token);
                break;
        }
        field++;
    }

    /* Mark as valid if fix_quality > 0 */
    if (data->fix_quality > 0) data->valid = true;
}
