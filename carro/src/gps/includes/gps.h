/*
 * uart.h
 *
 *  Created on: Nov 18, 2023
 *      Author: Fernando Mendoza
 */

#ifndef SRC_GPS_INCLUDES_GPS_H_
#define SRC_GPS_INCLUDES_GPS_H_
#include <stdint.h>

#define GPS_LAT_DIR_FLAG        (0x01u)
#define GPS_LON_DIR_FLAG        (0x02u)

uint8_t new_gps_data;

uint16_t lat_int;
uint32_t lat_dec;
uint16_t lon_int;
uint32_t lon_dec;
uint8_t n_sat;
uint8_t gps_flags;

void gps_init(uint16_t bitrate);

#endif /* SRC_GPS_INCLUDES_GPS_H_ */
