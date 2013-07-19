#ifndef DATA_H
#define DATA_H

typedef struct {
    uint32_t secSinceMidnight;
    uint32_t proxyTick;
} __attribute__((packed)) gps_sync_t;

typedef struct {
    uint8_t mantissa;
    uint8_t exponent;
} __attribute__((packed)) light_t;

typedef struct {
    int8_t x;
    int8_t y;
    int8_t z;
} __attribute__((packed)) xyz8_data_t;

typedef struct {
    uint16_t year;
    uint8_t nodeId;
    uint8_t month;
    uint8_t day;
    uint8_t hour;
    uint8_t minute;
    uint8_t second;
    int32_t latitude;
    int32_t longitude;
    uint8_t nsats;
    xyz8_data_t accel;
    xyz8_data_t mag;
    light_t light;
} __attribute__((packed)) sample_t;

#endif
