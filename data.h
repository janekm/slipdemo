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
    uint8_t accelX;
    uint8_t accelY;
    uint8_t accelZ;
    uint8_t magX;
    uint8_t magY;
    uint8_t magZ;
    uint8_t light_mant;
    uint8_t light_exp;
} __attribute__((packed)) sample_t;

#define PACKET_SIZE 32 // sizeof(sample_t)  (For the moment, code makes assumptions about size of packet, so keep at 32)

#endif
