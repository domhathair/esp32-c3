#include "../include/crc32.h"

uint32_t crc32(uint32_t crc, const char *buffer, size_t size) {
    while (size--) {
        crc ^= (uint32_t)(*buffer++) << 24;
        crc = (crc << 4) ^ crc32Table[crc >> 28];
        crc = (crc << 4) ^ crc32Table[crc >> 28];
    }
    return crc;
}