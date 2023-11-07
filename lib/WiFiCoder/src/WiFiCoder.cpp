#include "WiFiCoder.h"

uint32_t WiFiCoder::codeString(const char *string) {
    const uint32_t crc = 0xFFFFFFFF;

    return crc32(crc, string, strlen(string));
}

char *WiFiCoder::codeStringAsString(const char *string) {
    uint32_t crc = codeString(string);
    unsigned length;
    char *value = asnprintf(NULL, &length, "%0*X", sizeof(uint32_t) * 2U, crc);
    return value;
}

uint32_t WiFiCoder::crc32(uint32_t crc, const char *buffer, size_t size) {
    while (size--) {
        crc ^= (uint32_t)(*buffer++) << 24;
        crc = (crc << 4) ^ crc32Table[crc >> 28];
        crc = (crc << 4) ^ crc32Table[crc >> 28];
    }
    return crc;
}