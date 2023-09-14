#include "WiFiCoder.h"

uint32_t WiFiCoder::codeString(String string) {
    const uint32_t crc = 0xFFFFFFFF;

    return crc32(crc, string.c_str(), string.length());
}

String WiFiCoder::codeStringWithAppend(String string) {
    String local = string;
    String delimiter = "::";
    uint32_t crc = codeString(string);
    char crcHex[9] = {0};
    snprintf(crcHex, 9, "%X", crc);

    return local + delimiter + crcHex;
}

uint32_t WiFiCoder::crc32(uint32_t crc, const char *buffer, size_t size) {
    while (size--) {
        crc ^= (uint32_t)(*buffer++) << 24;
        crc = (crc << 4) ^ crc32Table[crc >> 28];
        crc = (crc << 4) ^ crc32Table[crc >> 28];
    }
    return crc;
}