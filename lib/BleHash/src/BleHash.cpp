#include "BleHash.h"

//unsigned BleHash::hash(const char *string, int h) {
//    return !string[h] ? 5381 : (hash(string, h + 1) * 33) ^ string[h];
//}

char *BleHash::toLower(const char *string) {
    unsigned length = strlen(string);
    char *result = new char[length + 1];

    for (unsigned counter = 0; counter < length; counter++)
        result[counter] = tolower(string[counter]);
    result[length] = '\0';

    return result;
}

void BleHash::initBleHash() {}