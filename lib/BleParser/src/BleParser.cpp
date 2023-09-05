#include "BleParser.h"

unsigned BleParser::parseString(String string, const char *delim) {
#if (DEBUG == 1)
    unsigned long timeStamp = micros();
#endif // DEBUG

    clear();

    unsigned length = string.length();
    argBasicPtr = new char[length + 1];
    if (!argBasicPtr) {
        log_e(">> Allocation error!");
        return 0;
    }
    memcpy(argBasicPtr, string.c_str(), length);
    argBasicPtr[length] = '\0';
    char *argPtr = argBasicPtr;
    argPtr = strtok(argPtr, delim);

    while (argPtr) {
        argVector.push_back(argPtr);
        argPtr = strtok(NULL, delim);
    }
    argc = argVector.size();
    argv = argVector.data();

#if (DEBUG == 1)
    log_e(">> Data parsed into %u parts in %u us:", argc, micros() - timeStamp);
    for (unsigned counter = 0; counter < argc; counter++)
        log_e(">> %u: %s", counter, argv[counter]);
    log_e(">> Free heap: %u", ESP.getFreeHeap());
#endif // DEBUG
    flag = true;

    return argc;
}

bool BleParser::isParsed() { return flag; }

void BleParser::clear() {
    delete[] argBasicPtr;
    argVector.clear();
    it = 0;
    argc = 0;
    argv = NULL;
    flag = false;
}
