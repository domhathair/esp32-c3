#ifndef BleParser_h
#define BleParser_h
#include "BleSerial.h"
#include <Arduino.h>
#include <cstring>

using namespace std;

class BleParser : public BleSerial {
  public:
    unsigned it;
    unsigned argc;
    char **argv;
    unsigned
    parseString(String &,
                const char * = " ,.;:!?-\"()[]{}<>@#$%^&*_+=~`|\\/\n\r");

    bool isParsed();

  private:
    void operator=(BleParser const &other) = delete;
    void clear();

    char *argBasicPtr;
    vector<char *> argVector;
    bool flag = false;
};

#endif // BleParser_h