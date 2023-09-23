#ifndef BleHash_h
#define BleHash_h
#include <Arduino.h>
#include <cctype>
#include <cstring>
#include <map>

class BleHash {
  public:
    static unsigned hash(const char *, int = 0);
    char *toLower(const char *);

    virtual void initBleHash();
    std::map<unsigned, void (*)()> commandList;

  private:
    void operator=(BleHash const &other) = delete;
};

#endif // BleHash_h