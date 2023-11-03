#ifndef BleHash_h
#define BleHash_h
#include <Arduino.h>
#include <cctype>
#include <cstring>
#include <map>

class BleHash {
  public:
    constexpr unsigned hash(const char *string, int h = 0) {
        return !string[h] ? 5381 : (hash(string, h + 1) * 33) ^ string[h];
    };
    char *toLower(const char *);

    virtual void initBleHash();
    std::map<unsigned, void (*)()> commandList;

  private:
    void operator=(BleHash const &other) = delete;
};

#endif // BleHash_h