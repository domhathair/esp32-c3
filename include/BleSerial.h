#ifndef BleSerial_h
#define BleSerial_h
#include <Arduino.h>
#include "ByteRingBuffer.h"
#include <BLE2902.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>

#define BLE_BUFFER_SIZE                                                        \
    ESP_GATT_MAX_ATTR_LEN // must be greater than MTU, less than
                          // ESP_GATT_MAX_ATTR_LEN
#define MIN_MTU 50

class BleSerial : public BLECharacteristicCallbacks,
                  public BLEServerCallbacks,
                  public Stream {
  public:
    BleSerial();

    bool begin(const char *name, bool enable_led = false, int led_pin = 13);
    bool connected();
    int available();
    int read();
    int peek();
    size_t readBytes(uint8_t *buffer, size_t bufferSize);
    size_t write(uint8_t byte);
    size_t write(const uint8_t *buffer, size_t bufferSize);
    size_t print(const char *value);
    void end();
    void flush();
    void onWrite(BLECharacteristic *pCharacteristic);
    void onConnect(BLEServer *pServer);
    void onDisconnect(BLEServer *pServer);

    BLEServer *Server;

    BLEAdvertising *Advertising;
    // BLESecurity *Security;

    BLEService *SerialService;
    BLECharacteristic *TxCharacteristic;
    BLECharacteristic *RxCharacteristic;

    bool enableLed = false;
    int ledPin = 13;

  protected:
    bool bleConnected;

    ByteRingBuffer<BLE_BUFFER_SIZE> receiveBuffer;
    uint8_t transmitBuffer[BLE_BUFFER_SIZE];
    size_t transmitBufferLength;

    uint16_t peerMTU;
    uint16_t maxTransferSize = BLE_BUFFER_SIZE;

    const char *BLE_SS_UUID = "6e400001-b5a3-f393-e0a9-e50e24dcca9e";
    const char *BLE_RX_UUID = "6e400002-b5a3-f393-e0a9-e50e24dcca9e";
    const char *BLE_TX_UUID = "6e400003-b5a3-f393-e0a9-e50e24dcca9e";

  private:
    BleSerial(BleSerial const &other) = delete;
    void operator=(BleSerial const &other) = delete;

    bool started = false;
};

#endif // BleSerial_h
