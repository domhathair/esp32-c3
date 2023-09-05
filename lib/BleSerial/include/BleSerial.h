#ifndef BleSerial_h
#define BleSerial_h
#include "RingBuffer.h"
#include <Arduino.h>
#include <BLE2902.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>

#define BLE_BASIC_SIZE 256
#define BLE_BUFFER_SIZE                                                        \
    (BLE_BASIC_SIZE > ESP_GATT_MAX_ATTR_LEN) ? ESP_GATT_MAX_ATTR_LEN           \
                                             : BLE_BASIC_SIZE
#define BLE_MTU 20

class BleSerial : public BLECharacteristicCallbacks,
                  public BLEServerCallbacks,
                  public Stream {
  public:
    BleSerial();

    bool begin(const char *name, bool enable_led = false, int led_pin = 13);
    bool connected();
    int available() override;
    int read() override;
    int peek() override;
    size_t readBytes(uint8_t *buffer, size_t bufferSize) override;
    String readString() override;
    size_t write(uint8_t byte) override;
    size_t write(const uint8_t *buffer, size_t bufferSize) override;
    void end();
    void flush() override;
    void onWrite(BLECharacteristic *pCharacteristic) override;
    void onConnect(BLEServer *pServer) override;
    void onDisconnect(BLEServer *pServer) override;

    BLEServer *Server;
    BLEAdvertising *Advertising;
    BLEService *Service;
    BLECharacteristic *TxCharacteristic;
    BLECharacteristic *RxCharacteristic;

    bool enableLed = false;
    int ledPin = 13;

  protected:
    bool bleConnected;

    RingBuffer<BLE_BUFFER_SIZE> receiveBuffer;
    uint8_t transmitBuffer[BLE_BUFFER_SIZE];
    size_t transmitBufferLength;

    uint16_t peerMTU;
    uint16_t maxTransferSize = BLE_BUFFER_SIZE;

    const char *BLE_SS_UUID = "6e400001-b5a3-f393-e0a9-e50e24dcca9e";
    const char *BLE_RX_UUID = "6e400002-b5a3-f393-e0a9-e50e24dcca9e";
    const char *BLE_TX_UUID = "6e400003-b5a3-f393-e0a9-e50e24dcca9e";

  private:
    void compatibilityTweak();

    BleSerial(BleSerial const &other) = delete;
    void operator=(BleSerial const &other) = delete;

    bool started = false;
};

#endif // BleSerial_h
