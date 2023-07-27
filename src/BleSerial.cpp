#include "BleSerial.h"

BleSerial::BleSerial() {}

bool BleSerial::begin(const char *name, bool enable_led, int led_pin) {
    enableLed = enable_led;
    ledPin = led_pin;

    if (enableLed)
        pinMode(ledPin, OUTPUT);

    BLEDevice::init(name);

    Server = BLEDevice::createServer();
    if (!Server) {
        return false;
        log_e(">> Bluetooth LE Server error.");
    }
    Server->setCallbacks(this);

    SerialService = Server->createService(BLE_SS_UUID);
    if (!SerialService) {
        return false;
        log_e(">> Bluetooth LE Serial service error.");
    }

    RxCharacteristic = SerialService->createCharacteristic(
        BLE_RX_UUID, BLECharacteristic::PROPERTY_WRITE);
    if (!RxCharacteristic) {
        return false;
        log_e(">> Bluetooth LE RxCharacteristic error.");
    }

    TxCharacteristic = SerialService->createCharacteristic(
        BLE_TX_UUID, BLECharacteristic::PROPERTY_NOTIFY);
    if (!TxCharacteristic) {
        return false;
        log_e(">> Bluetooth LE TxCharacteristic error.");
    }

    TxCharacteristic->setAccessPermissions(ESP_GATT_PERM_READ_ENCRYPTED);
    RxCharacteristic->setAccessPermissions(ESP_GATT_PERM_WRITE_ENCRYPTED);

    TxCharacteristic->addDescriptor(new BLE2902());
    RxCharacteristic->addDescriptor(new BLE2902());

    TxCharacteristic->setReadProperty(true);
    RxCharacteristic->setWriteProperty(true);
    RxCharacteristic->setCallbacks(this);
    SerialService->start();

    Advertising = BLEDevice::getAdvertising();
    if (!Advertising) {
        return false;
        log_e(">> Bluetooth LE Advertising error.");
    }
    Advertising->addServiceUUID(BLE_SS_UUID);
    Advertising->setScanResponse(true);
    Advertising->setMinPreferred(0x06);
    Advertising->setMinPreferred(0x12);
    Advertising->start();

    return true;
}

bool BleSerial::connected() { return Server->getConnectedCount() > 0; }

int BleSerial::available() { return this->receiveBuffer.getLength(); }

int BleSerial::read() {
    if (available())
        return this->receiveBuffer.pop();
    return -1;
}

int BleSerial::peek() {
    if (!available())
        return -1;
    return this->receiveBuffer.get(0);
}

size_t BleSerial::readBytes(uint8_t *buffer, size_t bufferSize) {
    unsigned counter = 0;
    while (counter < bufferSize)
        buffer[counter++] = read();
    return counter;
}

size_t BleSerial::write(uint8_t byte) {
    if (Server->getConnectedCount() < 1)
        return 0;
    this->transmitBuffer[this->transmitBufferLength++] = byte;
    if (this->transmitBufferLength == maxTransferSize)
        flush();
    return 1;
}

size_t BleSerial::write(const uint8_t *buffer, size_t bufferSize) {
    if (Server->getConnectedCount() < 1)
        return 0;

    if (maxTransferSize < MIN_MTU) {
        int oldTransferSize = maxTransferSize;
        peerMTU = Server->getPeerMTU(Server->getConnId()) - 5;
        maxTransferSize = peerMTU > BLE_BUFFER_SIZE ? BLE_BUFFER_SIZE : peerMTU;

        if (maxTransferSize != oldTransferSize)
            log_e(">> Max BLE transfer size set to %u", maxTransferSize);
    }

    if (maxTransferSize < MIN_MTU)
        return 0;

    size_t written = 0;
    for (unsigned counter = 0; counter < bufferSize; counter++)
        written += this->write(buffer[counter]);
    flush();
    return written;
}

size_t BleSerial::print(const char *str) {
    if (Server->getConnectedCount() < 1)
        return 0;
    size_t written = 0;
    for (unsigned counter = 0; str[counter] != '\0'; counter++)
        written += this->write(str[counter]);
    flush();
    return written;
}

void BleSerial::end() { BLEDevice::deinit(); }

void BleSerial::flush() {
    if (this->transmitBufferLength) {
        TxCharacteristic->setValue(this->transmitBuffer,
                                   this->transmitBufferLength);
        this->transmitBufferLength = 0;
    }
    TxCharacteristic->notify(true);
}

void BleSerial::onWrite(BLECharacteristic *pCharacteristic) {
    if (pCharacteristic->getUUID().toString() == BLE_RX_UUID) {
        String value(pCharacteristic->getValue().data());
        for (unsigned counter = 0; counter < value.length(); counter++)
            receiveBuffer.add(value[counter]);
    }
}

void BleSerial::onConnect(BLEServer *pServer) {
    bleConnected = true;
    if (enableLed)
        digitalWrite(ledPin, HIGH);
}

void BleSerial::onDisconnect(BLEServer *pServer) {
    bleConnected = false;
    if (enableLed)
        digitalWrite(ledPin, LOW);
    Server->startAdvertising();
}
