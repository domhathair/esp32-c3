#include "BleSerial.h"

BleSerial::BleSerial() {}

void BleSerial::compatibilityTweak() {
    esp_ble_auth_req_t auth_req = ESP_LE_AUTH_REQ_SC_MITM_BOND;
    esp_ble_io_cap_t iocap = ESP_IO_CAP_NONE;
    uint8_t key_size = 16;
    uint8_t init_key = ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK;
    uint8_t rsp_key = ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK;
    uint32_t passkey = 123456789;
    uint8_t auth_option = ESP_BLE_ONLY_ACCEPT_SPECIFIED_AUTH_DISABLE;
    uint8_t oob_support = ESP_BLE_OOB_DISABLE;
    uint32_t dev_num = esp_ble_get_bond_device_num();
    esp_ble_bond_dev_t *dev_list =
        (esp_ble_bond_dev_t *)calloc(dev_num, sizeof(esp_ble_bond_dev_t));
    esp_ble_gap_set_security_param(ESP_BLE_SM_SET_STATIC_PASSKEY, &passkey,
                                   sizeof(uint32_t));
    esp_ble_gap_set_security_param(ESP_BLE_SM_AUTHEN_REQ_MODE, &auth_req,
                                   sizeof(uint8_t));
    esp_ble_gap_set_security_param(ESP_BLE_SM_IOCAP_MODE, &iocap,
                                   sizeof(uint8_t));
    esp_ble_gap_set_security_param(ESP_BLE_SM_MAX_KEY_SIZE, &key_size,
                                   sizeof(uint8_t));
    esp_ble_gap_set_security_param(ESP_BLE_SM_ONLY_ACCEPT_SPECIFIED_SEC_AUTH,
                                   &auth_option, sizeof(uint8_t));
    esp_ble_gap_set_security_param(ESP_BLE_SM_OOB_SUPPORT, &oob_support,
                                   sizeof(uint8_t));
    esp_ble_gap_set_security_param(ESP_BLE_SM_SET_INIT_KEY, &init_key,
                                   sizeof(uint8_t));
    esp_ble_gap_set_security_param(ESP_BLE_SM_SET_RSP_KEY, &rsp_key,
                                   sizeof(uint8_t));

    if (esp_ble_get_bond_device_list((int *)&dev_num, dev_list) != ESP_OK)
        log_e(">> Bluetooth LE device list error.");
    else {
        log_e(">> Bluetooth LE device list:");
        for (unsigned counter = 0; counter < dev_num; counter++) {
            log_e("\t dev_list[%d].bd_addr: %x:%x:%x:%x:%x:%x.", counter,
                  dev_list[counter].bd_addr[0], dev_list[counter].bd_addr[1],
                  dev_list[counter].bd_addr[2], dev_list[counter].bd_addr[3],
                  dev_list[counter].bd_addr[4], dev_list[counter].bd_addr[5]);
            if (esp_ble_remove_bond_device(dev_list[counter].bd_addr) != ESP_OK)
                log_e(">> Bluetooth LE device removing error.");
            else
                log_e(">> Bluetooth LE device removed.");
        }
    }
    free(dev_list);
}

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

    Service = Server->createService(BLEUUID(BLE_SS_UUID), 64UL, 0UL);
    if (!Service) {
        return false;
        log_e(">> Bluetooth LE Serial service error.");
    }

    TxCharacteristic = Service->createCharacteristic(
        BLE_TX_UUID, BLECharacteristic::PROPERTY_NOTIFY);
    if (!TxCharacteristic) {
        return false;
        log_e(">> Bluetooth LE TxCharacteristic error.");
    } else
        log_e(">> Added Transmit Characteristic: %s, handle: %x",
              TxCharacteristic->getUUID().toString().c_str(),
              TxCharacteristic->getHandle());
    TxCharacteristic->addDescriptor(new BLE2902());

    RxCharacteristic = Service->createCharacteristic(
        BLE_RX_UUID, BLECharacteristic::PROPERTY_WRITE);
    if (!RxCharacteristic) {
        return false;
        log_e(">> Bluetooth LE RxCharacteristic error.");
    } else
        log_e(">> Added Receive Characteristic: %s, handle: %x",
              RxCharacteristic->getUUID().toString().c_str(),
              RxCharacteristic->getHandle());
    RxCharacteristic->setCallbacks(this);

    // compatibilityTweak();

    if (maxTransferSize < BLE_MTU) {
        uint16_t oldTransferSize = maxTransferSize;
        peerMTU = Server->getPeerMTU(Server->getConnId()) - 5U;
        maxTransferSize = peerMTU > BLE_BUFFER_SIZE ? BLE_BUFFER_SIZE : peerMTU;

        if (maxTransferSize != oldTransferSize)
            log_e(">> Max BLE transfer size set to %u", maxTransferSize);
    }

    Service->start();

    Advertising = Server->getAdvertising();
    if (!Advertising) {
        return false;
        log_e(">> Bluetooth LE Advertising error.");
    }
    Advertising->addServiceUUID(BLEUUID(BLE_SS_UUID));
    Advertising->start();

    return true;
}

bool BleSerial::connected() { return Server->getConnectedCount() > 0; }

int BleSerial::available() { return this->receiveBuffer.getLength(); }

int BleSerial::read() {
    if (available())
        return this->receiveBuffer.pop();
    return 0;
}

int BleSerial::peek() {
    if (!available())
        return 0;
    return this->receiveBuffer.get(0);
}

size_t BleSerial::readBytes(uint8_t *buffer, size_t bufferSize) {
    unsigned counter = 0;
    for (uint8_t byte = '\0';
         counter < bufferSize && (byte = (read() & 0xFF)) > 0;)
        buffer[counter++] = byte;
    return counter;
}

String BleSerial::readString() {
    String string;
    for (char byte = '\0'; (byte = (read() & 0xFF)) > 0;)
        string += byte;
    return string;
}

size_t BleSerial::write(uint8_t byte) {
    if (Server->getConnectedCount() < 1)
        return 0;
    this->transmitBuffer[this->transmitBufferLength++] = byte;
    if (this->transmitBufferLength == maxTransferSize /* ||
        this->transmitBufferLength == BLE_MTU */)
        flush();
    return 1;
}

size_t BleSerial::write(const uint8_t *buffer, size_t bufferSize) {
    if (Server->getConnectedCount() < 1)
        return 0;

    size_t written = 0;
    for (unsigned counter = 0; counter < bufferSize; counter++)
        written += this->write(buffer[counter]);
    flush();
    return written;
}

void BleSerial::end() { BLEDevice::deinit(true); }

void BleSerial::flush() {
    if (this->transmitBufferLength) {
        TxCharacteristic->setValue(this->transmitBuffer,
                                   this->transmitBufferLength);
        this->transmitBufferLength = 0;
        TxCharacteristic->notify(true);
    }
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
