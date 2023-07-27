#include "BleHash.h"
#include "BleParser.h"
#include "BleSerial.h"
#include <Arduino.h>
#include <ESPAsyncWebServer.h>
#include <Preferences.h>
#include <WiFi.h>

#define RED_LED 0
#define GREEN_LED 1
#define BLUE_LED 2

const unsigned creditianMaxLength = 32;
const unsigned outputLogMaxLength = 128;
char ssid[creditianMaxLength] = {0};
char passphrase[creditianMaxLength] = {0};
char outputLog[outputLogMaxLength] = {0};
AsyncWebServer server(80);
char *localIP = 0;
String BLEName = "BLE-UART";
Preferences Storage;
const char *filename = "settings";
const char *ssidKey = "ssid";
const char *passphraseKey = "passphrase";

static inline void printlog();
static void initWifiCreditian(const char *, char *);
static void changeWifiCreditian(char *);
static void saveWifiCreditian(const char *key, char *creditian);
static void connectToWiFi(char *, char *);

struct Led {
    const uint8_t red;
    const uint8_t green;
    const uint8_t blue;
    const uint8_t number;
    const uint8_t duty;
} Led = {.red = 3U, .green = 4U, .blue = 5U, .number = 3U, .duty = 32U};

class Ble : public BleParser, public BleHash {
  public:
    void onWrite(BLECharacteristic *pCharacteristic) override {
        static bool CR = false;
        if (pCharacteristic->getUUID().toString() == BLE_RX_UUID) {
            String value(pCharacteristic->getValue().data());
            for (unsigned counter = 0; counter < value.length(); counter++) {
                receiveBuffer.add(value[counter]);
                switch (value[counter]) {
                case '\r':
                    CR = true;
                    break;
                case '\n':
                    if (CR) {
                        String string = readString();
                        parseString(string, " ,.;:!?\n\r");
                        log_e(">> Received: %s", string.c_str());
                    }
                    [[fallthrough]];
                default:
                    CR = false;
                    break;
                }
            }
        }
    }

    void initBleHash() override {
        commandList[hash(ssidKey)] = []() { changeWifiCreditian(ssid); };
        commandList[hash(passphraseKey)] = []() {
            changeWifiCreditian(passphrase);
        };
        commandList[hash("connect")] = []() {
            connectToWiFi(ssid, passphrase);
        };
        commandList[hash("sleep")] = []() { log_e(">> It is sleep"); };
        commandList[hash("save")] = []() {
            saveWifiCreditian(ssidKey, ssid);
            saveWifiCreditian(passphraseKey, passphrase);
        };
    }
} Ble;

static inline void printlog() {
    if (Ble.connected())
        Ble.printf("%s\r\n", outputLog);
    log_e(">> %s", outputLog);
}

static void initWifiCreditian(const char *key, char *creditian) {
    if (Storage.isKey(key)) {
        Storage.getString(key, creditian, creditianMaxLength);
        snprintf(outputLog, outputLogMaxLength,
                 "Found creditian under key \"%s\": %s", key, creditian);
    } else {
        const char *basicCreditian = "hotspot";
        strncpy(creditian, basicCreditian, creditianMaxLength);
        Storage.putString(key, creditian);
        snprintf(outputLog, outputLogMaxLength,
                 "Created new creditian under key \"%s\": %s", key, creditian);
    }
    printlog();
}

static void changeWifiCreditian(char *creditian) {
    if (Ble.argc > Ble.it) {
        Ble.it++;
        unsigned length = strlen(Ble.argv[Ble.it]);
        if (length > creditianMaxLength) {
            snprintf(outputLog, outputLogMaxLength,
                     "Wi-Fi creditian length must be less than %u.",
                     creditianMaxLength);
        } else {
            memset(creditian, '\0', creditianMaxLength);
            strncpy(creditian, Ble.argv[Ble.it], creditianMaxLength);
            creditian[length] = '\0';
            snprintf(outputLog, outputLogMaxLength,
                     "Wi-Fi creditian %s changed to \"%s\".",
                     Ble.argv[Ble.it - 1], creditian);
        }
    } else {
        snprintf(outputLog, outputLogMaxLength,
                 "Fewer arguments than necessary in %s.", Ble.argv[Ble.it - 1]);
    }
    printlog();
}

static void saveWifiCreditian(const char *key, char *creditian) {
    Storage.begin(filename);

    if (Storage.isKey(key))
        Storage.remove(key);
    Storage.putString(key, creditian);

    snprintf(outputLog, outputLogMaxLength,
             "Creditian under key \"%s\" saved as: %s.", key, creditian);
    printlog();
}

static void connectToWiFi(char *ssid, char *passphrase) {
    unsigned times = 0;
    const unsigned timesMax = 32;

    WiFi.begin(ssid, passphrase);
    while (WiFi.status() != WL_CONNECTED && times++ < timesMax)
        delay(10);

    if (times < timesMax) {
        server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
            request->send(200, "text/plain", "Hello, World!\n");
        });

        server.begin();

        snprintf(outputLog, outputLogMaxLength, "Wi-Fi local IP: %s",
                 localIP = (char *)WiFi.localIP().toString().c_str());
        ledcWrite(GREEN_LED, Led.duty);
    } else {
        snprintf(outputLog, outputLogMaxLength,
                 "Wi-Fi not connected after %u times.", timesMax);
        WiFi.disconnect(true);
    }
    printlog();
}

void setup() {
    unsigned long timeStamp = millis();
    ledcSetup(RED_LED, 500, 8);
    ledcSetup(GREEN_LED, 500, 8);
    ledcSetup(BLUE_LED, 500, 8);

    ledcAttachPin(Led.red, RED_LED);
    ledcAttachPin(Led.green, GREEN_LED);
    ledcAttachPin(Led.blue, BLUE_LED);

    ledcWrite(RED_LED, Led.duty);

    Storage.begin(filename);

    if (!Ble.begin(BLEName.c_str(), true))
        log_e(">> BLE did not start!");
    else
        log_e(">> BLE started after name %s", BLEName);
    Ble.initBleHash();

    initWifiCreditian(ssidKey, ssid);
    initWifiCreditian(passphraseKey, passphrase);

    connectToWiFi(ssid, passphrase);

    Storage.end();

    snprintf(outputLog, outputLogMaxLength,
             "Initialization completed in %u ms.", millis() - timeStamp);
    printlog();
}

void loop() {
    static bool state = true;
    /*static uint32_t led = 0;
    static uint32_t times = 0;*/
    static bool connectFlag = false;
    static uint8_t duty = Led.duty;

    if (Ble.connected()) {
        if (!connectFlag) {
            snprintf(outputLog, outputLogMaxLength, "Bluetooth LE connected.");
            printlog();
            connectFlag = true;
        }
        if (Ble.isParsed() && !Ble.it) {
            for (; Ble.it < Ble.argc; Ble.it++) {
                Ble.argv[Ble.it] = Ble.toLower(Ble.argv[Ble.it]);
                auto it = Ble.commandList.find(Ble.hash(Ble.argv[Ble.it]));
                if (it != Ble.commandList.end())
                    it->second();
                else {
                    snprintf(outputLog, outputLogMaxLength,
                             "Unknown command received.");
                    printlog();
                }
            }
        }
        /*Ble.printf("Times: %d.\r\n", times++);*/
    } else {
        /*times = 0;*/
        if (connectFlag) {
            log_e(">> Bluetooth LE disconnected.");
            connectFlag = false;
        }
    }

#if (DEBUG == 1)
    switch (state) {
    case true:
        if (++duty == 0xFF)
            state = false;
        break;
    case false:
        if (--duty == 0x00)
            state = true;
        break;
    }
    ledcWrite(RED_LED, duty);
    delay(10);
#endif // DEBUG
}
