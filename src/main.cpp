#include <Arduino.h>
#include <BleHash.h>
#include <BleParser.h>
#include <BleSerial.h>
#include <Preferences.h>
#include <WiFi.h>
#include <WiFiClient.h>

#define RED_LED 0
#define GREEN_LED 1
#define BLUE_LED 2

const unsigned creditianMaxLength = 32;
const unsigned outputLogMaxLength = 128;
const unsigned port = 80;
char ssid[creditianMaxLength] = {0};
char pass[creditianMaxLength] = {0};
char addr[creditianMaxLength] = {0};
char outputLog[outputLogMaxLength] = {0};
String BLEName("BLE-MAC " + WiFi.macAddress());
Preferences Storage;
const char *filename = "settings";
const char *ssidKey = "ssid";
const char *passKey = "pass";
const char *addrKey = "addr";

static inline void printlog();
static void initWifiCreditian(const char *, char *);
static void changeWifiCreditian(char *);
static void saveWifiCreditian(const char *, char *);
static void connectToWiFi(char *, char *);
static void checkWifiCreditian();
static void rebootMCU();
static void temperatureMCU();

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
        static struct {
            bool CR : 1;
            bool PR : 1;
        } Flag = {.CR = false, .PR = false};
        if (pCharacteristic->getUUID().toString() == BLE_RX_UUID) {
            String value(pCharacteristic->getValue().data());
            for (unsigned counter = 0; counter < value.length(); counter++) {
                if (Flag.PR)
                    receiveBuffer.add(value[counter]);
                switch (value[counter]) {
                case '#':
                    Flag.PR = true;
                    break;
                case '\r':
                    Flag.CR = true;
                    break;
                case '\n':
                    if (Flag.CR) {
                        String string = readString();
                        parseString(string, " ,;:!?/%\n\r");
                        log_e(">> Received: %s", string.c_str());
                        Flag.PR = false;
                    }
                    [[fallthrough]];
                default:
                    Flag.CR = false;
                    break;
                }
            }
        }
    }

    void initBleHash() override {
        commandList[hash("ssid")] = []() { changeWifiCreditian(ssid); };
        commandList[hash("pass")] = []() { changeWifiCreditian(pass); };
        commandList[hash("addr")] = []() { changeWifiCreditian(addr); };
        commandList[hash("save")] = []() {
            snprintf(outputLog, outputLogMaxLength,
                     "Saved Wi-Fi creditians <\"key\": creditian>:");
            printlog();
            saveWifiCreditian(ssidKey, ssid);
            saveWifiCreditian(passKey, pass);
            saveWifiCreditian(addrKey, addr);
        };
        commandList[hash("connect")] = []() { connectToWiFi(ssid, pass); };
        commandList[hash("check")] = []() { checkWifiCreditian(); };
        commandList[hash("reboot")] = []() { rebootMCU(); };
        commandList[hash("temperature")] = []() { temperatureMCU(); };
    }
} Ble;

WiFiClient Client;

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
        const char *basicCreditian = "dummy";
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

    snprintf(outputLog, outputLogMaxLength, "\"%s\": %s.", key, creditian);
    printlog();
}

static void connectToWiFi(char *ssid, char *pass) {
    wl_status_t status = WL_IDLE_STATUS;
    unsigned times = 0;
    const unsigned timesMax = 64;

    WiFi.begin(ssid, pass);
    while ((status = WiFi.status()) != WL_CONNECTED && times++ < timesMax)
        delay(0x40);

    if (times < timesMax) {
        snprintf(outputLog, outputLogMaxLength, "Connecting to server...");
        printlog();
        if (Client.connect(addr, port)) {
            snprintf(outputLog, outputLogMaxLength,
                     "Connected to server succesfully!");
        } else {
            snprintf(outputLog, outputLogMaxLength,
                     "Failed to connect to the server.");
        }
    } else {
        snprintf(outputLog, outputLogMaxLength,
                 "Wi-Fi not connected after %u times.", timesMax);
        WiFi.disconnect(true);
    }
    printlog();
}

static void checkWifiCreditian() {
    snprintf(outputLog, outputLogMaxLength,
             "Wi-Fi creditians <ssid, pass, addr>: %s, %s, %s.", ssid, pass,
             addr);
    printlog();
}

static void rebootMCU() {
    snprintf(outputLog, outputLogMaxLength, "The device will now reboot!");
    printlog();
    delay(1000);
    esp_restart();
}

static void temperatureMCU() {
    snprintf(outputLog, outputLogMaxLength, "MCU temperature: %.1f C.",
             temperatureRead());
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
    ledcWrite(GREEN_LED, 0U);
    ledcWrite(BLUE_LED, 0U);

    Storage.begin(filename);

    if (!Ble.begin(BLEName.c_str(), true))
        log_e(">> BLE did not start!");
    else
        log_e(">> BLE started after name %s", BLEName);
    Ble.initBleHash();

    initWifiCreditian(ssidKey, ssid);
    initWifiCreditian(passKey, pass);
    initWifiCreditian(addrKey, addr);

    connectToWiFi(ssid, pass);

    Storage.end();

    snprintf(outputLog, outputLogMaxLength,
             "Initialization completed in %u ms.", millis() - timeStamp);
    printlog();
}

void loop() {
    static bool state = true;
    static struct Flag {
        bool ble : 1;
        bool wifi : 1;
    } Flag = {0};
    static uint8_t duty = Led.duty;

    if (Ble.connected()) {
        if (!Flag.ble) {
            snprintf(outputLog, outputLogMaxLength, "Bluetooth LE connected.");
            printlog();
            Flag.ble = true;
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
    } else {
        if (Flag.ble) {
            log_e(">> Bluetooth LE disconnected.");
            Flag.ble = false;
        }
    }

    if (Client.connected()) {
        if (!Flag.wifi) {
            snprintf(outputLog, outputLogMaxLength,
                     "Main loop Wi-Fi flag activated!");
            printlog();
            Flag.wifi = true;
            ledcWrite(GREEN_LED, Led.duty);
            Client.println("Привет, я клиент!");
        }

    } else {
        if (Flag.wifi) {
            snprintf(outputLog, outputLogMaxLength, "Wi-Fi disconnected.");
            printlog();
            Client.stop();
            WiFi.disconnect(true);
            Flag.wifi = false;
            ledcWrite(GREEN_LED, 0U);
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
#endif // DEBUG
    delay(10);
}
