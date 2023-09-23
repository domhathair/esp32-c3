#include <Arduino.h>
#include <BleHash.h>
#include <BleParser.h>
#include <BleSerial.h>
#include <Preferences.h>
#include <WiFi.h>
#include <WiFiClient.h>
#include <WiFiCoder.h>

#define RED_LED 0
#define GREEN_LED 1
#define BLUE_LED 2

const unsigned creditianLength = 32;
const unsigned outputLogLength = 128;
const unsigned port = 80;
char ssid[creditianLength] = {0};
char pass[creditianLength] = {0};
char addr[creditianLength] = {0};
char outputLog[outputLogLength] = {0};
String BLEName("BLE-MAC::" + WiFi.macAddress());
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
static void disconnectFromWiFi(bool);
static void checkWifiCreditian();
static void rebootMCU();
static void deepSleepMCU();
static void temperatureMCU();

struct Led {
    const uint8_t red;
    const uint8_t green;
    const uint8_t blue;
    const uint8_t number;
    const uint8_t duty;
} Led = {.red = 3U, .green = 4U, .blue = 5U, .number = 3U, .duty = 30U};

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
            snprintf(outputLog, outputLogLength,
                     "Saved Wi-Fi creditians <key: creditian>:");
            printlog();
            saveWifiCreditian(ssidKey, ssid);
            saveWifiCreditian(passKey, pass);
            saveWifiCreditian(addrKey, addr);
        };
        commandList[hash("connect")] = []() { connectToWiFi(ssid, pass); };
        commandList[hash("disconnect")] = []() { disconnectFromWiFi(true); };
        commandList[hash("check")] = []() { checkWifiCreditian(); };
        commandList[hash("reboot")] = []() { rebootMCU(); };
        commandList[hash("sleep")] = []() { deepSleepMCU(); };
        commandList[hash("temperature")] = []() { temperatureMCU(); };
    }
} Ble;

WiFiClient Client;

WiFiCoder Coder;

static inline void printlog() {
    if (Ble.connected())
        Ble.printf("%s\r\n", outputLog);
    log_e(">> %s", outputLog);
}

static void initWifiCreditian(const char *key, char *creditian) {
    if (Storage.isKey(key)) {
        Storage.getString(key, creditian, creditianLength);
        snprintf(outputLog, outputLogLength,
                 "Found creditian under key <%s>: %s", key, creditian);
    } else {
        const char *basicCreditian = "dummy";
        strncpy(creditian, basicCreditian, creditianLength);
        Storage.putString(key, creditian);
        snprintf(outputLog, outputLogLength,
                 "Created new creditian under key <%s>: %s", key, creditian);
    }
    printlog();
}

static void changeWifiCreditian(char *creditian) {
    if (Ble.argc > Ble.it) {
        Ble.it++;
        unsigned length = strlen(Ble.argv[Ble.it]);
        if (length > creditianLength) {
            snprintf(outputLog, outputLogLength,
                     "Wi-Fi creditian length must be less than %u.",
                     creditianLength);
        } else {
            memset(creditian, '\0', creditianLength);
            strncpy(creditian, Ble.argv[Ble.it], creditianLength);
            creditian[length] = '\0';
            snprintf(outputLog, outputLogLength,
                     "Wi-Fi creditian <%s> changed to \"%s\".",
                     Ble.argv[Ble.it - 1], creditian);
        }
    } else {
        snprintf(outputLog, outputLogLength,
                 "Fewer arguments than necessary in <%s>.",
                 Ble.argv[Ble.it - 1]);
    }
    printlog();
}

static void saveWifiCreditian(const char *key, char *creditian) {
    Storage.begin(filename);

    if (Storage.isKey(key))
        Storage.remove(key);
    Storage.putString(key, creditian);

    snprintf(outputLog, outputLogLength, "%s: %s.", key, creditian);
    printlog();
}

static void connectToWiFi(char *ssid, char *pass) {
    wl_status_t status = WL_IDLE_STATUS;
    bool errorOccured = false;
    unsigned times = 0;
    const unsigned timesMax = 64;

    WiFi.begin(ssid, pass);
    while ((status = WiFi.status()) != WL_CONNECTED && times++ < timesMax)
        delay(0x40);

    if (times < timesMax) {
        snprintf(outputLog, outputLogLength, "Connecting to server...");
        printlog();
        if (Client.connect(addr, port)) {
            snprintf(outputLog, outputLogLength,
                     "Connected to server succesfully!");
        } else {
            snprintf(outputLog, outputLogLength,
                     "Failed to connect to the server.");
            errorOccured = true;
        }
    } else {
        snprintf(outputLog, outputLogLength,
                 "Wi-Fi not connected after %u times.", timesMax);
        errorOccured = true;
    }
    printlog();
    if (errorOccured) {
        disconnectFromWiFi(false);
    }
}

static void disconnectFromWiFi(bool printMessage) {
    Client.stop();
    wifi_mode_t currentMode = WiFi.getMode();
    bool isEnabled = ((currentMode & WIFI_MODE_STA) != 0);
    if (isEnabled) {
        if (WiFi.disconnect(true)) {
            if (printMessage) {
                snprintf(outputLog, outputLogLength, "Wi-Fi disconnected.");
                printlog();
            }
        } else {
            snprintf(outputLog, outputLogLength,
                     "Can not stop Wi-Fi! It's still running.");
            printlog();
        }
    }
}

static void checkWifiCreditian() {
    snprintf(outputLog, outputLogLength,
             "Wi-Fi creditians <ssid, pass, addr>: %s, %s, %s.", ssid, pass,
             addr);
    printlog();
}

static void rebootMCU() {
    snprintf(outputLog, outputLogLength, "The device will now reboot!");
    printlog();
    disconnectFromWiFi(false);
    delay(1000);
    esp_restart();
}

static void deepSleepMCU() {
    snprintf(outputLog, outputLogLength, "The device goes to sleep.");
    printlog();
    disconnectFromWiFi(false);
    delay(1000);
    esp_deep_sleep_start();
}

static void temperatureMCU() {
    snprintf(outputLog, outputLogLength, "MCU temperature: %.1f C.",
             temperatureRead());
    printlog();
}

void setup() {
    unsigned long timeStamp = millis();
    if (!ledcSetup(RED_LED, 500, 8) || !ledcSetup(GREEN_LED, 500, 8) ||
        !ledcSetup(BLUE_LED, 500, 8)) {
        log_e(">> LED is not initialized correctly!");
    }

    ledcAttachPin(Led.red, RED_LED);
    ledcAttachPin(Led.green, GREEN_LED);
    ledcAttachPin(Led.blue, BLUE_LED);

#if (DEBUG == 1)
    ledcWrite(RED_LED, Led.duty);
#else
    ledcWrite(RED_LED, 0U);
#endif // DEBUG
    ledcWrite(GREEN_LED, 0U);
    ledcWrite(BLUE_LED, 0U);

    if (Storage.begin(filename)) {
        log_e(">> Storage error!");
    }

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

    snprintf(outputLog, outputLogLength, "Initialization completed in %u ms.",
             millis() - timeStamp);
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
            snprintf(outputLog, outputLogLength, "Bluetooth LE connected.");
            printlog();
            Flag.ble = true;
        }
        if (Ble.isParsed() && !Ble.it) {
            for (; Ble.it < Ble.argc; Ble.it++) {
                char *arg = Ble.toLower(Ble.argv[Ble.it]);
                auto it = Ble.commandList.find(Ble.hash(arg));
                if (it != Ble.commandList.end())
                    it->second();
                else {
                    snprintf(outputLog, outputLogLength,
                             "Unknown command received.");
                    printlog();
                }
                free(arg);
            }
        }
    } else {
        if (Flag.ble) {
            log_e(">> Bluetooth LE disconnected.");
            Flag.ble = false;
        }
    }

    if (WiFi.isConnected() && Client.connected()) {
        if (!Flag.wifi) {
            snprintf(outputLog, outputLogLength,
                     "Main loop Wi-Fi flag activated!");
            printlog();
            Flag.wifi = true;
#if (DEBUG == 1)
            ledcWrite(GREEN_LED, Led.duty);
#endif // DEBUG
            String greetings("Hello, I'm a client!");
            Client.print(WiFi.macAddress() +
                         "::" + Coder.codeStringWithAppend(greetings));
        }
        if (Client.available()) {
            String answer = Client.readString();
            snprintf(outputLog, outputLogLength,
                     "Data received: ", answer.c_str());
            printlog();
        }
    } else {
        if (Flag.wifi) {
            snprintf(outputLog, outputLogLength, "Wi-Fi main loop ended.");
            printlog();
            disconnectFromWiFi(true);
            Flag.wifi = false;
#if (DEBUG == 1)
            ledcWrite(GREEN_LED, 0U);
#endif // DEBUG
        }
    }

#if (DEBUG == 1)
    switch (state) {
    case true:
        if ((duty += 0x5) == 0xFF)
            state = false;
        break;
    case false:
        if ((duty -= 0x5) == 0x00)
            state = true;
        break;
    }
    ledcWrite(RED_LED, duty);
#endif // DEBUG
    delay(10);
}
