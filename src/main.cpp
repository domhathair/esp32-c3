#include <AdcSelf.h>
#include <Arduino.h>
#include <BleHash.h>
#include <BleParser.h>
#include <BleSerial.h>
#include <Preferences.h>
#include <WiFi.h>
#include <WiFiClient.h>
#include <WiFiCoder.h>

#define RED_LED_CHANNEL 0U
#define GREEN_LED_CHANNEL 1U
#define BLUE_LED_CHANNEL 2U
#define GATE_CHANNEL_0 6U
#define GATE_CHANNEL_1 7U
#define GATE_CHANNEL_2 8U
#define CLK_PIN 9U

static void initWifiCreditian(const char *, char *);
static void changeWifiCreditian(char *);
static void saveWifiCreditian(const char *, char *);
static void connectToWiFi(char *, char *);
static void disconnectFromWiFi(bool);
static void checkWifiCreditian();
static void rebootMCU();
static void deepSleepMCU();
static void temperatureMCU();
static void ARDUINO_ISR_ATTR onHighClkPinInterrupt();
static void ARDUINO_ISR_ATTR timer0Interrupt();

const unsigned creditianLength = 32U;
const unsigned outputLogLength = 128U;
const unsigned port = 80U;
const unsigned gates = 3U;
const unsigned bitsInPackage = 6U;
const unsigned timerDivider = 80U;
const unsigned timerStep = 47000U / 12U;
const unsigned wires = (unsigned)AdcSelf::sizeof_ADC * gates;
unsigned short wireData[bitsInPackage][wires] = {0U};
static char ssid[creditianLength] = {0};
static char pass[creditianLength] = {0};
static char addr[creditianLength] = {0};
static char outputLog[outputLogLength] = {0};
static String BLEName("BLE-MAC::" + WiFi.macAddress());
const char *filename = "settings";
const char *ssidKey = "ssid";
const char *passKey = "pass";
const char *addrKey = "addr";
const unsigned activeTimers = 1U;
static hw_timer_t *timer0 = NULL;
static bool setupReady = false;

struct Led {
    const uint8_t red;
    const uint8_t green;
    const uint8_t blue;
    const uint8_t number;
    const uint8_t duty;
} Led = {.red = 10U, .green = 11U, .blue = 12U, .number = 3U, .duty = 30U};

class Ble : public BleParser, public BleHash {
  public:
    void onWrite(BLECharacteristic *pCharacteristic) override;
    void initBleHash() override;
} Ble;

WiFiClient Client;

WiFiCoder Coder;

AdcSelf Adc;

Preferences Storage;

#define printlog(...)                                                          \
    {                                                                          \
        snprintf(outputLog, outputLogLength, __VA_ARGS__);                     \
        if (Ble.connected()) {                                                 \
            Ble.printf("%s\r\n", outputLog);                                   \
        }                                                                      \
        log_e(">> %s", outputLog);                                             \
    }

void Ble::onWrite(BLECharacteristic *pCharacteristic) {
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
                if (Flag.PR && Flag.CR) {
                    String string = readString();
                    parseString(string, " ,;:!?/%\n\r");
#if (DEBUG == 1)
                    log_e(">> Received: %s", string.c_str());
#endif // DEBUG
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

void Ble::initBleHash() {
    commandList[hash("ssid")] = []() { changeWifiCreditian(ssid); };
    commandList[hash("pass")] = []() { changeWifiCreditian(pass); };
    commandList[hash("addr")] = []() { changeWifiCreditian(addr); };
    commandList[hash("save")] = []() {
        log_e(">> Saved Wi-Fi creditians <key: creditian>:");
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

static void initWifiCreditian(const char *key, char *creditian) {
    if (Storage.isKey(key)) {
        Storage.getString(key, creditian, creditianLength);
        printlog("Found creditian under key <%s>: %s", key, creditian);
    } else {
        const char *basicCreditian = "dummy";
        strncpy(creditian, basicCreditian, creditianLength);
        Storage.putString(key, creditian);
        printlog("Created new creditian under key <%s>: %s", key, creditian);
    }
}

static void changeWifiCreditian(char *creditian) {
    if (Ble.argc > Ble.it) {
        Ble.it++;
        unsigned length = strlen(Ble.argv[Ble.it]);
        if (length > creditianLength) {
            printlog("Wi-Fi creditian length must be less than %u.",
                     creditianLength);
        } else {
            memset(creditian, '\0', creditianLength);
            strncpy(creditian, Ble.argv[Ble.it], creditianLength);
            creditian[length] = '\0';
            printlog("Wi-Fi creditian <%s> changed to \"%s\".",
                     Ble.argv[Ble.it - 1], creditian);
        }
    } else {
        printlog("Fewer arguments than necessary in <%s>.",
                 Ble.argv[Ble.it - 1]);
    }
}

static void saveWifiCreditian(const char *key, char *creditian) {
    Storage.begin(filename);

    if (Storage.isKey(key))
        Storage.remove(key);
    Storage.putString(key, creditian);

    printlog("%s: %s.", key, creditian);
}

static void connectToWiFi(char *ssid, char *pass) {
    wl_status_t status = WL_IDLE_STATUS;
    bool errorOccured = false;
    unsigned times = 0U;
    const unsigned timesMax = 64U;

    WiFi.begin(ssid, pass);
    while ((status = WiFi.status()) != WL_CONNECTED && times++ < timesMax)
        delay(0x10U);

    if (times < timesMax) {
        printlog("Connecting to server...");
        if (Client.connect(addr, port)) {
            printlog("Connected to server succesfully!");
        } else {
            printlog("Failed to connect to the server.");
            errorOccured = true;
        }
    } else {
        printlog("Wi-Fi not connected after %u times.", timesMax);
        errorOccured = true;
    }
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
                printlog("Wi-Fi disconnected.");
            }
        } else {
            printlog("Can not stop Wi-Fi! It's still running.");
        }
    }
}

static void checkWifiCreditian() {
    printlog("Wi-Fi creditians <ssid, pass, addr>: %s, %s, %s.", ssid, pass,
             addr);
}

static void rebootMCU() {
    printlog("The device will now reboot!");
    disconnectFromWiFi(false);
    delay(1000);
    esp_restart();
}

static void deepSleepMCU() {
    printlog("The device goes to sleep.");
    disconnectFromWiFi(false);
    delay(1000);
    esp_deep_sleep_start();
}

static void temperatureMCU() {
    printlog("MCU temperature: %.1f C.", temperatureRead());
}

static void ARDUINO_ISR_ATTR onHighClkPinInterrupt() {
    if (setupReady) {
        detachInterrupt(digitalPinToInterrupt(CLK_PIN));

        timerStart(timer0);

        printlog("Clock signal caught.");
    }
    /* */
}

static void ARDUINO_ISR_ATTR timer0Interrupt() {
    static unsigned char currentBit = 0;
#if (DEBUG == 1)
    static unsigned long oldStamp = 0;
    unsigned long timeStamp = micros();
    printlog("%s: %u us.", (!oldStamp) ? "timer started at" : "timer duration",
             timeStamp - oldStamp);
    oldStamp = timeStamp;
#endif // DEBUG

    if (currentBit < bitsInPackage) {
        for (unsigned gate = 0; gate < gates; gate++) {
            digitalWrite(GATE_CHANNEL_0 + gate, true);
            for (unsigned pin = 0; pin < AdcSelf::sizeof_ADC; pin++) {
                wireData[currentBit][gate * (unsigned)AdcSelf::sizeof_ADC +
                                     pin] = Adc.read((AdcSelf::channel_t)pin);
            }
            digitalWrite(GATE_CHANNEL_0 + gate, false);
        }
        currentBit++;
    } else {
        timerStop(timer0);
        currentBit = 0;
        oldStamp = 0;
        printlog("Data copied to buffer.");
    }
}

void setup() {
    unsigned long timeStamp = millis();
    bool errorOccured = false;

    if (!Ble.begin(BLEName.c_str(), true)) {
        log_e(">> BLE did not start!");
        errorOccured = true;
    } else {
        printlog("BLE started after name %s", BLEName.c_str());
    }
    Ble.initBleHash();

    if (!ledcSetup(RED_LED_CHANNEL, 500, 8) ||
        !ledcSetup(GREEN_LED_CHANNEL, 500, 8) ||
        !ledcSetup(BLUE_LED_CHANNEL, 500, 8)) {
        printlog("LED is not initialized correctly at %u ms.",
                 millis() - timeStamp);
        errorOccured = true;
    }

    ledcAttachPin(Led.red, RED_LED_CHANNEL);
    ledcAttachPin(Led.green, GREEN_LED_CHANNEL);
    ledcAttachPin(Led.blue, BLUE_LED_CHANNEL);
    printlog("LED attached to their channels at %u ms.", millis() - timeStamp);

    timer0 = timerBegin(0, timerDivider, true);
    timerStop(timer0);
    timerAttachInterrupt(timer0, &timer0Interrupt, true);
    timerAlarmWrite(timer0, timerStep, true);
    timerAlarmEnable(timer0);
    timerWrite(timer0, 0U);
    printlog("Timer0 initiated at %u ms.", millis() - timeStamp);

    for (unsigned pin = 0; pin < AdcSelf::sizeof_ADC; pin++) {
        if (!Adc.attachPin((AdcSelf::channel_t)pin)) {
            printlog("The ADC did not work properly!");
            errorOccured = true;
        }
    }
    for (unsigned gate = 0; gate < gates; gate++) {
        pinMode(GATE_CHANNEL_0 + gate, OUTPUT); /*Communicator gates*/
        digitalWrite(GATE_CHANNEL_0 + gate, false);
    }
    pinMode(CLK_PIN, OPEN_DRAIN);
    attachInterrupt(digitalPinToInterrupt(CLK_PIN), &onHighClkPinInterrupt,
                    RISING);
    printlog("Digital pins initiated at %u ms.", millis() - timeStamp);

#if (DEBUG == 1)
    ledcWrite(RED_LED_CHANNEL, Led.duty);
#else
    ledcWrite(RED_LED_CHANNEL, 0U);
#endif // DEBUG
    ledcWrite(GREEN_LED_CHANNEL, 0U);
    ledcWrite(BLUE_LED_CHANNEL, 0U);

    if (!Storage.begin(filename)) {
        printlog("Storage error at %u ms.", millis() - timeStamp);
        errorOccured = true;
    }

    initWifiCreditian(ssidKey, ssid);
    initWifiCreditian(passKey, pass);
    initWifiCreditian(addrKey, addr);

    connectToWiFi(ssid, pass);

    Storage.end();

    printlog("Initialization completed %s in %u ms.",
             errorOccured ? "with errors" : "successfully",
             millis() - timeStamp);
    setupReady = true;
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
            printlog("Bluetooth LE connected.");
            Flag.ble = true;
        }
        if (Ble.isParsed() && !Ble.it) {
            for (; Ble.it < Ble.argc; Ble.it++) {
                char *arg = Ble.toLower(Ble.argv[Ble.it]);
                auto it = Ble.commandList.find(Ble.hash(arg));
                if (it != Ble.commandList.end())
                    it->second();
                else {
                    printlog("Unknown command received.");
                }
                free(arg);
            }
        }
    } else {
        if (Flag.ble) {
            printlog("Bluetooth LE disconnected.");
            Flag.ble = false;
        }
    }

    if (WiFi.isConnected() && Client.connected()) {
        if (!Flag.wifi) {
            printlog("Main loop Wi-Fi flag activated!");
            Flag.wifi = true;
#if (DEBUG == 1)
            ledcWrite(GREEN_LED_CHANNEL, Led.duty);
#endif // DEBUG
            String greetings("Hello, I'm a client!");
            Client.print(WiFi.macAddress() +
                         "::" + Coder.codeStringWithAppend(greetings));
        }
        if (Client.available()) {
            String answer = Client.readString();
            printlog("Data received: ", answer.c_str());
        }
    } else {
        if (Flag.wifi) {
            printlog("Wi-Fi main loop ended.");
            disconnectFromWiFi(true);
            Flag.wifi = false;
#if (DEBUG == 1)
            ledcWrite(GREEN_LED_CHANNEL, 0U);
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
    ledcWrite(RED_LED_CHANNEL, duty);
#endif // DEBUG
    delay(1);
}
