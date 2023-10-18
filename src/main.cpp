#include <AdcSelf.h>
#include <Arduino.h>
#include <BleHash.h>
#include <BleParser.h>
#include <BleSerial.h>
#include <DS18B20.h>
#include <LcdParser.h>
#include <Preferences.h>
#include <WiFi.h>
#include <WiFiClient.h>
#include <WiFiCoder.h>

#define RED_LED_CHANNEL 0U
#define GREEN_LED_CHANNEL 1U

#define ADC1_CH0_PIN ADC1_CH0 /*GPIO0*/
#define ADC1_CH1_PIN ADC1_CH1 /*GPIO1*/
#define ADC1_CH2_PIN ADC1_CH2 /*GPIO2*/
#define ADC1_CH3_PIN ADC1_CH3 /*GPIO3*/
#define ADC1_CH4_PIN ADC1_CH4 /*GPIO4*/
#define ADC2_CH0_PIN ADC2_CH0 /*GPIO5*/
#define GATE_0_PIN 6U         /*GPIO6*/
#define GATE_1_PIN 7U         /*GPIO7*/
#define GATE_2_PIN 8U         /*GPIO8*/
#if (DEBUG == 1)
#define DEBUG_PIN 9U      /*GPIO9*/
#endif                    // DEBUG
#define CLK_PIN 10U       /*GPIO10*/
#define RED_LED_PIN 12U   /*SPIHD*/
#define GREEN_LED_PIN 13U /*SPIWP*/
#define ONEWIRE_PIN 16U   /*SPID*/
#define PUMP_PIN 17U      /*SPIQ*/
#define USB_N_PIN 18U     /*GPIO18*/
#define USB_P_PIN 19U     /*GPIO19*/

static void initWifiCreditian(const char *, char *);
static void changeWifiCreditian(char *);
static void saveWifiCreditian(const char *, char *);
static void connectToWiFi(char *, char *);
static void disconnectFromWiFi(bool);
static void checkWifiCreditian();
static void rebootMCU();
static void deepSleepMCU();
static void temperatureMCU();
static void ARDUINO_ISR_ATTR risingClkPinInterrupt();
static void ARDUINO_ISR_ATTR timer0Interrupt();

const unsigned creditianLength = 32U;
const unsigned outputLogLength = 128U;
const unsigned port = 80U;
const unsigned timerDivider = 80U;
const unsigned timerFirstStep = 1000U;
const unsigned timerStep = 47000U / 12U;
const unsigned activeTimers = 1U;
const char *filename = "settings";
const char *ssidKey = "ssid";
const char *passKey = "pass";
const char *addrKey = "addr";
static hw_timer_t *timer0 = NULL;
static unsigned long oldTimeStamp = 0;
static char ssid[creditianLength] = {0};
static char pass[creditianLength] = {0};
static char addr[creditianLength] = {0};
static char outputLog[outputLogLength] = {0};
static String BLEName("BLE-MAC::" + WiFi.macAddress());

static struct flag_s {
    bool ble : 1;
    bool wifi : 1;
    bool delay : 1;
    bool timer : 1;
    bool ready : 1;
} Flag = {false};

static struct led_s {
    const uint8_t red;
    const uint8_t green;
    const uint8_t number;
    const uint8_t duty;
} Led = {.red = RED_LED_PIN, .green = GREEN_LED_PIN, .number = 3U, .duty = 30U};

class Ble : public BleParser, public BleHash {
  public:
    void onWrite(BLECharacteristic *pCharacteristic) override;
    void initBleHash() override;
} Ble;

WiFiClient Client;

WiFiCoder Coder;

AdcSelf Adc;

Preferences Storage;

LcdParser Lcd;

DS18B20 Temperature(ONEWIRE_PIN);

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

static void ARDUINO_ISR_ATTR fallingPumpPinInterrupt() {
    attachInterrupt(digitalPinToInterrupt(CLK_PIN), &risingClkPinInterrupt,
                    RISING); /*To activate it only after pump shuts down*/
    detachInterrupt(digitalPinToInterrupt(PUMP_PIN));
}

static void ARDUINO_ISR_ATTR risingClkPinInterrupt() {
    static unsigned short suppressClock = 8U;
    oldTimeStamp = micros();
    if (Flag.ready && !suppressClock--) {
        detachInterrupt(digitalPinToInterrupt(CLK_PIN));
        timerStart(timer0);
    }
}

static void ARDUINO_ISR_ATTR timer0Interrupt() {
    static unsigned char currentBit = 0;
#if (DEBUG == 1)
    digitalWrite(DEBUG_PIN, !digitalRead(DEBUG_PIN));
    unsigned long timeStamp = micros();
    const unsigned timeStampsCounts =
        BITS_IN_PACKAGE + 2; /*For output and first entry*/
    static unsigned timeStampCounter = 0U;
    static unsigned long timeStamps[timeStampsCounts] = {0U};
    timeStamps[timeStampCounter++] = timeStamp - oldTimeStamp;
    oldTimeStamp = timeStamp;
#endif // DEBUG

    if (!Flag.delay) {
        timerWrite(timer0, 0U);
        timerAlarmWrite(timer0, timerStep, true);
        Flag.delay = true;
    } else if (currentBit < BITS_IN_PACKAGE && Flag.delay) {
        for (unsigned gate = 0; gate < GATES_IN_LCD; gate++) {
            digitalWrite(GATE_0_PIN + gate, true);
            for (unsigned pin = 0; pin < sizeof_ADC; pin++) {
                uint16_t wireData = Adc.read(pin);
                switch (wireData) {
                case (uint16_t)(4096.0f * 0.70f)...(uint16_t)(4096.0f * 1.00f):
                    Lcd.wiresData[gate * sizeof_ADC + pin][currentBit] =
                        LCD_HIGH;
                    break;
                case (uint16_t)(4096.0f * 0.00f)...(uint16_t)(4096.0f * 0.30f):
                    Lcd.wiresData[gate * sizeof_ADC + pin][currentBit] =
                        LCD_MIDDLE;
                    break;
                default:
                    Lcd.wiresData[gate * sizeof_ADC + pin][currentBit] =
                        LCD_LOW;
                    break;
                }
            }
            digitalWrite(GATE_0_PIN + gate, false);
        }
        currentBit++;
    } else {
        timerStop(timer0);
        currentBit = 0;
        Flag.timer = true;
#if (DEBUG == 1)
        oldTimeStamp = 0;
        printlog("Data copied to buffer.");
        printlog("Time stamps:");
        for (timeStampCounter = 0; timeStampCounter < timeStampsCounts;
             timeStampCounter++) {
            printlog("%u: %u us%c", timeStampCounter,
                     timeStamps[timeStampCounter],
                     (timeStampCounter < timeStampsCounts - 1) ? ';' : '.');
        }
#endif // DEBUG
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
        !ledcSetup(GREEN_LED_CHANNEL, 500, 8)) {
        printlog("LED is not initialized correctly at %u ms.",
                 millis() - timeStamp);
        errorOccured = true;
    }

    ledcAttachPin(Led.red, RED_LED_CHANNEL);
    ledcAttachPin(Led.green, GREEN_LED_CHANNEL);
    printlog("LED attached to their channels at %u ms.", millis() - timeStamp);

    timer0 = timerBegin(0, timerDivider, true);
    timerStop(timer0);
    timerAttachInterrupt(timer0, &timer0Interrupt, true);
    timerAlarmWrite(timer0, timerFirstStep, false);
    timerAlarmEnable(timer0);
    timerWrite(timer0, 0U);
    printlog("Timer0 initiated at %u ms.", millis() - timeStamp);

    for (unsigned pin = 0; pin < sizeof_ADC; pin++) {
        if (!Adc.attachPin(pin)) {
            printlog("The ADC did not work properly!");
            errorOccured = true;
        }
    }
    for (unsigned gate = 0; gate < GATES_IN_LCD; gate++) {
        pinMode(GATE_0_PIN + gate, OUTPUT); /*Communicator gates*/
        digitalWrite(GATE_0_PIN + gate, false);
    }
    pinMode(CLK_PIN, OPEN_DRAIN);
    digitalWrite(CLK_PIN, false); /*To charge capacitor ASAP*/
#if (DEBUG == 1)
    pinMode(DEBUG_PIN, OUTPUT);
    digitalWrite(DEBUG_PIN, true);
#endif // DEBUG
    pinMode(PUMP_PIN, OPEN_DRAIN);
    attachInterrupt(digitalPinToInterrupt(OPEN_DRAIN), &fallingPumpPinInterrupt,
                    FALLING);

    printlog("Digital pins initiated at %u ms.", millis() - timeStamp);

#if (DEBUG == 1)
    ledcWrite(RED_LED_CHANNEL, Led.duty);
#else
    ledcWrite(RED_LED_CHANNEL, 0U);
#endif // DEBUG
    ledcWrite(GREEN_LED_CHANNEL, 0U);

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
    Flag.ready = true;
}

void loop() {
    static bool state = true;
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
        if (Flag.timer) {
#define LCD_DATA                                                               \
    "SYS:%s::DIA:%s::PUL:%s::TMP:%02.1f", Lcd.Data.SYS, Lcd.Data.DIA,          \
        Lcd.Data.PUL, Temperature.getTempC()
            Lcd.parseLcd();
            unsigned length = snprintf(NULL, 0, LCD_DATA) + 1U;
            char *lcdData = new char[length];
            snprintf(lcdData, length, LCD_DATA);
            Client.print(lcdData);
            printlog(lcdData);
            Flag.timer = false;
            delete[] lcdData;
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
    delay(10);
}
