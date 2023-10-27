#include "defines.h"
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
#include <esp32-hal-adc.h>
#include <sleep_modes.h>

static void initWifiCreditian(const char *, char *);
static void changeWifiCreditian(char *);
static void saveWifiCreditian(const char *, char *);
static void connectToWiFi(char *, char *);
static void disconnectFromWiFi(bool);
static void checkWifiCreditian();
static void temperatureMCU();
static inline void resetAdcPins();
static void shutDownPeripheral(bool);
static void RTC_IRAM_ATTR wakeStub();
static void sleepMCU();
static void rebootMCU();
static void ARDUINO_ISR_ATTR risingPumpPinInterrupt();
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
const unsigned valueToSleep = 6000U;
static unsigned loopsToSleep = 0U;
static hw_timer_t *timer0 = NULL;
static unsigned long oldTimeStamp = 0U;
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
    bool look : 1;
} Flag = {false};
static struct led_s {
    const uint8_t red;
    const uint8_t green;
    const uint8_t number;
    const uint8_t duty;
} Led = {.red = RED_LED_PIN, .green = GREEN_LED_PIN, .number = 3U, .duty = 30U};
static class Ble : public BleParser, public BleHash {
  public:
    void onWrite(BLECharacteristic *pCharacteristic) override;
    void initBleHash() override;
} Ble;
static WiFiClient Client;
static WiFiCoder Coder;
static AdcSelf Adc;
static Preferences Storage;
static LcdParser Lcd;
static DS18B20 Temperature(ONEWIRE_PIN);

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
        for (unsigned counter = 0U; counter < value.length(); counter++) {
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
    commandList[hash("temperature")] = []() { temperatureMCU(); };
    commandList[hash("sleep")] = []() { sleepMCU(); };
    commandList[hash("reboot")] = []() { rebootMCU(); };
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
    const unsigned delayValue = 32U;

    WiFi.begin(ssid, pass);
    while ((status = WiFi.status()) != WL_CONNECTED && times++ < timesMax)
        delay(delayValue);

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

static void temperatureMCU() {
    printlog("MCU temperature: %.1f C.", temperatureRead());
}

static inline void resetAdcPins() {
    gpio_deep_sleep_wakeup_disable(static_cast<gpio_num_t>(WAKEUP_PIN));
    gpio_force_unhold_all();
    for (unsigned pin = 0; pin < sizeof_ADC; pin++) {
        gpio_config_t pinConfig = {.pin_bit_mask = 0x1UL << pin,
                                   .mode = GPIO_MODE_DISABLE,
                                   .pull_up_en = GPIO_PULLUP_DISABLE,
                                   .pull_down_en = GPIO_PULLDOWN_DISABLE,
                                   .intr_type = GPIO_INTR_DISABLE};
        ESP_ERROR_CHECK(gpio_config(&pinConfig));
    }
}

static void shutDownPeripheral(bool resetAdc) {
    if (resetAdc)
        resetAdcPins();
    disconnectFromWiFi(false);
    BLEDevice::deinit(true);
}

static void RTC_IRAM_ATTR wakeStub() {
    Sleep::esp_default_wake_deep_sleep();
    resetAdcPins();
}

static void sleepMCU() {
    const uint64_t WAKEUP_HIGH_PIN_BITMASK = 1 << WAKEUP_PIN;
    printlog("The device goes to sleep.");
    digitalWrite(GATE_2_PIN, true);
    pinMode(WAKEUP_PIN, INPUT);
    gpio_deep_sleep_hold_en();
    Sleep::esp_deep_sleep_enable_gpio_wakeup(WAKEUP_HIGH_PIN_BITMASK,
                                             ESP_GPIO_WAKEUP_GPIO_HIGH);
    Sleep::esp_set_deep_sleep_wake_stub(&wakeStub);
    shutDownPeripheral(false);
    delay(100U);
    Sleep::esp_deep_sleep_start();
}

static void rebootMCU() {
    printlog("The device will now reboot!");
    shutDownPeripheral(true);
    delay(100U);
    esp_restart();
}

static void ARDUINO_ISR_ATTR risingPumpPinInterrupt() {
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
    static unsigned char currentBit = 0U;
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
        for (unsigned gate = 0U; gate < GATES_IN_LCD; gate++) {
            digitalWrite(GATE_0_PIN + gate, true);
            delayMicroseconds(10U);
            for (unsigned pin = 0U; pin < sizeof_ADC; pin++) {
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
        currentBit = 0U;
        Flag.timer = true;
#if (DEBUG == 1)
        oldTimeStamp = 0U;
        printlog("Data copied to buffer.");
        printlog("Time stamps:");
        for (timeStampCounter = 0U; timeStampCounter < timeStampsCounts;
             timeStampCounter++) {
            printlog("%u: %u us%c", timeStampCounter,
                     timeStamps[timeStampCounter],
                     (timeStampCounter < timeStampsCounts - 1) ? ';' : '.');
        }
#endif // DEBUG
    }
}

/***************************************************************************/

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

    timer0 = timerBegin(0U, timerDivider, true);
    timerStop(timer0);
    timerAttachInterrupt(timer0, &timer0Interrupt, true);
    timerAlarmWrite(timer0, timerFirstStep, false);
    timerAlarmEnable(timer0);
    timerWrite(timer0, 0U);
    printlog("Timer0 initiated at %u ms.", millis() - timeStamp);

    for (unsigned pin = 0U; pin < sizeof_ADC; pin++) {
        if (!Adc.attachPin(pin)) {
            printlog("The ADC did not work properly!");
            errorOccured = true;
        }
    }
    for (unsigned gate = 0U; gate < GATES_IN_LCD; gate++) {
        pinMode(GATE_0_PIN + gate, OUTPUT); /*Communicator gates*/
        digitalWrite(GATE_0_PIN + gate, false);
    }
    pinMode(CLK_PIN, OPEN_DRAIN);
    digitalWrite(CLK_PIN, false); /*To charge capacitor ASAP*/
#if (DEBUG == 1)
    pinMode(DEBUG_PIN, OUTPUT);
    digitalWrite(DEBUG_PIN, true);
#endif // DEBUG
    pinMode(PUMP_PIN, INPUT);
    attachInterrupt(digitalPinToInterrupt(PUMP_PIN), &risingPumpPinInterrupt,
                    RISING);
    Flag.look = digitalRead(PUMP_PIN);

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

    if ((Flag.timer || Flag.look) && !Flag.ble) {
        if (loopsToSleep++ == valueToSleep) {
            sleepMCU();
        }
        if (!digitalRead(PUMP_PIN)) {
            rebootMCU();
        }
    }

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
            float temperature;
#if (DEMO == 1)
            temperature = (float)random(35L, 37L) + ((float)random(9L) / 10.0f);
            snprintf(Lcd.Data.SYS, sizeof(Lcd.Data.SYS) * sizeof(char), "%ld",
                     random(90L, 150L));
            snprintf(Lcd.Data.DIA, sizeof(Lcd.Data.DIA) * sizeof(char), "%ld",
                     random(60L, 100L));
            snprintf(Lcd.Data.PUL, sizeof(Lcd.Data.PUL) * sizeof(char), "%ld",
                     random(50L, 90L));
#else
            temperature = Temperature.getTempC();
            Lcd.parseLcd();
#endif // DEMO

#define LCD_DATA                                                               \
    "SYS:%s::DIA:%s::PUL:%s::TMP:%02.1f", Lcd.Data.SYS, Lcd.Data.DIA,          \
        Lcd.Data.PUL, temperature
            unsigned length = snprintf(NULL, 0U, LCD_DATA) + 1U;
            char *lcdData = new char[length];
            snprintf(lcdData, length, LCD_DATA);
            Client.print(WiFi.macAddress() +
                         "::" + Coder.codeStringWithAppend(lcdData));
            printlog(lcdData);
            Flag.timer = false;
            delete[] lcdData;
            sleepMCU();
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
