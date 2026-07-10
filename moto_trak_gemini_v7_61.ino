// =========================================================================
//  MOTO TRACKER V7.61 - LOG STATUS STRING FIX
// =========================================================================
// HARDWARE: LilyGo T-SIM7000G (ESP32)
// CARRIER: Google Fi (APN: h2g2)
//
// CHANGES FROM V7.60:
//   - CLEARLOG now publishes "LOG_WIPED" instead of "LOG_CLEARED" on motostatus.
//     "LOG_CLEARED" contained the substring "CLEARED", which HA's
//     alarm_control_panel/switch value_templates match to mean "disarmed" -
//     so clearing the debug log was silently flipping the armed state in HA.
//
// CARRIED OVER FROM V7.60:
//   - LittleFS debug logger (motolog.txt): BOOT, SLEEP_ENTER, WAKE, MQTT_FAIL,
//     LTE_REBUILD, LTE_RETRY, THEFT_TRIGGERED, RECOVERY_MODE_ENTER, TELEMETRY
//   - dumplog/clearlog serial commands, DUMPLOG/CLEARLOG MQTT commands
//   - Log rotates at ~180KB, two generations kept (motolog.txt / motolog_old.txt)
//
// CARRIED OVER FROM V7.59:
//   - ADC-only battery (GPIO35), single motobatt feed
//   - Credentials in secrets.h (copy secrets.example.h)
//   - Heartbeat wakes re-sleep immediately (low-power duty cycle)
//   - 60s MQTT keepalive + 30s socket timeout, non-blocking mqttDelay()
//   - loop() pumped through setup() init gap; NFC optional; serial noise ignored
//
//   NOTE: GPIO35 only reads real voltage on BATTERY power, not USB.
//   NOTE: Requires a partition scheme with a LittleFS/SPIFFS data region.
// =========================================================================
// =========================================================================

#define TINY_GSM_MODEM_SIM7000
#define TINY_GSM_RX_BUFFER 4096
#define MQTT_MAX_PACKET_SIZE 4096

#include <Wire.h>
#include <Adafruit_PN532.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <PubSubClient.h>
#include <TinyGsmClient.h>
#include <driver/gpio.h>
#include <driver/rtc_io.h>
#include <LittleFS.h>
#include "secrets.h"   // MQTT credentials - copy secrets.example.h to secrets.h

// --- HARDWARE MAP ---
#define PIN_TX 27
#define PIN_RX 26
#define PWR_PIN 4
#define LED_PIN 18

// --- USER UPDATED PINS ---
#define MPU_INT_PIN 32
#define NFC_IRQ_PIN 33
#define NFC_RST_PIN 19

// --- OBJECTS ---
HardwareSerial SerialAT(1);
TinyGsm modem(SerialAT);
TinyGsmClient gsmClient(modem);
PubSubClient mqtt(gsmClient);
Adafruit_MPU6050 mpu;
Adafruit_PN532 nfc(NFC_IRQ_PIN, NFC_RST_PIN);

// --- USER SETTINGS (credentials live in secrets.h - see secrets.example.h) ---
const char* mqtt_user = SECRET_MQTT_USER;
const char* mqtt_pass = SECRET_MQTT_PASS;
const char* mqtt_server = "io.adafruit.com";
const int mqtt_port = 1883;
const char* apn = "h2g2";   // Google Fi public APN - not a secret

const float sensitivity = 4.0;
const float wakeup_threshold = 2.0;

// --- BATTERY THRESHOLDS (volts) ---
const float BATT_WARN_V = 3.50;   // publishes BATT_LOW_X.XXV
const float BATT_CRIT_V = 3.20;   // forces safe shutdown

// --- ONBOARD BATTERY ADC (GPIO35) ---
// GPIO35 is the board's BAT ADC, fed by a ~2:1 divider. It reads ~0 on USB power
// by design - only valid on battery. Tune BAT_ADC_SCALE to your multimeter (start ~2.0).
#define BAT_ADC_PIN 35
const float BAT_ADC_SCALE = 2.0;

// --- DEBUG LOGGER (LittleFS) ---
#define LOG_FILE       "/motolog.txt"
#define LOG_FILE_OLD   "/motolog_old.txt"
// Sized so a full DUMPLOG stays under ~10 min: dumpLogToMqtt() sends 200-byte
// chunks at 3s each, so (36000/200)*3s = 9 min nominal, leaving margin.
#define LOG_MAX_BYTES  36000UL
bool logFsReady = false;

// --- TOPICS ---
String gps_topic    = String(mqtt_user) + "/feeds/motogps/csv";
String batt_topic   = String(mqtt_user) + "/feeds/motobatt";
String signal_topic = String(mqtt_user) + "/feeds/motosignal";
String status_topic = String(mqtt_user) + "/feeds/motostatus";
String cmd_topic    = String(mqtt_user) + "/feeds/motocmd";
String log_topic    = String(mqtt_user) + "/feeds/motolog";

// --- STATE VARIABLES ---
RTC_DATA_ATTR bool isArmed = false;
RTC_DATA_ATTR bool theftActive = false;
RTC_DATA_ATTR bool isStolen = false; // RECOVERY MODE FLAG

bool nfcPresent = false;        // set true only if PN532 responds at boot
bool batteryCritical = false;   // set by health check, handled in loop()
bool immediateResleep = false;  // set after a heartbeat wake to skip the 5-min idle window

unsigned long lastGpsSent = 0;
bool mqttConnected = false;
int mqttFailCount = 0;

bool learnMode = false;
unsigned long learnStartTime = 0;
const unsigned long learnTimeout = 30000;

unsigned long lastFastBlink = 0;
unsigned long lastSlowBlink = 0;

unsigned long lastSuccessfulScan = 0;
const unsigned long scanDebounce = 3000;

unsigned long armTime = 0;
const unsigned long armGracePeriod = 10000;

unsigned long lastActivityTime = 0;
const unsigned long sleepTimeout = 300000;

unsigned long lastMotionTime = 0;
bool motionLatched = false;

unsigned long absoluteWakeTime = 0;
const unsigned long hardKillTimeout = 360000;

// --- FORWARD DECLARATIONS ---
void mqttCallback(char* topic, byte* payload, unsigned int length);
bool initializeLTE();
void connectMQTT();
void enterSleep();
float readBatteryV();
void logEvent(const String& tag, const String& msg);

// -------------------------------------------------------------------------
// HELPERS
// -------------------------------------------------------------------------

// Non-blocking delay that keeps the MQTT keepalive (PINGREQ) flowing.
// Use this instead of delay() anywhere the MQTT socket needs to stay alive.
void mqttDelay(unsigned long ms) {
    unsigned long start = millis();
    while (millis() - start < ms) {
        if (mqtt.connected()) mqtt.loop();
        delay(10);
    }
}

void flashWakeSequence() {
    for (int i = 0; i < 4; i++) {
        digitalWrite(LED_PIN, HIGH); delay(80);
        digitalWrite(LED_PIN, LOW);  delay(80);
    }
    digitalWrite(LED_PIN, HIGH);
}

void commandFeedback(bool isNowArmed) {
    digitalWrite(LED_PIN, LOW);
    delay(100);
    if (isNowArmed) {
        digitalWrite(LED_PIN, HIGH); delay(150);
        digitalWrite(LED_PIN, LOW); delay(150);
        digitalWrite(LED_PIN, HIGH); delay(150);
        digitalWrite(LED_PIN, LOW);
    } else {
        digitalWrite(LED_PIN, HIGH); delay(800);
        digitalWrite(LED_PIN, LOW);
    }
}

void publishStatus(const String& statusMsg) {
  Serial.print("[STATUS] ");
  Serial.println(statusMsg);
  if (mqttConnected) {
      if(mqtt.publish(status_topic.c_str(), statusMsg.c_str())) {
          Serial.println("   >>> MQTT Publish OK");
      } else {
          Serial.println("   >>> MQTT Publish FAILED");
      }
      mqttDelay(1200);
  }
}

// Reads the onboard battery divider on GPIO35 using the ESP32's factory-calibrated
// millivolt path (better than raw analogRead * scale). Averages 16 samples.
// NOTE: GPIO35 returns ~0 while USB is connected - this is only meaningful on battery.
float readBatteryV() {
    uint32_t acc = 0;
    for (int i = 0; i < 16; i++) { acc += analogReadMilliVolts(BAT_ADC_PIN); delay(2); }
    float pin_mv = acc / 16.0;
    return (pin_mv * BAT_ADC_SCALE) / 1000.0;   // 2:1 divider; tune BAT_ADC_SCALE to multimeter
}

// Validity-guarded health check. AT+CBC can return bogus values on this board,
// so anything outside 2.0-4.5V is ignored rather than acted on.
void checkBatteryHealth(float voltage) {
    if (voltage < 2.0 || voltage > 4.5) {
        Serial.println("[BATT] Invalid reading - skipping health check");
        return;
    }
    if (voltage <= BATT_CRIT_V) {
        Serial.print("[BATT] CRITICAL (");
        Serial.print(voltage, 2);
        Serial.println("V) - flagging safe shutdown");
        publishStatus("BATT_CRITICAL_SHUTDOWN");
        batteryCritical = true;   // acted on in loop() -> enterSleep()
    } else if (voltage <= BATT_WARN_V) {
        Serial.print("[BATT] LOW (");
        Serial.print(voltage, 2);
        Serial.println("V)");
        publishStatus("BATT_LOW_" + String(voltage, 2) + "V");
    }
}

// -------------------------------------------------------------------------
// DEBUG LOGGER (LittleFS)
// -------------------------------------------------------------------------
// Mounts the onboard SPI flash data partition. true = format on first-ever
// mount / corruption. Safe to call once at boot; does not erase an existing,
// healthy filesystem on subsequent boots.
void logInit() {
    if (!LittleFS.begin(true)) {
        Serial.println("[LOG] LittleFS mount FAILED - logging disabled this boot");
        logFsReady = false;
        return;
    }
    logFsReady = true;
    Serial.print("[LOG] LittleFS OK - used ");
    Serial.print(LittleFS.usedBytes());
    Serial.print(" / ");
    Serial.print(LittleFS.totalBytes());
    Serial.println(" bytes");

    // Rotate on boot too, in case the size cap was crossed right before a reset.
    File f = LittleFS.open(LOG_FILE, FILE_APPEND);
    if (f) {
        if (f.size() > LOG_MAX_BYTES) {
            f.close();
            LittleFS.remove(LOG_FILE_OLD);
            LittleFS.rename(LOG_FILE, LOG_FILE_OLD);
        } else {
            f.close();
        }
    }
}

String resetReasonToStr(esp_reset_reason_t r) {
    switch (r) {
        case ESP_RST_POWERON:   return "POWERON";
        case ESP_RST_EXT:       return "EXT_RESET";
        case ESP_RST_SW:        return "SW_RESET";
        case ESP_RST_PANIC:     return "PANIC";
        case ESP_RST_INT_WDT:   return "INT_WDT";
        case ESP_RST_TASK_WDT:  return "TASK_WDT";
        case ESP_RST_WDT:       return "WDT";
        case ESP_RST_DEEPSLEEP: return "DEEPSLEEP_WAKE";
        case ESP_RST_BROWNOUT:  return "BROWNOUT";
        case ESP_RST_SDIO:      return "SDIO";
        default:                return "UNKNOWN(" + String((int)r) + ")";
    }
}

// Appends one CSV line: millis,battV,freeHeap,TAG,message
// Cheap on flash (small append, no full rewrite). Rotates opportunistically
// every 50 writes rather than checking file size on every single call.
void logEvent(const String& tag, const String& msg) {
    unsigned long t = millis();
    float vbat = readBatteryV();
    uint32_t heap = ESP.getFreeHeap();

    String line = String(t) + "," + String(vbat, 2) + "," + String(heap) + "," + tag + "," + msg;
    Serial.print("[LOG] "); Serial.println(line);

    if (!logFsReady) return;

    File f = LittleFS.open(LOG_FILE, FILE_APPEND);
    if (f) {
        f.println(line);
        f.close();
    }

    static uint16_t writeCount = 0;
    if (++writeCount >= 50) {
        writeCount = 0;
        File chk = LittleFS.open(LOG_FILE, FILE_READ);
        if (chk) {
            bool tooBig = chk.size() > LOG_MAX_BYTES;
            chk.close();
            if (tooBig) {
                LittleFS.remove(LOG_FILE_OLD);
                LittleFS.rename(LOG_FILE, LOG_FILE_OLD);
            }
        }
    }
}

// Prints the full current log to Serial. Triggered by the "dumplog" serial command.
void dumpLogToSerial() {
    if (!logFsReady) { Serial.println("[LOG] FS not ready"); return; }
    File f = LittleFS.open(LOG_FILE, FILE_READ);
    if (!f) { Serial.println("[LOG] No log file yet"); return; }
    Serial.println("---- LOG DUMP START ----");
    while (f.available()) Serial.write(f.read());
    Serial.println("---- LOG DUMP END ----");
    f.close();
}

// Publishes the full current log to the motolog feed in small chunks.
// Triggered by the DUMPLOG MQTT command. 1200ms pacing (used elsewhere in this
// file) is too fast here - a chunked dump sustains ~50 msg/min, well past
// Adafruit IO's free-tier ~30/min rate limit, so it trips the throttle for
// the whole dump. 3000ms keeps it under that ceiling with margin.
void dumpLogToMqtt() {
    if (!logFsReady) { publishStatus("LOG_FS_NOT_READY"); return; }
    File f = LittleFS.open(LOG_FILE, FILE_READ);
    if (!f) { publishStatus("LOG_FILE_EMPTY"); return; }

    const int CHUNK = 200;
    char buf[CHUNK + 1];
    while (f.available()) {
        int n = f.readBytes(buf, CHUNK);
        buf[n] = 0;
        if (mqttConnected) {
            mqtt.publish(log_topic.c_str(), buf);
            mqttDelay(3000);
        }
    }
    f.close();
    publishStatus("LOG_DUMP_COMPLETE");
}

void logClear() {
    if (!logFsReady) return;
    LittleFS.remove(LOG_FILE);
    LittleFS.remove(LOG_FILE_OLD);
    Serial.println("[LOG] Cleared");
}

void sendTelemetry() {
    Serial.println("[TEL] Reading Battery & Signal...");

    float battV  = readBatteryV();          // GPIO35 calibrated divider read
    int   battMv = (int)(battV * 1000.0);
    int   signalQual = modem.getSignalQuality();

    Serial.print("[TEL-UPDATE] Batt: ");
    Serial.print(battV, 2);
    Serial.print("V (");
    Serial.print(battMv);
    Serial.print("mV) | Sig: ");
    Serial.println(signalQual);

    if (mqttConnected) {
        mqtt.publish(batt_topic.c_str(), String(battMv).c_str());
        mqttDelay(1200);
        mqtt.publish(signal_topic.c_str(), String(signalQual).c_str());
        mqttDelay(1200);
    }

    // Health runs on the GPIO35 reading; the validity guard skips it on USB (~0V).
    checkBatteryHealth(battV);

    logEvent("TELEMETRY", "batt=" + String(battV, 2) + " sig=" + String(signalQual));
}

// -------------------------------------------------------------------------
// MQTT CONNECT
// -------------------------------------------------------------------------
void connectMQTT() {
    Serial.print("[MQTT] Checking connection... ");
    if (mqtt.connected()) {
        Serial.println("ALREADY CONNECTED");
        mqttConnected = true;
        mqttFailCount = 0;
        return;
    }

    mqttConnected = false;
    Serial.println("DISCONNECTED. Attempting to connect...");

    String clientId = "MotoTrak_V7_61";

    if (mqtt.connect(clientId.c_str(), mqtt_user, mqtt_pass)) {
        Serial.println("[MQTT] Connected Success!");
        if(mqtt.subscribe(cmd_topic.c_str())) {
             Serial.println("[MQTT] Subscribed to CMD topic");
        } else {
             Serial.println("[MQTT] Subscription FAILED");
        }
        mqttConnected = true;
        mqttFailCount = 0;
    } else {
        Serial.print("[MQTT] Connection Failed! State Code: ");
        Serial.println(mqtt.state());
        mqttConnected = false;

        mqttFailCount++;
        Serial.print("[MQTT] Consecutive Failures: ");
        Serial.print(mqttFailCount);
        Serial.println("/3");

        logEvent("MQTT_FAIL", "count=" + String(mqttFailCount) + " state=" + String(mqtt.state()));

        if (mqttFailCount >= 3) {
            Serial.println("\n[MQTT] !!! STRIKE 3 - NUCLEAR OPTION !!!");
            Serial.println("[MQTT] GPRS is likely a Zombie. Restarting Cellular Layer...");

            modem.gprsDisconnect();
            delay(3000);

            bool rebuilt = initializeLTE();
            if (rebuilt) {
                Serial.println("[MQTT] Cellular Layer Rebuilt.");
            } else {
                Serial.println("[MQTT] Cellular Rebuild Failed.");
            }
            logEvent("LTE_REBUILD", rebuilt ? "ok" : "fail");
            mqttFailCount = 0;
        }
    }
}

// -------------------------------------------------------------------------
// BOOT GPS LOCK
// -------------------------------------------------------------------------
void getInitialGpsLock() {
    Serial.println("\n------------------------------------------------");
    Serial.println("[GPS] STARTING INITIAL LOCK SEQUENCE");

    unsigned long start = millis();
    int dots = 0;

    while (millis() - start < 120000) {
        float lat, lon, speed, alt, acc;
        int vs, us, yr, mo, dy, hr, mi, sc;

        if (modem.getGPS(&lat, &lon, &speed, &alt, &vs, &us, &acc, &yr, &mo, &dy, &hr, &mi, &sc)) {
            Serial.println("\n[GPS] LOCK ACQUIRED!");
            Serial.print("   > Lat: "); Serial.println(lat, 6);
            Serial.print("   > Lon: "); Serial.println(lon, 6);

            String gpsData = String(speed, 2) + "," + String(lat, 6) + "," + String(lon, 6) + "," + String(alt, 2);

            if (!mqtt.connected()) {
                connectMQTT();
                mqttDelay(500);
            }

            if (mqttConnected) {
                 Serial.println("[GPS] Attempting MQTT Publish...");

                 if (mqtt.publish(gps_topic.c_str(), gpsData.c_str())) {
                     Serial.println("[GPS] SENT OK (Attempt 1)");
                     mqttDelay(1200);
                 } else {
                     Serial.println("[GPS] Send Failed. Forcing Reconnect...");
                     mqtt.disconnect();
                     connectMQTT();
                     mqttDelay(500);

                     if (mqtt.publish(gps_topic.c_str(), gpsData.c_str())) {
                         Serial.println("[GPS] SENT OK (Attempt 2)");
                         mqttDelay(1200);
                     } else {
                         Serial.println("[GPS] CRITICAL FAIL - Broker rejected packet.");
                     }
                 }
            }
            break;
        }
        Serial.print(".");
        dots++;
        if(dots > 60) { Serial.println(); dots = 0; }

        for (int i = 0; i < 50; i++) {
            if (mqttConnected && mqtt.connected()) {
                mqtt.loop();
            }
            delay(100);
        }
    }

    Serial.println("\n[GPS] Initial search finished.");
    Serial.println("------------------------------------------------\n");
}

// -------------------------------------------------------------------------
// LTE INITIALIZATION
// -------------------------------------------------------------------------
bool initializeLTE() {
    SerialAT.begin(115200, SERIAL_8N1, PIN_RX, PIN_TX);

    Serial.println("[LTE] Checking Modem State...");
    bool modemOn = false;
    for (int i = 0; i < 3; i++) {
        if (modem.testAT()) {
            modemOn = true;
            break;
        }
        Serial.print(".");
        delay(500);
    }

    if (modemOn) {
        Serial.println(" OK (Modem was already awake)");
    } else {
        Serial.println(" No Response. Powering up...");
        digitalWrite(PWR_PIN, HIGH);
        delay(1200);
        digitalWrite(PWR_PIN, LOW);
        delay(10000);
    }

    Serial.print("[LTE] Testing AT...");
    unsigned long loopStart = millis();
    while(!modem.testAT()) {
        Serial.print(".");
        delay(1000);
        if (millis() - loopStart > 20000) {
            Serial.println(" FAIL (Modem stuck)");
            return false;
        }
    }
    Serial.println(" OK");

    Serial.println("[LTE] Configuring Parameters...");
    modem.sendAT("+CFUN=1"); delay(1000);
    modem.sendAT("+CMNB=1"); delay(1000);
    modem.sendAT("+CNMP=38"); delay(1000);
    modem.sendAT("+CBANDCFG=\"CAT-M\",2,4,12,66,71"); delay(1000);
    modem.sendAT("+CGDCONT=1,\"IPV4V6\",\"h2g2\""); delay(1000);
    modem.sendAT("+CGAUTH=1,1,\"\",\"\""); delay(1000);

    Serial.println("[LTE] Enabling GPS Power...");
    modem.sendAT("+CGPIO=0,48,1,1");
    delay(500);
    modem.sendAT("+CGNSPWR=1");
    modem.sendAT("+CGNSSEQ=\"L\"");
    delay(2000);

    Serial.print("[LTE] Waiting for Network...");
    unsigned long netStart = millis();
    while (!modem.waitForNetwork(1000L)) {
        Serial.print(".");
        if (millis() - netStart > 45000) {
            Serial.println(" FAIL - Timeout waiting for Cell Tower");
            return false;
        }
    }
    Serial.println(" ATTACHED");

    Serial.print("[LTE] Connecting GPRS Data...");
    if (!modem.gprsConnect(apn, "", "")) {
        Serial.println(" FAIL - GPRS Connection Refused");
        return false;
    }
    Serial.println(" OK");

    return true;
}

// -------------------------------------------------------------------------
// SLEEP LOGIC
// -------------------------------------------------------------------------
void enterSleep() {
    Serial.println("\n[SLEEP] >>> ENTERING SLEEP SEQUENCE <<<");
    Serial.print("[SLEEP] Current State: ");
    Serial.println(isArmed ? "ARMED" : "DISARMED");

    logEvent("SLEEP_ENTER", isStolen ? "recovery" : (isArmed ? "armed" : "disarmed"));

    publishStatus(isArmed ? "Sleeping_Armed" : "Sleeping_Disarmed");

    sensors_event_t a, g, t;
    mpu.getEvent(&a, &g, &t);
    delay(50);

    Serial.println("[SLEEP] Terminating GPS...");
    modem.sendAT("+CGNSPWR=0");
    delay(500);

    Serial.println("[SLEEP] Shutting down Cellular Modem...");
    if (!modem.poweroff()) {
        Serial.println("[SLEEP] WARNING: Modem unresponsive! Executing physical hardware kill...");
        digitalWrite(PWR_PIN, HIGH);
        delay(1500);
        digitalWrite(PWR_PIN, LOW);
    }
    delay(2000);

    Serial.println("[SLEEP] Turning LED OFF...");
    digitalWrite(LED_PIN, LOW);
    gpio_hold_en((gpio_num_t)LED_PIN);

    Serial.println("[SLEEP] Enabling Wakeup Triggers...");
    if (isArmed) {
        gpio_wakeup_enable((gpio_num_t)MPU_INT_PIN, GPIO_INTR_LOW_LEVEL);
    } else {
        gpio_wakeup_disable((gpio_num_t)MPU_INT_PIN);
    }
    // Only arm the NFC wake line if the reader is actually present.
    if (nfcPresent) {
        gpio_wakeup_enable((gpio_num_t)NFC_IRQ_PIN, GPIO_INTR_LOW_LEVEL);
    } else {
        gpio_wakeup_disable((gpio_num_t)NFC_IRQ_PIN);
    }
    esp_sleep_enable_gpio_wakeup();

    // --- DYNAMIC RECOVERY TIMER ---
    if (isStolen) {
        Serial.println("[SLEEP] Recovery Mode Active - Setting 30-Minute Wakeup.");
        esp_sleep_enable_timer_wakeup(1800000000ULL);
    } else {
        esp_sleep_enable_timer_wakeup(3600000000ULL);
    }

    for(int i = 0; i < 3; i++) {
        mpu.getMotionInterruptStatus();
        delay(20);
    }

    Serial.flush();
    Serial.println("[SLEEP] Goodnight.");
    esp_light_sleep_start();

    // =========================================================================
    // --- WAKE UP HAPPENS HERE ---
    // =========================================================================

    gpio_hold_dis((gpio_num_t)LED_PIN);
    digitalWrite(LED_PIN, HIGH);

    Serial.println("\n[WAKE] >>> I AM AWAKE <<<");
    flashWakeSequence();

    esp_sleep_wakeup_cause_t wakeup_reason = esp_sleep_get_wakeup_cause();

    logEvent("WAKE", "cause=" + String((int)wakeup_reason) +
                      " stolen=" + String(isStolen) +
                      " armed=" + String(isArmed));

    for(int i = 0; i < 3; i++) {
        mpu.getMotionInterruptStatus();
        delay(20);
    }

    if (wakeup_reason == ESP_SLEEP_WAKEUP_TIMER) {
        Serial.println(isStolen ? "[WAKE] 30-MIN RECOVERY PING: Reconnecting..." : "[WAKE] 1-HOUR HEARTBEAT: Reconnecting...");
        initializeLTE();
        connectMQTT();

        if (isStolen) {
            publishStatus("RECOVERY_MODE_PING");
            Serial.println("[GPS] Getting Recovery Coordinates...");
            float lat, lon, speed, alt, acc; int vs, us, yr, mo, dy, hr, mi, sc;
            if (modem.getGPS(&lat, &lon, &speed, &alt, &vs, &us, &acc, &yr, &mo, &dy, &hr, &mi, &sc)) {
                 String gpsData = String(speed, 2) + "," + String(lat, 6) + "," + String(lon, 6) + "," + String(alt, 2);
                 if (mqttConnected) { mqtt.publish(gps_topic.c_str(), gpsData.c_str()); mqttDelay(500); }
                 Serial.println("[GPS] Recovery Ping Sent.");
            } else {
                 Serial.println("[GPS] Recovery Ping Failed - No Fix.");
            }
        } else {
            publishStatus(isArmed ? "HEARTBEAT_ARMED" : "HEARTBEAT_DISARMED");
        }

        Serial.println("[WAKE] Requesting missed commands...");
        mqtt.publish((cmd_topic + "/get").c_str(), "");

        unsigned long listenStart = millis();
        while (millis() - listenStart < 10000) {
            mqtt.loop();
            delay(200);
        }

        sendTelemetry();
        lastActivityTime = millis();
        immediateResleep = true;   // heartbeat done - go straight back to sleep, don't idle

    } else {
        immediateResleep = false;  // physical/NFC wake - keep the 5-min interaction window
        bool validTagFound = false;

        if (isArmed) {
            if (nfcPresent) {
                Serial.println("[WAKE] ARMED WAKE: Starting 10-second NFC Grace Period...");
                unsigned long graceStart = millis();
                unsigned long lastGraceFlash = 0;
                bool ledState = false;

                while (millis() - graceStart < 10000) {
                    if (millis() - lastGraceFlash > 100) {
                        lastGraceFlash = millis();
                        ledState = !ledState;
                        digitalWrite(LED_PIN, ledState);
                    }
                    uint8_t uid[10] = {0};
                    uint8_t uidLen;
                    if (nfc.readPassiveTargetID(PN532_MIFARE_ISO14443A, uid, &uidLen, 100)) {
                        validTagFound = true;
                        digitalWrite(LED_PIN, LOW);
                        break;
                    }
                }
            } else {
                Serial.println("[WAKE] ARMED WAKE: No NFC reader - disarm via MQTT only.");
            }
        } else {
            if (nfcPresent) {
                Serial.println("[WAKE] DISARMED WAKE: Quick NFC check...");
                uint8_t uid[10] = {0};
                uint8_t uidLen;
                if (nfc.readPassiveTargetID(PN532_MIFARE_ISO14443A, uid, &uidLen, 1000)) {
                    validTagFound = true;
                }
            } else {
                Serial.println("[WAKE] DISARMED WAKE: No NFC reader.");
            }
        }

        digitalWrite(LED_PIN, LOW);

        String pendingStatusMsg = "";

        if (validTagFound) {
            if (theftActive || isStolen) {
                theftActive = false; isArmed = false; isStolen = false;
                pendingStatusMsg = "WAKE_ALARM_CLEARED";
            } else {
                isArmed = !isArmed;
                pendingStatusMsg = isArmed ? "WAKE_ARMED" : "WAKE_DISARMED";
            }
            commandFeedback(isArmed);

        } else {
            if (isArmed) {
                Serial.println("[WAKE] THEFT TRIGGERED!");
                theftActive = true;
                pendingStatusMsg = "WAKE_THEFT_TRIGGERED";
                logEvent("THEFT_TRIGGERED", "source=wake_gpio");
            } else {
                Serial.println("[WAKE] SYSTEM AWAKE (Disarmed Physical Trigger)");
                pendingStatusMsg = "SYSTEM_AWAKE";
            }
        }

        Serial.println("[WAKE] Rebuilding Cellular Connection to sync with cloud...");
        if (!modem.isGprsConnected()) initializeLTE();
        if (!mqtt.connected()) connectMQTT();

        publishStatus(pendingStatusMsg);
        sendTelemetry();

        lastActivityTime = millis();
    }

    if (!modem.isGprsConnected()) initializeLTE();
    digitalWrite(LED_PIN, LOW);
    absoluteWakeTime = millis();
    lastSuccessfulScan = millis();
}

// -------------------------------------------------------------------------
// SERIAL & LED
// -------------------------------------------------------------------------
void handleSerialCommands() {
    if (Serial.available()) {
        String cmd = Serial.readStringUntil('\n');
        cmd.trim(); cmd.toLowerCase();
        if (cmd.length() == 0) return;   // ignore stray serial noise (the rogue byte)
        lastActivityTime = millis();
        Serial.print("[CMD] Received: "); Serial.println(cmd);
        if (cmd == "learn") {
            learnMode = true; learnStartTime = millis();
            publishStatus("LEARN_Mode_Active");
        } else if (cmd == "dumplog") {
            dumpLogToSerial();
        } else if (cmd == "clearlog") {
            logClear();
        }
    }
    if (learnMode && millis() - learnStartTime > learnTimeout) {
        Serial.println("[CMD] Learn Mode Timed Out");
        learnMode = false;
    }
}

void updateLED() {
  unsigned long now = millis();
  if (learnMode || theftActive) {
    if (now - lastFastBlink > 100) {
        lastFastBlink = now;
        digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    }
  } else if (isArmed) {
    if (now - lastSlowBlink > 5000) {
        lastSlowBlink = now; digitalWrite(LED_PIN, HIGH);
    } else if (now - lastSlowBlink > 300) {
        digitalWrite(LED_PIN, LOW);
    }
  } else {
    if (!learnMode && !theftActive && !isArmed && (millis() - lastActivityTime < sleepTimeout)) {
       digitalWrite(LED_PIN, LOW);
    }
  }
}

// -------------------------------------------------------------------------
// SETUP
// -------------------------------------------------------------------------
void setup() {
    Serial.begin(115200);
    delay(1000);
    Serial.println("\n\n====================================");
    Serial.println("   MOTO TRACKER V7.61 - LOG FIX   ");
    Serial.println("====================================");

    // GPIO35 battery ADC: pin sees ~0-2.1V through the 2:1 divider at 4.2V cell.
    // Configured before logInit()/the first logEvent() so even the boot-time
    // battery reading (embedded in the BOOT log line) uses a calibrated channel
    // instead of whatever the ADC defaults to pre-configuration.
    analogReadResolution(12);
    analogSetPinAttenuation(BAT_ADC_PIN, ADC_11db);

    logInit();
    logEvent("BOOT", resetReasonToStr(esp_reset_reason()));

    pinMode(LED_PIN, OUTPUT);
    flashWakeSequence();
    Serial.println("[SETUP] LED Configured & Boot Sequence Started");

    pinMode(PWR_PIN, OUTPUT); digitalWrite(PWR_PIN, LOW);
    pinMode(MPU_INT_PIN, INPUT_PULLUP);
    pinMode(NFC_IRQ_PIN, INPUT_PULLUP);

    if (initializeLTE()) {
        mqtt.setServer(mqtt_server, mqtt_port);
        mqtt.setKeepAlive(60);       // was default 15s -> caused the flap
        mqtt.setSocketTimeout(30);   // give the cellular link breathing room
        mqtt.setCallback(mqttCallback);
        connectMQTT();
        getInitialGpsLock();
    }

    Wire.begin(21, 22);
    bool hardwareError = false;

    Serial.print("[SETUP] Initializing NFC...");
    if (nfc.begin()) {
        nfc.SAMConfig();
        nfcPresent = true;
        Serial.println(" OK (PN532 Found)");
    } else {
        nfcPresent = false;
        Serial.println(" NOT FOUND - running without NFC (use MQTT ARM/DISARM)");
    }
    if (mqtt.connected()) mqtt.loop();   // keep keepalive current after blocking init

    Serial.print("[SETUP] Initializing MPU6050...");
    if (mpu.begin()) {
        mpu.setHighPassFilter(MPU6050_HIGHPASS_DISABLE);
        mpu.setMotionDetectionThreshold(3); // SENSITIVITY SETTING
        mpu.setMotionDetectionDuration(5);  // DURATION OF TRIGGER SETTING
        mpu.setInterruptPinPolarity(true);
        mpu.setMotionInterrupt(true);
        mpu.setInterruptPinLatch(true);
        Serial.println(" OK (Motion Configured)");
    } else {
        Serial.println(" FAIL (MPU6050 Missing)"); hardwareError = true;
    }
    if (mqtt.connected()) mqtt.loop();   // keep keepalive current after blocking init

    sendTelemetry();
    digitalWrite(LED_PIN, LOW);
    lastActivityTime = millis();
    absoluteWakeTime = millis();

    if (hardwareError) publishStatus("BOOT_HARDWARE_FAIL");
    else if (theftActive || isStolen) publishStatus("RECOVERY_TRACKING_RESUMED");
    else publishStatus("SYSTEM_READY");

    Serial.println("[SETUP] Initialization Complete. Entering Loop.\n");
}

// -------------------------------------------------------------------------
// LOOP
// -------------------------------------------------------------------------
void loop() {
    handleSerialCommands();
    if (mqttConnected) mqtt.loop();
    updateLED();

    static unsigned long lastLteRetry = 0;
    if (!modem.isGprsConnected() && (millis() - lastLteRetry > 60000)) {
        Serial.println("\n[LOOP] Cellular connection dead. Retrying LTE...");
        logEvent("LTE_RETRY", "gprs_down");
        modem.gprsDisconnect(); delay(2000);
        initializeLTE();
        lastLteRetry = millis();
    }
    if (!mqtt.connected() && modem.isGprsConnected()) connectMQTT();

    // --- BATTERY SAFETY OVERRIDE ---
    if (batteryCritical) {
        Serial.println("\n[BATT] Critical flag set - entering safe sleep.");
        enterSleep();
        batteryCritical = false;   // re-evaluated on next telemetry read
    }

    // --- SLEEP & FAILSAFE LOGIC ---
    if (!theftActive) {
        if (immediateResleep) {
            immediateResleep = false;
            Serial.println("\n[SLEEP] Heartbeat complete - returning to sleep now.");
            enterSleep();
        } else if (millis() - lastActivityTime > sleepTimeout) {
            enterSleep();
        } else if (millis() - absoluteWakeTime > hardKillTimeout) {
            Serial.println("\n[FAILSAFE] ABSOLUTE TIMEOUT REACHED (6 Min). FORCING SLEEP!");
            publishStatus("FAILSAFE_FORCED_SLEEP");
            enterSleep();
        }
    } else {
        // --- 30-Minute maximum awake time triggers RECOVERY MODE ---
        if (millis() - absoluteWakeTime > 1800000) {
            Serial.println("\n[FAILSAFE] THEFT TIMEOUT (30 Min). ACTIVATING RECOVERY MODE!");
            publishStatus("THEFT_PULSE_SLEEP");
            logEvent("RECOVERY_MODE_ENTER", "theft_timeout_30min");
            theftActive = false;
            isArmed = true;
            isStolen = true; // ACTIVATE RECOVERY FLAG
            enterSleep();
        }
    }

    if (motionLatched && (millis() - lastMotionTime >= 3000)) {
        mpu.getMotionInterruptStatus();
        motionLatched = false;
        Serial.println("[MPU] 3-second timeout reached. Latch cleared.");
    }

    // --- NFC TAG POLLING (skipped entirely when reader is detached) ---
    if (nfcPresent) {
        static unsigned long lastTagCheck = 0;
        if (millis() - lastTagCheck > 350) {
            lastTagCheck = millis();
            uint8_t uid[10]; uint8_t uidLen;
            if (nfc.readPassiveTargetID(PN532_MIFARE_ISO14443A, uid, &uidLen, 100)) {
                lastActivityTime = millis();
                String hex = "";
                for (uint8_t i = 0; i < uidLen; i++) {
                    if (uid[i] < 0x10) hex += "0";
                    hex += String(uid[i], HEX);
                }
                hex.toUpperCase();
                Serial.print("[NFC] Scanned: "); Serial.println(hex);

                if (learnMode) {
                    publishStatus("New_Tag_Learned_" + hex);
                    learnMode = false;
                } else {
                    if (millis() - lastSuccessfulScan > scanDebounce) {
                        if (theftActive || isStolen) {
                            theftActive = false; isArmed = false; isStolen = false;
                            digitalWrite(LED_PIN, LOW);
                            publishStatus("ALARM_CLEARED");
                            Serial.println("[NFC] ALARM DISARMED BY TAG");
                            commandFeedback(isArmed);
                        } else {
                            isArmed = !isArmed;
                            if (isArmed) armTime = millis();
                            digitalWrite(LED_PIN, LOW);
                            publishStatus(isArmed ? "ARMED_LOCAL" : "DISARMED_LOCAL");
                            Serial.print("[NFC] System State Toggled: ");
                            Serial.println(isArmed ? "ARMED" : "DISARMED");
                            commandFeedback(isArmed);
                        }
                        sendTelemetry();
                        lastSuccessfulScan = millis();
                    }
                }
            }
        }
    }

    if (isArmed && !theftActive && (millis() - armTime > 3000)) {
        if (mpu.getMotionInterruptStatus()) {
            lastActivityTime = millis();
            theftActive = true;
            motionLatched = true;
            lastMotionTime = millis();

            Serial.println("\n[MPU] HARDWARE MOTION DETECTED!");
            Serial.println("!!! THEFT ALARM TRIGGERED !!!");

            logEvent("THEFT_TRIGGERED", "source=mpu_motion");
            publishStatus("THEFT_DETECTED");
            sendTelemetry();
        }
    }

    if (theftActive && millis() - lastGpsSent > 20000) {
        lastGpsSent = millis();
        lastActivityTime = millis();
        float lat, lon, speed, alt, acc;
        int vs, us, yr, mo, dy, hr, mi, sc;

        Serial.println("[GPS] Reading Coords for Tracking...");
        if (modem.getGPS(&lat, &lon, &speed, &alt, &vs, &us, &acc, &yr, &mo, &dy, &hr, &mi, &sc)) {
             String gpsData = String(speed, 2) + "," + String(lat, 6) + "," + String(lon, 6) + "," + String(alt, 2);
             if (mqttConnected) {
                 mqtt.publish(gps_topic.c_str(), gpsData.c_str());
                 mqttDelay(1200);
             }
             Serial.print("[GPS-TRACK] Sent: ");
             Serial.println(gpsData);
        } else {
             Serial.println("[GPS] Failed to read location.");
        }
    }
}

// -------------------------------------------------------------------------
// MQTTCALLBACK
// -------------------------------------------------------------------------
void mqttCallback(char* topic, byte* payload, unsigned int length) {
    String rawMsg = "";
    for (unsigned int i = 0; i < length; i++) {
        if (isPrintable(payload[i])) rawMsg += (char)payload[i];
    }
    rawMsg.trim(); rawMsg.toUpperCase();
    if (rawMsg.length() == 0 || rawMsg == "IDLE") return;

    Serial.print("[MQTT] Received: "); Serial.println(rawMsg);

    mqtt.publish(cmd_topic.c_str(), "IDLE");

    if (rawMsg == "DISARM") {
        if (!isArmed && !isStolen) { Serial.println("[MQTT] Already DISARMED."); return; }
        Serial.println("[MQTT] Executing DISARM");
        isArmed = false; theftActive = false; isStolen = false; digitalWrite(LED_PIN, LOW);
        commandFeedback(isArmed); sendTelemetry(); publishStatus("DISARMED_REMOTE");
    }
    else if (rawMsg == "ARM") {
        if (isArmed) { Serial.println("[MQTT] Already ARMED."); return; }
        Serial.println("[MQTT] Executing ARM");
        isArmed = true; armTime = millis();
        commandFeedback(isArmed); sendTelemetry(); publishStatus("ARMED_REMOTE");
    }
    else if (rawMsg == "GPS" || rawMsg == "LOCATE") {
        Serial.println("[MQTT] Executing GPS Query");

        publishStatus(isArmed ? "GPS_QUERY_ARMED" : "GPS_QUERY_DISARMED");

        float lat, lon, speed, alt, acc; int vs, us, yr, mo, dy, hr, mi, sc;
        if (modem.getGPS(&lat, &lon, &speed, &alt, &vs, &us, &acc, &yr, &mo, &dy, &hr, &mi, &sc)) {
             String gpsData = String(speed, 2) + "," + String(lat, 6) + "," + String(lon, 6) + "," + String(alt, 2);
             if (mqttConnected) { mqtt.publish(gps_topic.c_str(), gpsData.c_str()); mqttDelay(500); }

             publishStatus(isArmed ? "GPS_SENT_ARMED" : "GPS_SENT_DISARMED");
        } else {
             publishStatus(isArmed ? "GPS_FAIL_ARMED" : "GPS_FAIL_DISARMED");
        }
    }
    else if (rawMsg == "DUMPLOG") {
        Serial.println("[MQTT] Executing DUMPLOG");
        dumpLogToMqtt();
    }
    else if (rawMsg == "CLEARLOG") {
        Serial.println("[MQTT] Executing CLEARLOG");
        logClear();
        publishStatus("LOG_WIPED");
    }
}
