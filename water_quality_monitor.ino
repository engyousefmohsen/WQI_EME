/*
 * ============================================================================
 *  Water Quality Monitoring System (ESP32)
 * ============================================================================
 *  What this firmware does, in one wake cycle:
 *    1. Wakes up from deep sleep and connects to Wi-Fi (via WiFiManager).
 *    2. Occasionally checks GitHub for a newer firmware and updates itself (OTA).
 *    3. Syncs the clock (needed for TLS certificate validation).
 *    4. Starts several FreeRTOS tasks:
 *         - mqtt         : connects to the MQTT broker and publishes data
 *         - read_sensors : reads LDR / potentiometer / pH and computes the WQI
 *         - alert_task   : drives LEDs + buzzer depending on water quality
 *         - print        : shows the readings on the OLED display
 *         - sleep_task   : waits for everything to finish, then deep-sleeps
 *    5. Goes to deep sleep for SLEEP_SECONDS, then the whole cycle repeats.
 *
 *  WQI (Water Quality Index) here is a "badness" score: 0 = perfect, 100 = worst.
 *    WQI = 0.5 * turbidity% + 0.5 * pH penalty%
 * ============================================================================
 */

// ------------------------------ Libraries ----------------------------------
#include <WiFi.h>              // ESP32 Wi-Fi driver
#include <WiFiClientSecure.h>  // TLS (HTTPS / MQTTS) client
#include <PubSubClient.h>      // MQTT client
#include <Wire.h>              // I2C bus (used by the OLED)
#include <Adafruit_GFX.h>      // Graphics base library for the OLED
#include <Adafruit_SSD1306.h>  // SSD1306 OLED driver
#include <WiFiManager.h>       // Captive portal to enter Wi-Fi credentials
#include <HTTPClient.h>        // HTTP client (OTA version check)
#include <HTTPUpdate.h>        // Downloads and flashes new firmware (OTA)
#include <esp_task_wdt.h>      // Task watchdog timer

// ------------------------------ OLED size ----------------------------------
#define SCREEN_H 64            // OLED height in pixels
#define SCREEN_W 128           // OLED width in pixels

// ------------------------------ Pin mapping --------------------------------
#define LDR_PIN   34           // Light sensor (used as a turbidity/clarity sensor), analog input
#define GREEN_LED 4            // "Water is OK" indicator
#define RED_LED   23           // "Water is critical" indicator
#define BUZZER    25           // Alarm buzzer
#define POT_PIN   39           // Potentiometer (simulates a salinity / pH sensor), analog input

// ------------------------------ Event group bits ---------------------------
// An event group is a set of flags that tasks use to signal each other.
#define WIFI_UP_BIT     BIT0   // Set while Wi-Fi is connected and has an IP
#define MQTT_UP_BIT     BIT1   // Set while the MQTT broker connection is up
#define ALERT_DONE_BIT  BIT2   // Set by alert_task when its LED/buzzer pattern has finished
#define SLEEP_NOW_BIT   BIT3   // Set by sleep_task to ask the MQTT task to disconnect
#define MQTT_CLOSED_BIT BIT4   // Set by the MQTT task to confirm it has disconnected

// ------------------------------ Thresholds & timing ------------------------
#define WQI_WARNING   30.0f    // WQI at or above this = WARNING
#define WQI_ALERT     70.0f    // WQI above this = CRITICAL (triggers alarm + MQTT alert)
#define SLEEP_SECONDS 60       // How long to deep-sleep between cycles
#define AWAKE_MAX_MS  90000    // Failsafe: never stay awake longer than 90 s

// If true, pH is typed in over the Serial monitor (simulation).
// If false, pH is derived from the potentiometer reading.
#define SERIAL_PH true

// ================= OTA SETTINGS =================
// Firmware version of THIS build.
// Change this number every time you upload a new build to GitHub.
const char* FW_VERSION = "1.0.1";

// URL of a plain text file that contains ONLY the latest version number, e.g. 1.0.1
const char* versionUrl  = "https://raw.githubusercontent.com/engyousefmohsen/WQI_EME/refs/heads/main/version.txt";
// URL of the compiled firmware binary (latest GitHub release)
const char* firmwareUrl = "https://github.com/engyousefmohsen/WQI_EME/releases/latest/download/firmware.bin";

// Only check for OTA updates every N boots (plus on the first boot) to save time and power
#define OTA_CHECK_EVERY_N_BOOTS 5
// =================================================

// OLED display object (I2C, no reset pin)
Adafruit_SSD1306 display(SCREEN_W, SCREEN_H, &Wire, -1);

// Name of this device (not currently used in the MQTT topics)
const char* DEVICE_ID = "ESP32-1 Zagazig Team";

// ------------------------------ MQTT broker settings -----------------------
const char* server = "emqxserver";  // EMQX Cloud broker address
const int   port   = 8883;                                    // 8883 = MQTT over TLS
const char* user   = "lzka";                                  // MQTT username
const char* pass_q = "87654321";                              // MQTT password

// Root CA certificate (DigiCert Global Root G2) used to verify the broker's TLS identity
const char* CA_CERT = R"EOF(-----BEGIN CERTIFICATE-----
MIIDjjCCAnagAwIBAgIQAzrx5qcRqaC7KGSxHQn65TANBgkqhkiG9w0BAQsFADBh
MQswCQYDVQQGEwJVUzEVMBMGA1UEChMMRGlnaUNlcnQgSW5jMRkwFwYDVQQLExB3
d3cuZGlnaWNlcnQuY29tMSAwHgYDVQQDExdEaWdpQ2VydCBHbG9iYWwgUm9vdCBH
MjAeFw0xMzA4MDExMjAwMDBaFw0zODAxMTUxMjAwMDBaMGExCzAJBgNVBAYTAlVT
MRUwEwYDVQQKEwxEaWdpQ2VydCBJbmMxGTAXBgNVBAsTEHd3dy5kaWdpY2VydC5j
b20xIDAeBgNVBAMTF0RpZ2lDZXJ0IEdsb2JhbCBSb290IEcyMIIBIjANBgkqhkiG
9w0BAQEFAAOCAQ8AMIIBCgKCAQEAuzfNNNx7a8myaJCtSnX/RrohCgiN9RlUyfuI
2/Ou8jqJkTx65qsGGmvPrC3oXgkkRLpimn7Wo6h+4FR1IAWsULecYxpsMNzaHxmx
1x7e/dfgy5SDN67sH0NO3Xss0r0upS/kqbitOtSZpLYl6ZtrAGCSYP9PIUkY92eQ
q2EGnI/yuum06ZIya7XzV+hdG82MHauVBJVJ8zUtluNJbd134/tJS7SsVQepj5Wz
tCO7TG1F8PapspUwtP1MVYwnSlcUfIKdzXOS0xZKBgyMUNGPHgm+F6HmIcr9g+UQ
vIOlCsRnKPZzFBQ9RnbDhxSJITRNrw9FDKZJobq7nMWxM4MphQIDAQABo0IwQDAP
BgNVHRMBAf8EBTADAQH/MA4GA1UdDwEB/wQEAwIBhjAdBgNVHQ4EFgQUTiJUIBiV
5uNu5g/6+rkS7QYXjzkwDQYJKoZIhvcNAQELBQADggEBAGBnKJRvDkhj6zHd6mcY
1Yl9PMWLSn/pvtsrF9+wX3N3KjITOYFnQoQj8kVnNeyIv/iPsGEMNKSuIEyExtv4
NeF22d+mQrvHRAiGfzZ0JFrabA0UWTW98kndth/Jsw1HKj2ZL7tcu7XUIOGZX1NG
Fdtom/DzMNU+MeKNhJ7jitralj41E6Vf8PlwUHBHQRFXGU7Aj64GxJUTFy8bJZ91
8rGOmaFvE7FBcf6IKshPECBV1/MUReXgRPTqh5Uykw7+U0b6LJ3/iyK5S9kJRaTe
pLiaWN0bfVKfjllDiIGknibVb63dDcY3fe0Dkhvld1927jyNxF1WW6LZZm6zNTfl
MrY=
-----END CERTIFICATE-----)EOF";

// Secure network client + MQTT client built on top of it
WiFiClientSecure esp;
PubSubClient client(esp);

// Queues used to pass data between tasks (each holds exactly 1 item, see setup()):
//   displayQueue : sensor data  -> OLED task
//   readQueue    : sensor data  -> alert task
//   MQTTQueue    : sensor data  -> MQTT task
//   alertQueue   : alert text   -> MQTT task
QueueHandle_t displayQueue, readQueue, MQTTQueue, alertQueue;

// Event group holding the status flags defined above
EventGroupHandle_t events;

// Boot counter stored in RTC memory, so it survives deep sleep (but not a power loss)
RTC_DATA_ATTR int bootCount = 0;

// Set to true right before deep sleep so the Wi-Fi handler doesn't try to reconnect
volatile bool sleeping = false;

// Timestamp (millis) when this awake cycle started; used by the failsafe in loop()
uint32_t awakeStart = 0;

// One complete set of readings, passed between tasks through the queues
typedef struct {
  int bootcount = 0;   // Boot number when the reading was taken
  int ldr_value = 0;   // Raw LDR ADC value (0-4095)
  int pot_value = 0;   // Raw potentiometer ADC value (0-4095)
  int last_read = 0;   // Unused
  float WQI = 0;       // Computed Water Quality Index (0 = good, 100 = bad)
  float ph = 0;        // pH value (0-14)
  float turb = 0;      // Turbidity as a percentage (0-100)
} message;

// Alert text sent to the MQTT task. Only the MQTT task touches `client`,
// so other tasks hand it the text through a queue instead of publishing directly.
typedef struct {
  char text[96];
} AlertMsg;

// Shared reading structure filled in by read_sensors()
message msg;

// ================= OTA FUNCTIONS =================

// Shows up to two lines of text on the OLED (used for OTA progress messages)
void oledStatus(const char* l1, const char* l2 = "") {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println(l1);
  display.println(l2);
  display.display();
}

// Returns true if the version on GitHub is different from FW_VERSION
bool updateAvailable() {
  WiFiClientSecure otaClient;
  otaClient.setInsecure();   // Skips TLS certificate verification (convenient, but less secure)

  HTTPClient http;
  http.setFollowRedirects(HTTPC_FORCE_FOLLOW_REDIRECTS);   // GitHub may redirect, so follow it
  if (!http.begin(otaClient, versionUrl)) return false;    // Could not start the request

  int code = http.GET();                                   // Download version.txt
  if (code != HTTP_CODE_OK) {
    Serial.printf("Version check failed, HTTP %d\n", code);
    http.end();
    return false;
  }

  String latest = http.getString();   // Body of the file = latest version number
  http.end();
  latest.trim();                      // Remove newline / spaces

  Serial.printf("OTA -> Current: %s  Latest: %s\n", FW_VERSION, latest.c_str());
  // Update only if the file is not empty and differs from the running version
  return latest.length() > 0 && latest != FW_VERSION;
}

// Called repeatedly during the firmware download to show progress on the OLED
void onOtaProgress(int cur, int total) {
  static int lastPct = -1;                          // Remembers the last percentage shown
  int pct = (total > 0) ? (cur * 100 / total) : 0;  // Percent downloaded
  if (pct != lastPct && pct % 10 == 0) {            // Refresh only every 10% to avoid slowing the download
    lastPct = pct;
    char buf[24];
    snprintf(buf, sizeof(buf), "%d %%", pct);
    oledStatus("Updating firmware...", buf);
  }
}

// Downloads the firmware binary from GitHub and flashes it
void updateFromGitHub() {
  WiFiClientSecure otaClient;
  otaClient.setInsecure();   // Skips certificate verification

  httpUpdate.setFollowRedirects(HTTPC_FORCE_FOLLOW_REDIRECTS);  // GitHub release links redirect
  httpUpdate.rebootOnUpdate(true);                              // Reboot automatically after success
  httpUpdate.onProgress(onOtaProgress);                         // Register the progress callback

  oledStatus("New firmware found", "Downloading...");
  Serial.println("Downloading firmware...");
  t_httpUpdate_return ret = httpUpdate.update(otaClient, firmwareUrl);

  switch (ret) {
    case HTTP_UPDATE_FAILED:
      // Update failed: show the error and carry on running the current firmware
      Serial.printf("Update failed (%d): %s\n",
                    httpUpdate.getLastError(),
                    httpUpdate.getLastErrorString().c_str());
      oledStatus("Update failed", "Continuing...");
      delay(1500);
      break;
    case HTTP_UPDATE_NO_UPDATES:
      Serial.println("No update available");
      break;
    case HTTP_UPDATE_OK:
      Serial.println("Update OK, rebooting...");  // Normally never reached, the device reboots first
      break;
  }
}

// Decides whether it is time to check for an update, and applies it if one exists
void checkForOTA() {
  // Check on the first boot, then only every N boots
  if (bootCount != 1 && (bootCount % OTA_CHECK_EVERY_N_BOOTS) != 0) return;

  oledStatus("Checking for", "updates...");
  if (updateAvailable()) {
    updateFromGitHub();
  } else {
    Serial.println("Firmware is up to date");
  }
}
// =================================================

// Gets the current time from NTP servers.
// TLS needs a correct clock to check certificate validity dates, so this must run before setCACert/connect.
void syncTime() {
  configTime(0, 0, "pool.ntp.org", "time.nist.gov");   // UTC, no daylight saving offset
  // Wait up to 10 s (20 x 500 ms) until the time is a realistic value (after Nov 2023)
  for (int i = 0; i < 20 && time(nullptr) < 1700000000; i++) delay(500);
  Serial.printf("Time synced: %s\n", time(nullptr) > 1700000000 ? "yes" : "NO");
}

// Called by PubSubClient whenever a message arrives on a topic we subscribed to.
// Here it just prints the topic and payload to the Serial monitor.
void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message received [");
  Serial.print(topic);
  Serial.print("] : ");
  for (unsigned int i = 0; i < length; ++i)
    Serial.print((char)payload[i]);
  Serial.println();
}

// Wi-Fi event handler: keeps the WIFI_UP_BIT flag in sync with the real connection state
void onWiFiEvent(arduino_event_id_t event, arduino_event_info_t info) {
  switch (event) {
    case ARDUINO_EVENT_WIFI_STA_GOT_IP:
      // Connected and received an IP address -> tell other tasks Wi-Fi is available
      xEventGroupSetBits(events, WIFI_UP_BIT);
      Serial.printf("WiFi up, IP: %s \n", WiFi.localIP().toString().c_str());
      break;

    case ARDUINO_EVENT_WIFI_STA_DISCONNECTED:
      // Connection lost -> clear the flag so tasks stop using the network
      xEventGroupClearBits(events, WIFI_UP_BIT);
      Serial.printf("WiFi lost, reason=%d \n", info.wifi_sta_disconnected.reason);
      if (!sleeping) WiFi.reconnect();   // Try to reconnect, unless we are intentionally going to sleep
      break;

    default: break;
  }
}

// Connects to the MQTT broker. Keeps retrying (every 5 s) while Wi-Fi is up and MQTT is not.
void reconnect() {
  while (!client.connected() && (xEventGroupGetBits(events) & WIFI_UP_BIT)) {
    Serial.print("MQTT connecting to server... ");
    // Build a unique client ID from the chip's MAC address (the broker requires unique IDs)
    String id = "esp32-" + String((uint32_t)ESP.getEfuseMac(), HEX);
    if (client.connect(id.c_str(), user, pass_q)) {
      // Success: raise the flag, announce ourselves and subscribe to the input topic
      xEventGroupSetBits(events, MQTT_UP_BIT);
      Serial.println("Connected");
      client.publish("ZagazigTeam/outTopic", "ESPConnected");
      client.subscribe("ZagazigTeam/inTopic");
    } else {
      // Failure: clear the flag, print the error code, wait, then retry
      xEventGroupClearBits(events, MQTT_UP_BIT);
      Serial.print("Failed rc = ");
      Serial.println(client.state());
      Serial.println("Retrying in 5 seconds");
      vTaskDelay(pdMS_TO_TICKS(5000));
    }
  }
}

// ============================= MQTT TASK ====================================
// The ONLY task that touches `client` (PubSubClient is not thread-safe).
// It keeps the connection alive and publishes readings and alerts it receives through queues.
void mqtt(void* ar) {
  for (;;) {
    // Block here until Wi-Fi is up (flags are not cleared on exit, wait for ALL bits)
    xEventGroupWaitBits(events, WIFI_UP_BIT, pdFALSE, pdTRUE, portMAX_DELAY);

    // sleep_task asks us to close the connection cleanly before deep sleep
    if (xEventGroupGetBits(events) & SLEEP_NOW_BIT) {
      client.disconnect();                          // Close the MQTT session
      xEventGroupSetBits(events, MQTT_CLOSED_BIT);  // Confirm to sleep_task
      vTaskSuspend(NULL);                           // Stop this task (NULL = suspend myself)
    }

    // If the broker connection dropped, clear the flag and reconnect
    if (!client.connected()) {
      xEventGroupClearBits(events, MQTT_UP_BIT);
      reconnect();
    }
    client.loop();   // Process incoming MQTT traffic and send keep-alive pings

    // Still not connected: wait a moment instead of spinning in a tight loop,
    // and don't take the reading out of the queue (so it isn't lost)
    if (!client.connected()) {
      vTaskDelay(pdMS_TO_TICKS(100));
      continue;
    }

    // Wait up to 100 ms for a new set of readings and publish each value to its own topic
    message ms;
    if (xQueueReceive(MQTTQueue, &ms, pdMS_TO_TICKS(100)) == pdTRUE) {
      char buf[16];
      snprintf(buf, sizeof(buf), "%.3f", ms.ph);
      client.publish("water/PH", buf);       // pH value
      snprintf(buf, sizeof(buf), "%d", ms.ldr_value);
      client.publish("water/clear", buf);    // LDR value (water clarity)
      snprintf(buf, sizeof(buf), "%d", ms.pot_value);
      client.publish("water/salt", buf);     // Potentiometer value (salinity)
      snprintf(buf, sizeof(buf), "%.2f", ms.WQI);
      client.publish("water/WQI", buf);      // Water Quality Index
    }

    // Check (without waiting) whether an alert message is pending and publish it
    AlertMsg a;
    if (xQueueReceive(alertQueue, &a, 0) == pdTRUE) {
      client.publish("testtopic/alert", a.text);
    }
  }
}

// ============================= DISPLAY TASK =================================
// Shows a title, then the latest readings whenever new data arrives.
void print(void* ar) {
  // Draw the title once at startup
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println(F("Water Quality Monitoring System"));
  display.display();

  for (;;) {
    message reci;
    // Wait up to 50 ms for new data; if it arrives, append it below the title
    if (xQueueReceive(displayQueue, &reci, pdMS_TO_TICKS(50)) == pdTRUE) {
      display.printf("LDR : %d", reci.ldr_value);
      display.println();
      display.printf("POT : %d", reci.pot_value);
      display.println();
      display.printf("Boot counts : %d", reci.bootcount);
      display.println();
      display.printf("FW : %s", FW_VERSION);
      display.display();   // Push everything to the screen
    }
  }
}

// ============================ SENSOR TASK ===================================
// Runs once per wake cycle: reads sensors, computes the WQI, distributes the result, then deletes itself.
void read_sensors(void* ar) {
  // Wait up to 20 s for Wi-Fi AND MQTT to be up, so the data can be sent right away.
  // If the timeout passes we continue anyway.
  xEventGroupWaitBits(events, WIFI_UP_BIT | MQTT_UP_BIT, pdFALSE, pdTRUE, pdMS_TO_TICKS(20000));

  // Take the raw analog readings (0-4095 on the ESP32's 12-bit ADC)
  msg.ldr_value = analogRead(LDR_PIN);
  msg.pot_value = analogRead(POT_PIN);
  msg.bootcount = bootCount;

  if(SERIAL_PH) {
    // --- Simulation mode: the user types a pH value in the Serial monitor ---
    msg.ph = 7.0f;   // Default (neutral) if nothing is typed in time
    Serial.println("Please Enter a Number Simulating PH value! You have 5 Seconds");
    while (Serial.available()) Serial.read();   // Discard any old input in the buffer
    uint32_t current = millis();                // Start time of the 5 s window
    bool got = false;                           // Becomes true when a valid number is received
    while (!got && (millis() - current < 5000)) {
      if (Serial.available()) {
        String line = Serial.readStringUntil('\n');   // Read one line
        line.trim();
        if (line.length() > 0) {
          float v = line.toFloat();
          // Accept only if it starts like a number and is within the valid pH range 0-14
          if ((isdigit(line[0]) || line[0] == '.') && v >= 0.0f && v <= 14.0f) {
            msg.ph = v;
            got = true;
            Serial.printf("You sent : %.2f\n", msg.ph);
          } else {
            Serial.println("Enter a number from 0 to 14");
          }
        }
      }
      vTaskDelay(pdMS_TO_TICKS(20));   // Yield the CPU to other tasks
    }
  }
  else {
    // --- Real mode: map the potentiometer (0-4095) to a pH range (0-14) ---
    msg.ph = (msg.pot_value / 4095.0f) * 14.0f;
  }

  // Turbidity %: map the LDR reading (0-4095) to 0-100%
  msg.turb = (msg.ldr_value / 4095.0f) * 100.0f;
  // pH penalty %: how far pH is from neutral (7), scaled so that pH 0 or 14 = 100%
  float phPenalty = fabsf(msg.ph - 7.0f) / 7.0f * 100.0f;
  // Final index: equal-weight average of turbidity and pH penalty (higher = worse)
  msg.WQI = 0.5f * msg.turb + 0.5f * phPenalty;

  // Hand the same reading to all three consumers.
  // xQueueOverwrite always succeeds for a 1-item queue and replaces any old value.
  if (xQueueOverwrite(readQueue, &msg) == pdTRUE && xQueueOverwrite(displayQueue, &msg) == pdTRUE && xQueueOverwrite(MQTTQueue, &msg) == pdTRUE) {
    Serial.println("Sent : ");
    Serial.printf("--> LDR  = %d", msg.ldr_value);
    Serial.println();
    Serial.printf("--> Potentiometer = %d", msg.pot_value);
    Serial.println();
    Serial.printf("--> Boot count = %d", msg.bootcount);
    Serial.println();
    Serial.printf("--> WQI  = %f ", msg.WQI);
  } else {
    Serial.println("Couldn't Send readings!");
  }
  vTaskDelete(NULL);   // This task has finished its job, remove it (NULL = delete myself)
}

// ============================= ALERT TASK ===================================
// Waits for a new reading, classifies it (GOOD / WARNING / CRITICAL) and gives physical feedback.
void alert_task(void* ar) {
  for (;;) {
    message r;
    // Sleep until a reading arrives (portMAX_DELAY = wait forever)
    if (xQueueReceive(readQueue, &r, portMAX_DELAY) == pdTRUE) {
      float wqi = r.WQI;

      // Classify the water quality
      const char* status = (wqi > WQI_ALERT) ? "CRITICAL" : (wqi >= WQI_WARNING ? "WARNING" : "GOOD");
      Serial.printf("WQI = %.1f (pH %.2f, turb %.1f%%) -> %s\n", r.WQI, r.ph, r.turb, status);

      if (wqi > WQI_ALERT) {
        // --- CRITICAL: build a JSON alert and queue it for the MQTT task to publish ---
        AlertMsg a;
        snprintf(a.text, sizeof(a.text), "{\"alert\":\"CRITICAL\",\"wqi\":%.1f,\"ph\":%.2f,\"turbidity\":%.1f}", r.WQI, r.ph, r.turb);
        xQueueOverwrite(alertQueue, &a);

        // Flash the red LED and beep the buzzer 5 times (200 ms on / 200 ms off)
        for (int i = 0; i < 5; i++) {
          digitalWrite(RED_LED, HIGH);
          tone(BUZZER, 5000);                   // 5 kHz tone
          vTaskDelay(pdMS_TO_TICKS(200));
          digitalWrite(RED_LED, LOW);
          noTone(BUZZER);
          vTaskDelay(pdMS_TO_TICKS(200));
        }
        digitalWrite(RED_LED, HIGH);            // Leave the red LED on after the pattern
      } else {
        // --- GOOD or WARNING: make sure the alarm is off and blink green 5 times ---
        digitalWrite(RED_LED, LOW);
        noTone(BUZZER);
        for (int i = 0; i < 5; ++i) {
          digitalWrite(GREEN_LED, HIGH);
          vTaskDelay(pdMS_TO_TICKS(200));
          digitalWrite(GREEN_LED, LOW);
          vTaskDelay(pdMS_TO_TICKS(200));
        }
      }
      // Tell sleep_task that the alert pattern has finished
      xEventGroupSetBits(events, ALERT_DONE_BIT);
    }
  }
}

// ============================ DEEP SLEEP ====================================
// Shuts things down, shows a message and puts the ESP32 into deep sleep.
// The chip restarts from setup() when the timer expires.
void goToSleep() {
  sleeping = true;   // Stops the Wi-Fi event handler from reconnecting
  Serial.printf("Going to deep sleep for %d s...\n", SLEEP_SECONDS);
  Serial.flush();    // Make sure all serial text has been sent before sleeping

  WiFi.disconnect(true);       // Disconnect and turn the Wi-Fi radio off
  digitalWrite(RED_LED, LOW);  // Turn off the red LED
  noTone(BUZZER);              // Make sure the buzzer is silent

  // Show "Sleeping" and the boot count on the OLED
  display.clearDisplay();
  display.setTextSize(2);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println("Sleeping");
  display.printf("Boot Count : %d", bootCount);
  display.display();

  // Set the wake-up timer (the function expects microseconds) and enter deep sleep
  esp_sleep_enable_timer_wakeup((uint64_t)SLEEP_SECONDS * 1000000ULL);
  esp_deep_sleep_start();
}

// ============================= SLEEP TASK ===================================
// Coordinates the shutdown: waits for the alert to finish, closes MQTT cleanly, then sleeps.
void sleep_task(void* ar) {
  // Wait up to 40 s for the alert task to finish its LED/buzzer pattern
  xEventGroupWaitBits(events, ALERT_DONE_BIT, pdFALSE, pdTRUE, pdMS_TO_TICKS(40000));
  // Extra 5 s so the MQTT task has time to publish the last readings/alert
  vTaskDelay(pdMS_TO_TICKS(5000));

  // Ask the MQTT task to disconnect, then wait (up to 2 s) for its confirmation
  xEventGroupSetBits(events, SLEEP_NOW_BIT);
  xEventGroupWaitBits(events, MQTT_CLOSED_BIT, pdFALSE, pdTRUE, pdMS_TO_TICKS(2000));

  goToSleep();
}

// Called by WiFiManager when it starts its configuration access point.
// Shows the user how to connect and enter the Wi-Fi credentials.
void WiFiManager_Callback(WiFiManager* wm) {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println(F("Wi-Fi setup"));
  display.println();
  display.println(F("Join Wi-Fi:"));
  display.println(F("WaterMonitor-Setup"));   // Name of the access point created by the ESP32
  display.println();
  display.println(F("Open 192.168.4.1"));     // Default address of the config web page
  display.display();
}

// ============================= WATCHDOG =====================================
// If the main loop hangs for longer than this, the chip is reset automatically.
#define WDT_TIMEOUT_S 120

void startWatchdog() {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  // Arduino-ESP32 core 3.x: the watchdog is configured with a struct
  esp_task_wdt_config_t cfg = {
    .timeout_ms     = WDT_TIMEOUT_S * 1000,
    .idle_core_mask = 0,      // Don't watch the idle tasks, so existing behaviour is unchanged
    .trigger_panic  = true    // Reset the chip when the watchdog fires
  };
  esp_task_wdt_reconfigure(&cfg);
#else
  // Arduino-ESP32 core 2.x: simple init (timeout in seconds, panic on timeout)
  esp_task_wdt_init(WDT_TIMEOUT_S, true);
#endif
  // Subscribe the current task (loopTask) to the watchdog.
  // It never feeds the watchdog, so the chip resets after WDT_TIMEOUT_S as a last-resort failsafe.
  esp_task_wdt_add(NULL);
}

// ================================ SETUP =====================================
// Runs once at every boot and every wake-up from deep sleep.
void setup() {
  Serial.begin(115200);
  Wire.begin(21, 22);   // Start I2C: SDA = GPIO21, SCL = GPIO22 (for the OLED)
  bootCount++;          // Count this boot (kept in RTC memory across deep sleep)

  Serial.printf("\nFirmware version %s | boot #%d\n", FW_VERSION, bootCount);

  // Configure the pins
  pinMode(RED_LED, OUTPUT);
  pinMode(BUZZER, OUTPUT);
  pinMode(GREEN_LED, OUTPUT);
  pinMode(POT_PIN, INPUT);
  pinMode(LDR_PIN, INPUT);

  events = xEventGroupCreate();   // Create the flag group used for task synchronisation
  WiFi.mode(WIFI_STA);            // Station mode (connect to a router)

  // Initialise the OLED at I2C address 0x3C
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println("[Setup] OLED init FAILED – check wiring!");
  } else {
    display.clearDisplay();
    display.display();
  }

  // ----- Wi-Fi connection through WiFiManager -----
  WiFiManager wm;
  //wm.resetSettings();                      // Uncomment to erase saved Wi-Fi credentials
  wm.setAPCallback(WiFiManager_Callback);    // Show instructions on the OLED when the portal opens
  wm.setConnectTimeout(20);                  // Give up on a saved network after 20 s
  wm.setConfigPortalTimeout(180);            // Close the config portal after 180 s

  // The setup portal is only allowed on the very first boot.
  // On later wake-ups the device must never sit waiting for someone to configure it.
  if (bootCount > 1) {
    wm.setEnableConfigPortal(false);
  }

  // Try the saved network; open the portal if allowed and needed
  if (!wm.autoConnect("WaterMonitor-Setup")) {
    // Could not connect: sleep and try again on the next cycle
    Serial.println("No Wi-Fi, sleeping and trying again");
    esp_sleep_enable_timer_wakeup((uint64_t)SLEEP_SECONDS * 1000000ULL);
    esp_deep_sleep_start();
  }

  // Register the Wi-Fi event handler AFTER WiFiManager is done, so its
  // WiFi.reconnect() call can't interfere with WiFiManager's own connection attempts.
  WiFi.onEvent(onWiFiEvent);

  xEventGroupSetBits(events, WIFI_UP_BIT);   // Wi-Fi is up (the event handler missed this connection)
  Serial.println("\nWiFi Connected");

  // ===== OTA: after Wi-Fi and before MQTT and the tasks =====
  // If an update is found, the device flashes it and restarts by itself, so the code below is not reached.
  checkForOTA();
  // ==========================================================

  syncTime();   // Set the clock, required for TLS certificate validation

  // Configure the secure MQTT connection
  esp.setCACert(CA_CERT);              // Trust this root certificate
  client.setServer(server, port);      // Broker address and port
  client.setCallback(callback);        // Function to call for incoming messages

  awakeStart = millis();   // Remember when this awake period started (for the failsafe in loop())
  startWatchdog();         // Arm the hardware/software watchdog

  // Create the queues, each holds 1 item (always the latest value)
  MQTTQueue    = xQueueCreate(1, sizeof(message));
  displayQueue = xQueueCreate(1, sizeof(message));
  readQueue    = xQueueCreate(1, sizeof(message));
  alertQueue   = xQueueCreate(1, sizeof(AlertMsg));

  // Start the tasks: xTaskCreate(function, name, stack size in bytes, parameter, priority, handle)
  // A higher priority number means the task is scheduled first.
  xTaskCreate(mqtt,         "MQTT",  8192, NULL, 5, NULL);   // Highest priority: networking
  xTaskCreate(read_sensors, "read",  8192, NULL, 4, NULL);   // Reads sensors, then deletes itself
  xTaskCreate(print,        "write", 8192, NULL, 2, NULL);   // OLED updates
  xTaskCreate(alert_task,   "alert", 4096, NULL, 3, NULL);   // LEDs + buzzer
  xTaskCreate(sleep_task,   "sleep", 4096, NULL, 1, NULL);   // Lowest priority: shutdown coordinator
}

// ================================ LOOP ======================================
// All the real work is done by the tasks. loop() only acts as a failsafe timer.
void loop() {
  vTaskDelay(pdMS_TO_TICKS(1000));   // Check once per second
  // If we have been awake too long (something got stuck), force deep sleep
  if (millis() - awakeStart > AWAKE_MAX_MS) {
    Serial.println("Failsafe: awake too long, forcing sleep");
    goToSleep();
  }
}
