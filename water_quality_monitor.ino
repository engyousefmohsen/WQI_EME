#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <WiFiManager.h>
#include <HTTPClient.h>   // OTA
#include <HTTPUpdate.h>   // OTA
#include <esp_task_wdt.h>

#define SCREEN_H 64
#define SCREEN_W 128

#define LDR_PIN 34
#define GREEN_LED 4
#define RED_LED 23
#define BUZZER 25
#define POT_PIN 39

#define WIFI_UP_BIT  BIT0
#define MQTT_UP_BIT  BIT1
#define ALERT_DONE_BIT BIT2
#define SLEEP_NOW_BIT   BIT3   // FIX: sleep_task asks the MQTT task to disconnect
#define MQTT_CLOSED_BIT BIT4   // FIX: MQTT task confirms it has disconnected

#define WQI_WARNING   30.0f
#define WQI_ALERT     70.0f
#define SLEEP_SECONDS 60
#define AWAKE_MAX_MS  90000    // FIX: failsafe, never stay awake longer than this

#define SERIAL_PH true

// ================= OTA SETTINGS =================
// غيّري الرقم دة مع كل نسخة جديدة بترفعيها على GitHub
const char* FW_VERSION = "1.0.1";

// ملف نصي فيه رقم آخر نسخة بس، مثلا: 1.0.1
const char* versionUrl  = "https://raw.githubusercontent.com/engyousefmohsen/WQI_EME/refs/heads/main/version.txt";
const char* firmwareUrl = "https://github.com/engyousefmohsen/WQI_EME/releases/latest/download/firmware.bin";


#define OTA_CHECK_EVERY_N_BOOTS 5
// =================================================

Adafruit_SSD1306 display(SCREEN_W, SCREEN_H, &Wire, -1);

const char* DEVICE_ID = "ESP32-1 Zagazig Team";

const char* server = "y1257324.ala.eu-central-1.emqxsl.com";
const int   port   = 8883;
const char* user   = "lzka";
const char* pass_q = "87654321";

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

WiFiClientSecure esp;
PubSubClient client(esp);

QueueHandle_t displayQueue, readQueue, MQTTQueue, alertQueue;
EventGroupHandle_t events;

RTC_DATA_ATTR int bootCount = 0;

volatile bool sleeping = false;   // FIX: stops the Wi-Fi handler from reconnecting while going to sleep
uint32_t awakeStart = 0;          // FIX: start of the awake period, used by the failsafe in loop()

typedef struct {
  int bootcount = 0;
  int ldr_value = 0;
  int pot_value = 0;
  int last_read = 0;
  float WQI = 0;
  float ph = 0;
  float turb = 0;
} message;

// FIX: alert text is passed to the MQTT task, so only that task touches `client`
typedef struct {
  char text[96];
} AlertMsg;

message msg;

// ================= OTA FUNCTIONS =================
void oledStatus(const char* l1, const char* l2 = "") {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println(l1);
  display.println(l2);
  display.display();
}

// بترجع true لو النسخة على GitHub مختلفة عن FW_VERSION
bool updateAvailable() {
  WiFiClientSecure otaClient;
  otaClient.setInsecure();   // بيتخطى فحص الشهادة (شوفي الملاحظة)

  HTTPClient http;
  http.setFollowRedirects(HTTPC_FORCE_FOLLOW_REDIRECTS);
  if (!http.begin(otaClient, versionUrl)) return false;

  int code = http.GET();
  if (code != HTTP_CODE_OK) {
    Serial.printf("Version check failed, HTTP %d\n", code);
    http.end();
    return false;
  }

  String latest = http.getString();
  http.end();
  latest.trim();

  Serial.printf("OTA -> Current: %s  Latest: %s\n", FW_VERSION, latest.c_str());
  return latest.length() > 0 && latest != FW_VERSION;
}

void onOtaProgress(int cur, int total) {
  static int lastPct = -1;
  int pct = (total > 0) ? (cur * 100 / total) : 0;
  if (pct != lastPct && pct % 10 == 0) {
    lastPct = pct;
    char buf[24];
    snprintf(buf, sizeof(buf), "%d %%", pct);
    oledStatus("Updating firmware...", buf);
  }
}

void updateFromGitHub() {
  WiFiClientSecure otaClient;
  otaClient.setInsecure();

  httpUpdate.setFollowRedirects(HTTPC_FORCE_FOLLOW_REDIRECTS);
  httpUpdate.rebootOnUpdate(true);
  httpUpdate.onProgress(onOtaProgress);

  oledStatus("New firmware found", "Downloading...");
  Serial.println("Downloading firmware...");
  t_httpUpdate_return ret = httpUpdate.update(otaClient, firmwareUrl);

  switch (ret) {
    case HTTP_UPDATE_FAILED:
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
      Serial.println("Update OK, rebooting...");  // عادة مش بيوصل هنا
      break;
  }
}

void checkForOTA() {
  // أول بوت، وبعدين كل N بوت
  if (bootCount != 1 && (bootCount % OTA_CHECK_EVERY_N_BOOTS) != 0) return;

  oledStatus("Checking for", "updates...");
  if (updateAvailable()) {
    updateFromGitHub();
  } else {
    Serial.println("Firmware is up to date");
  }
}
// =================================================
void syncTime() {
  configTime(0, 0, "pool.ntp.org", "time.nist.gov");
  for (int i = 0; i < 20 && time(nullptr) < 1700000000; i++) delay(500);
  Serial.printf("Time synced: %s\n", time(nullptr) > 1700000000 ? "yes" : "NO");
}

void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message received [");
  Serial.print(topic);
  Serial.print("] : ");
  for (unsigned int i = 0; i < length; ++i)
    Serial.print((char)payload[i]);
  Serial.println();
}

void onWiFiEvent(arduino_event_id_t event, arduino_event_info_t info) {
  switch (event) {
    case ARDUINO_EVENT_WIFI_STA_GOT_IP:
      xEventGroupSetBits(events, WIFI_UP_BIT);
      Serial.printf("WiFi up, IP: %s \n", WiFi.localIP().toString().c_str());
      break;

    case ARDUINO_EVENT_WIFI_STA_DISCONNECTED:
      xEventGroupClearBits(events, WIFI_UP_BIT);
      Serial.printf("WiFi lost, reason=%d \n", info.wifi_sta_disconnected.reason);
      if (!sleeping) WiFi.reconnect();   // FIX: don't reconnect while going to sleep
      break;

    default: break;
  }
}

void reconnect() {
  while (!client.connected() && (xEventGroupGetBits(events) & WIFI_UP_BIT)) {
    Serial.print("MQTT connecting to server... ");
    String id = "esp32-" + String((uint32_t)ESP.getEfuseMac(), HEX);
    if (client.connect(id.c_str(), user, pass_q)) {
      xEventGroupSetBits(events, MQTT_UP_BIT);
      Serial.println("Connected");
      client.publish("ZagazigTeam/outTopic", "ESPConnected");
      client.subscribe("ZagazigTeam/inTopic");
    } else {
      xEventGroupClearBits(events, MQTT_UP_BIT);
      Serial.print("Failed rc = ");
      Serial.println(client.state());
      Serial.println("Retrying in 5 seconds");
      vTaskDelay(pdMS_TO_TICKS(5000));
    }
  }
}

// The ONLY task that touches `client`
void mqtt(void* ar) {
  for (;;) {
    xEventGroupWaitBits(events, WIFI_UP_BIT, pdFALSE, pdTRUE, portMAX_DELAY);

    // FIX: sleep_task asks us to close the connection (no other task touches `client`)
    if (xEventGroupGetBits(events) & SLEEP_NOW_BIT) {
      client.disconnect();
      xEventGroupSetBits(events, MQTT_CLOSED_BIT);
      vTaskSuspend(NULL);
    }

    if (!client.connected()) {
      xEventGroupClearBits(events, MQTT_UP_BIT);
      reconnect();
    }
    client.loop();

    // FIX: if not connected, wait a moment instead of spinning (and don't consume the reading)
    if (!client.connected()) {
      vTaskDelay(pdMS_TO_TICKS(100));
      continue;
    }

    message ms;
    if (xQueueReceive(MQTTQueue, &ms, pdMS_TO_TICKS(100)) == pdTRUE) {
      char buf[16];
      snprintf(buf, sizeof(buf), "%.3f", ms.ph);
      client.publish("water/PH", buf);
      snprintf(buf, sizeof(buf), "%d", ms.ldr_value);
      client.publish("water/clear", buf);
      snprintf(buf, sizeof(buf), "%d", ms.pot_value);
      client.publish("water/salt", buf);
      snprintf(buf, sizeof(buf), "%.2f", ms.WQI);
      client.publish("water/WQI", buf);
    }
    AlertMsg a;
    if (xQueueReceive(alertQueue, &a, 0) == pdTRUE) {
      client.publish("testtopic/alert", a.text);
    }
  }
}

void print(void* ar) {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println(F("Water Quality Monitoring System"));
  display.display();
  for (;;) {
    message reci;
    if (xQueueReceive(displayQueue, &reci, pdMS_TO_TICKS(50)) == pdTRUE) {
      display.printf("LDR : %d", reci.ldr_value);
      display.println();
      display.printf("POT : %d", reci.pot_value);
      display.println();
      display.printf("Boot counts : %d", reci.bootcount);
      display.println();
      display.printf("FW : %s", FW_VERSION);
      display.display();
    }
  }
}

void read_sensors(void* ar) {
  xEventGroupWaitBits(events, WIFI_UP_BIT | MQTT_UP_BIT, pdFALSE, pdTRUE, pdMS_TO_TICKS(20000));

  msg.ldr_value = analogRead(LDR_PIN);
  msg.pot_value = analogRead(POT_PIN);
  msg.bootcount = bootCount;

  if(SERIAL_PH) {
    msg.ph = 7.0f;
    Serial.println("Please Enter a Number Simulating PH value! You have 5 Seconds");
    while (Serial.available()) Serial.read();   // clear old input
    uint32_t current = millis();
    bool got = false;
    while (!got && (millis() - current < 5000)) {
      if (Serial.available()) {
        String line = Serial.readStringUntil('\n');
        line.trim();
        if (line.length() > 0) {
          float v = line.toFloat();
          if ((isdigit(line[0]) || line[0] == '.') && v >= 0.0f && v <= 14.0f) {
            msg.ph = v;
            got = true;
            Serial.printf("You sent : %.2f\n", msg.ph);
          } else {
            Serial.println("Enter a number from 0 to 14");
          }
        }
      }
      vTaskDelay(pdMS_TO_TICKS(20));
    }
  }
  else {
    msg.ph = (msg.pot_value / 4095.0f) * 14.0f;
  }

  msg.turb = (msg.ldr_value / 4095.0f) * 100.0f;
  float phPenalty = fabsf(msg.ph - 7.0f) / 7.0f * 100.0f;
  msg.WQI = 0.5f * msg.turb + 0.5f * phPenalty;
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
  vTaskDelete(NULL);
}

void alert_task(void* ar) {
  for (;;) {
    message r;
    if (xQueueReceive(readQueue, &r, portMAX_DELAY) == pdTRUE) {
      float wqi = r.WQI;

      const char* status = (wqi > WQI_ALERT) ? "CRITICAL" : (wqi >= WQI_WARNING ? "WARNING" : "GOOD");
      Serial.printf("WQI = %.1f (pH %.2f, turb %.1f%%) -> %s\n", r.WQI, r.ph, r.turb, status);

      if (wqi > WQI_ALERT) {
        AlertMsg a;
        snprintf(a.text, sizeof(a.text), "{\"alert\":\"CRITICAL\",\"wqi\":%.1f,\"ph\":%.2f,\"turbidity\":%.1f}", r.WQI, r.ph, r.turb);
        xQueueOverwrite(alertQueue, &a);

        for (int i = 0; i < 5; i++) {
          digitalWrite(RED_LED, HIGH);
          tone(BUZZER, 5000);
          vTaskDelay(pdMS_TO_TICKS(200));
          digitalWrite(RED_LED, LOW);
          noTone(BUZZER);
          vTaskDelay(pdMS_TO_TICKS(200));
        }
        digitalWrite(RED_LED, HIGH);
      } else {
        digitalWrite(RED_LED, LOW);
        noTone(BUZZER);
        for (int i = 0; i < 5; ++i) {
          digitalWrite(GREEN_LED, HIGH);
          vTaskDelay(pdMS_TO_TICKS(200));
          digitalWrite(GREEN_LED, LOW);
          vTaskDelay(pdMS_TO_TICKS(200));
        }
      }
      xEventGroupSetBits(events, ALERT_DONE_BIT);
    }
  }
}

void goToSleep() {
  sleeping = true;
  Serial.printf("Going to deep sleep for %d s...\n", SLEEP_SECONDS);
  Serial.flush();

  WiFi.disconnect(true);
  digitalWrite(RED_LED, LOW);
  noTone(BUZZER);
  display.clearDisplay();
  display.setTextSize(2);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println("Sleeping");
  display.printf("Boot Count : %d", bootCount);
  display.display();

  esp_sleep_enable_timer_wakeup((uint64_t)SLEEP_SECONDS * 1000000ULL);
  esp_deep_sleep_start();
}

void sleep_task(void* ar) {
  xEventGroupWaitBits(events, ALERT_DONE_BIT, pdFALSE, pdTRUE, pdMS_TO_TICKS(40000));
  vTaskDelay(pdMS_TO_TICKS(5000));


  xEventGroupSetBits(events, SLEEP_NOW_BIT);
  xEventGroupWaitBits(events, MQTT_CLOSED_BIT, pdFALSE, pdTRUE, pdMS_TO_TICKS(2000));

  goToSleep();
}

void WiFiManager_Callback(WiFiManager* wm) {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println(F("Wi-Fi setup"));
  display.println();
  display.println(F("Join Wi-Fi:"));
  display.println(F("WaterMonitor-Setup"));
  display.println();
  display.println(F("Open 192.168.4.1"));
  display.display();
}

#define WDT_TIMEOUT_S 120 

void startWatchdog() {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  esp_task_wdt_config_t cfg = {
    .timeout_ms     = WDT_TIMEOUT_S * 1000,
    .idle_core_mask = 0,      // don't watch idle tasks, so existing behavior is unchanged
    .trigger_panic  = true    // reset on hang
  };
  esp_task_wdt_reconfigure(&cfg);
#else
  esp_task_wdt_init(WDT_TIMEOUT_S, true);
#endif
  esp_task_wdt_add(NULL);     // watches loopTask, which never resets it
}

void setup() {
  Serial.begin(115200);
  Wire.begin(21, 22);
  bootCount++;

  Serial.printf("\nFirmware version %s | boot #%d\n", FW_VERSION, bootCount);

  pinMode(RED_LED, OUTPUT);
  pinMode(BUZZER, OUTPUT);
  pinMode(GREEN_LED, OUTPUT);
  pinMode(POT_PIN, INPUT);
  pinMode(LDR_PIN, INPUT);

  events = xEventGroupCreate();
  WiFi.mode(WIFI_STA);

  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println("[Setup] OLED init FAILED – check wiring!");
  } else {
    display.clearDisplay();
    display.display();
  }

  WiFiManager wm;
  //wm.resetSettings();
  wm.setAPCallback(WiFiManager_Callback);
  wm.setConnectTimeout(20);
  wm.setConfigPortalTimeout(180);

  if (bootCount > 1) {
    wm.setEnableConfigPortal(false);
  }

  if (!wm.autoConnect("WaterMonitor-Setup")) {
    Serial.println("No Wi-Fi, sleeping and trying again");
    esp_sleep_enable_timer_wakeup((uint64_t)SLEEP_SECONDS * 1000000ULL);
    esp_deep_sleep_start();
  }

  // FIX: handler registered AFTER WiFiManager, so its WiFi.reconnect() can't
  // interfere with WiFiManager's own connection attempts
  WiFi.onEvent(onWiFiEvent);

  xEventGroupSetBits(events, WIFI_UP_BIT);
  Serial.println("\nWiFi Connected");

  // ===== OTA: بعد الواي فاي وقبل MQTT والـ tasks =====
  // لو فيه تحديث الجهاز هيعمل restart لوحده ومش هيكمل
  checkForOTA();
  // ===================================================

  syncTime();   // FIX: needed for setCACert()

  esp.setCACert(CA_CERT);
  client.setServer(server, port);
  client.setCallback(callback);

  awakeStart = millis();
  startWatchdog();

  MQTTQueue = xQueueCreate(1, sizeof(message));
  displayQueue = xQueueCreate(1, sizeof(message));
  readQueue = xQueueCreate(1, sizeof(message));
  alertQueue = xQueueCreate(1, sizeof(AlertMsg));
  xTaskCreate(mqtt, "MQTT", 8192, NULL, 5, NULL);
  xTaskCreate(read_sensors, "read", 8192, NULL, 4, NULL);
  xTaskCreate(print, "write", 8192, NULL, 2, NULL);
  xTaskCreate(alert_task, "alert", 4096, NULL, 3, NULL);
  xTaskCreate(sleep_task, "sleep", 4096, NULL, 1, NULL);
}

void loop() {
  vTaskDelay(pdMS_TO_TICKS(1000));
  if (millis() - awakeStart > AWAKE_MAX_MS) {
    Serial.println("Failsafe: awake too long, forcing sleep");
    goToSleep();
  }
}
