#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define SCREEN_H 64
#define SCREEN_W 128

#define LDR_PIN 34
#define BLUE_LED 4
#define RED_LED 23
#define BUZZER 25
#define POT_PIN 39

#define WIFI_UP_BIT  BIT0
#define MQTT_UP_BIT  BIT1

Adafruit_SSD1306 display(SCREEN_W, SCREEN_H, &Wire, -1);

const char* ssid   = "Reman";
const char* pass   = "remansupergirl!!123";

const char* server = "y1257324.ala.eu-central-1.emqxsl.com";
const int   port   = 8883;
const char* user   = "lzka";
const char* pass_q = "87654321";

WiFiClientSecure esp;
PubSubClient client(esp);

// SemaphoreHandle_t SerialMutex;

QueueHandle_t displayQueue, readQueue, MQTTQueue;

EventGroupHandle_t events;

typedef struct {
  int bootcount = 0;
  int ldr_value = 0;
  int pot_value = 0;
  int last_read = 0;
} message;

message msg;

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
      WiFi.reconnect();            // quick call, OK here
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
      client.publish("testtopic/outTopic", "ESPConnected");
      client.subscribe("testtopic/inTopic");
    } else {
      xEventGroupClearBits(events, MQTT_UP_BIT);
      Serial.print("Failed rc = ");
      Serial.println(client.state());
      Serial.println("Retrying in 5 seconds");
      vTaskDelay(pdMS_TO_TICKS(5000));
    }
  }
}

void mqtt(void* ar) {
  for (;;) {

    xEventGroupWaitBits(events, WIFI_UP_BIT, pdFALSE, pdTRUE, portMAX_DELAY);
    if (!client.connected()) {
      xEventGroupClearBits(events, MQTT_UP_BIT);
      reconnect();
    } 
    client.loop();
    // if (Serial.available()) {
    //   String mes = Serial.readStringUntil('\n');
    //   mes.trim();
    //   if(mes.length() > 0) {
    //     client.publish("testtopic/outTopic", mes.c_str());
    //     Serial.print("Sent :" + mes);
    //   }
    // }
    message ms;
    if(xQueueReceive(MQTTQueue, &ms, 0) == pdTRUE) {
      char payload[96];
      snprintf(payload, sizeof(payload), "{\"ldr\":%d,\"pot\":%d,\"boot\":%d}", ms.ldr_value, ms.pot_value, ms.bootcount);
      client.publish("testtopic/readings", payload);
    }
    vTaskDelay(pdMS_TO_TICKS(2000));
  }
}

void print(void* ar) {
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println(F("Water Quality Monitoring System"));
  display.display();
  for(;;) {
    message reci;
    if(xQueueReceive(displayQueue, &reci , pdMS_TO_TICKS(50)) == pdTRUE) {
      display.printf("LDR : %d", reci.ldr_value);
      display.println();
      display.printf("POT : %d", reci.pot_value);
      display.println();
      display.printf("Boot counts : %d", reci.bootcount);
      display.display();
    }
  }
}

void read_sensors(void* ar) {
  while(1) {
    xEventGroupWaitBits(events, WIFI_UP_BIT | MQTT_UP_BIT, pdFALSE, pdTRUE, portMAX_DELAY);
    msg.ldr_value = analogRead(LDR_PIN);
    msg.pot_value = analogRead(POT_PIN);
    msg.bootcount++;
    if(xQueueOverwrite(readQueue, &msg) == pdTRUE && xQueueOverwrite(displayQueue, &msg) == pdTRUE && xQueueOverwrite(MQTTQueue, &msg) == pdTRUE) {
      Serial.println("Sent : ");
      Serial.printf("--> LDR  = %d", msg.ldr_value);
      Serial.println();
      Serial.printf("--> Potentiometer = %d", msg.pot_value);
      Serial.println();
      Serial.printf("--> Boot count = %d", msg.bootcount); 
      Serial.println();
    }
    else {
      Serial.println("Couldn't Send readings!");
    }
    vTaskDelay(pdMS_TO_TICKS(3000));
  }
}

void setup() {
  Serial.begin(115200);
  Wire.begin(21, 22);

  events = xEventGroupCreate();
  WiFi.onEvent(onWiFiEvent);
  WiFi.begin(ssid, pass);

  while (WiFi.status() != WL_CONNECTED) {
    vTaskDelay(pdMS_TO_TICKS(500));
    Serial.print(".");
  }
  
  xEventGroupSetBits(events, WIFI_UP_BIT);
  Serial.println("\nWiFi Connected");

  esp.setInsecure();
  client.setServer(server, port);
  client.setCallback(callback);

  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println("[Setup] OLED init FAILED – check wiring!");
  } else {
    display.clearDisplay();
    display.display();
  }

  // SerialMutex = xSemaphoreCreateMutex();

  // if(SerialMutex == NULL) {
  //   Serial.println("SerialMutex Creation Failed");
  //   while (true)
  //     delay(1000);
  // }

  MQTTQueue = xQueueCreate(1, sizeof(message));
  displayQueue = xQueueCreate(1, sizeof(message));
  readQueue = xQueueCreate(1, sizeof(message));
  xTaskCreate(mqtt, "MQTT", 8192, NULL, 5, NULL);
  xTaskCreate(read_sensors, "read", 8192, NULL, 4, NULL);
  xTaskCreate(print, "write", 8192, NULL, 2, NULL);
}

void loop() {
  vTaskDelay(portMAX_DELAY);
}