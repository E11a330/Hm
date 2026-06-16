#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <Ticker.h>
#include <TFT_eSPI.h>
#include <DHT.h>
#include <math.h>

/*************************************************
 *
 * Hardware Config
 *
 *************************************************/

// 板载 LED
#define STATUS_LED_PIN 1

// BOOT 按键，低电平有效
#define BOOT_KEY_PIN 0
#define KEY_ACTIVE_LEVEL LOW

// DHT11 传感器，OUT 接 IO15
#define DHT_PIN 15
#define DHT_TYPE DHT11

// XL9555 I2C 配置
#define XL9555_ADDR 0x20
#define XL9555_SDA_PIN 41
#define XL9555_SCL_PIN 42
#define XL9555_REG_OUTP0 0x02
#define XL9555_REG_OUTP1 0x03
#define XL9555_REG_CFG0 0x06
#define XL9555_REG_CFG1 0x07

// 蜂鸣器接在 XL9555 的 P0_3
#define BUZZER_BIT 3
#define BUZZER_ON_LEVEL 0
#define BUZZER_OFF_LEVEL 1
#define BUZZER_ALARM_BEEPS 5
#define BUZZER_BEEP_MS 300

/*************************************************
 *
 * TFT Layout
 *
 *************************************************/

#define TFT_SCREEN_WIDTH 320
#define TFT_SCREEN_HEIGHT 240

/*************************************************
 *
 * WiFi Config
 *
 *************************************************/

const char *WIFI_SSID = "arduino_108";
const char *WIFI_PASSWORD = "a1b2c3d4";

/*************************************************
 *
 * MQTT Config
 *
 *************************************************/

const char *MQTT_SERVER = "broker.emqx.io";
const int MQTT_PORT = 1883;
const char *MQTT_USER = "arduino";
const char *MQTT_PASSWORD = "123456";
const char *MQTT_TOPIC_PUB = "plant/status";
const char *MQTT_TOPIC_SUB = "plant/control";
const char *MQTT_CLIENT_ID = "Plant_Monitor_001";

/*************************************************
 *
 * Key Config
 *
 *************************************************/

#define KEY_DEBOUNCE_MS 30
#define KEY_DOUBLE_CLICK_MS 350
#define KEY_LONG_PRESS_MS 800

/*************************************************
 *
 * Demo Alarm Config
 *
 *************************************************/

// 演示模式：湿度 <= 65% 触发报警，>= 70% 恢复
#define HUMI_ALARM_PERCENT 65
#define HUMI_RECOVER_PERCENT 70

/*************************************************
 *
 * Global Objects
 *
 *************************************************/

WiFiClient espClient;
PubSubClient client(espClient);
Ticker sampleTicker;
TFT_eSPI tft = TFT_eSPI();
DHT dht(DHT_PIN, DHT_TYPE);

/*************************************************
 *
 * System State
 *
 *************************************************/

bool wifiOK = false;
bool mqttOK = false;

float temperature = NAN;
float humidity = NAN;
int humidityPercent = 0;

volatile bool sampleFlag = false;

unsigned long lastWiFiRetryMs = 0;
unsigned long lastMqttRetryMs = 0;

/*************************************************
 *
 * XL9555 / Buzzer State
 *
 *************************************************/

bool xl9555OK = false;
uint8_t xl9555_outp0 = 0xFF;

bool buzzerAlarmActive = false;
bool buzzerOnState = false;
int buzzerBeepDone = 0;
int buzzerBeepTotal = 0;
unsigned long buzzerLastMs = 0;

bool humidityAlarmed = false;

/*************************************************
 *
 * BOOT Key State Machine
 *
 * 单击：刷新传感器 / 停止报警
 * 双击：打印调试信息
 * 长按：立即上报 MQTT 状态
 *
 *************************************************/

bool keyStableState = HIGH;
bool keyLastReading = HIGH;
unsigned long keyLastDebounceTime = 0;

bool keyPressed = false;
bool longPressTriggered = false;
unsigned long keyPressStartTime = 0;

bool waitingSecondClick = false;
unsigned long firstClickReleaseTime = 0;

/*************************************************
 *
 * UI Cache
 *
 *************************************************/

String lastHumidityText = "";
String lastTemperatureText = "";
String lastAlarmText = "";
String lastWifiText = "";
String lastMqttText = "";
int lastBarValue = -1;

/*************************************************
 *
 * Function Declarations
 *
 *************************************************/

void wifi_init();
void wifi_check();

void mqtt_init();
void mqtt_reconnect();
void mqtt_callback(char *, byte *, unsigned int);

void led_init();
void led_update();

void xl9555_init();
bool xl9555_writeReg(uint8_t reg, uint8_t val);

void buzzer_set(bool on);
void buzzer_start_alarm(int beeps);
void buzzer_stop();
void buzzer_update();

void key_init();
void key_scan();
void onSingleClick();
void onDoubleClick();
void onLongPress();

void sensor_init();
void read_sensor();

void tft_init();
void tft_drawStaticUI();
void tft_updateDynamicUI();
void drawStatusBadge(const String &text, uint16_t color);
void drawHumidityBar(int value);

void publishStatus();
void sample_callback();

/*************************************************
 *
 * setup
 *
 *************************************************/

void setup()
{
  Serial.begin(115200);
  delay(1000);

  Serial.println("进入 setup");

  led_init();
  Serial.println("LED 初始化完成");

  key_init();
  Serial.println("按键初始化完成");

  xl9555_init();

  sensor_init();
  Serial.println("DHT11 初始化完成");

  tft_init();

  wifi_init();
  mqtt_init();

  sampleTicker.attach(3.0, sample_callback);

  Serial.println("系统启动完成");
}

/*************************************************
 *
 * loop
 *
 *************************************************/

void loop()
{
  key_scan();
  wifi_check();

  if (wifiOK)
  {
    if (!client.connected())
    {
      mqtt_reconnect();
    }

    if (client.connected())
    {
      mqttOK = true;
      client.loop();
    }
    else
    {
      mqttOK = false;
    }
  }
  else
  {
    mqttOK = false;
  }

  if (sampleFlag)
  {
    sampleFlag = false;
    read_sensor();
    publishStatus();
    tft_updateDynamicUI();
  }

  buzzer_update();
  led_update();
}

/*************************************************
 *
 * WiFi Module
 *
 *************************************************/

void wifi_init()
{
  Serial.println("======== 正在连接 WiFi ========");

  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  unsigned long startMs = millis();

  while (WiFi.status() != WL_CONNECTED && millis() - startMs < 15000)
  {
    delay(500);
    Serial.print("。");
  }

  Serial.println();

  if (WiFi.status() == WL_CONNECTED)
  {
    wifiOK = true;
    Serial.println("WiFi 连接成功");
    Serial.print("本机 IP：");
    Serial.println(WiFi.localIP());
  }
  else
  {
    wifiOK = false;
    Serial.println("WiFi 连接失败");
  }
}

void wifi_check()
{
  bool prevWifi = wifiOK;

  if (WiFi.status() == WL_CONNECTED)
  {
    wifiOK = true;
  }
  else
  {
    wifiOK = false;

    if (millis() - lastWiFiRetryMs >= 5000)
    {
      lastWiFiRetryMs = millis();

      Serial.println("正在重连 WiFi...");
      WiFi.disconnect();
      WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    }
  }

  if (prevWifi != wifiOK)
  {
    tft_updateDynamicUI();
  }
}

/*************************************************
 *
 * MQTT Module
 *
 *************************************************/

void mqtt_init()
{
  client.setServer(MQTT_SERVER, MQTT_PORT);
  client.setCallback(mqtt_callback);

  Serial.println("MQTT 模块初始化完成");
}

void mqtt_reconnect()
{
  if (!wifiOK)
    return;

  if (client.connected())
    return;

  if (millis() - lastMqttRetryMs < 3000)
    return;

  lastMqttRetryMs = millis();

  Serial.println("正在连接 MQTT 服务器...");

  if (client.connect(MQTT_CLIENT_ID, MQTT_USER, MQTT_PASSWORD))
  {
    Serial.println("MQTT 连接成功");

    client.subscribe(MQTT_TOPIC_SUB);

    Serial.print("已订阅主题：");
    Serial.println(MQTT_TOPIC_SUB);

    mqttOK = true;
    tft_updateDynamicUI();
  }
  else
  {
    mqttOK = false;

    Serial.print("MQTT 连接失败，错误码：");
    Serial.println(client.state());

    tft_updateDynamicUI();
  }
}

void mqtt_callback(char *topic, byte *payload, unsigned int length)
{
  char msg[256] = {0};

  if (length >= sizeof(msg))
  {
    length = sizeof(msg) - 1;
  }

  memcpy(msg, payload, length);
  msg[length] = '\0';

  Serial.println("======= 收到 MQTT 消息 =======");
  Serial.print("主题：");
  Serial.println(topic);
  Serial.print("内容：");
  Serial.println(msg);

  JsonDocument doc;
  DeserializationError error = deserializeJson(doc, msg);

  if (error)
  {
    Serial.print("JSON 解析失败：");
    Serial.println(error.c_str());
    return;
  }

  String cmd = doc["cmd"] | "";

  if (cmd == "refresh")
  {
    Serial.println("执行远程刷新");
    read_sensor();
  }
  else if (cmd == "ping")
  {
    Serial.println("收到远程 ping");
  }
  else if (cmd == "stop_alarm")
  {
    buzzer_stop();
    Serial.println("已通过 MQTT 关闭蜂鸣器");
  }
  else if (cmd == "beep")
  {
    Serial.println("收到远程蜂鸣器测试");
    buzzer_start_alarm(BUZZER_ALARM_BEEPS);
  }

  publishStatus();
  tft_updateDynamicUI();
}

/*************************************************
 *
 * LED Module
 *
 *************************************************/

void led_init()
{
  pinMode(STATUS_LED_PIN, OUTPUT);
  digitalWrite(STATUS_LED_PIN, LOW);
}

void led_update()
{
  static unsigned long lastBlink = 0;
  static bool ledState = false;

  if (wifiOK && mqttOK)
  {
    digitalWrite(STATUS_LED_PIN, HIGH);
  }
  else if (wifiOK)
  {
    if (millis() - lastBlink >= 800)
    {
      lastBlink = millis();
      ledState = !ledState;
      digitalWrite(STATUS_LED_PIN, ledState);
    }
  }
  else
  {
    if (millis() - lastBlink >= 250)
    {
      lastBlink = millis();
      ledState = !ledState;
      digitalWrite(STATUS_LED_PIN, ledState);
    }
  }
}

/*************************************************
 *
 * XL9555 + Buzzer Module
 *
 *************************************************/

bool xl9555_writeReg(uint8_t reg, uint8_t val)
{
  Wire.beginTransmission(XL9555_ADDR);
  Wire.write(reg);
  Wire.write(val);
  return (Wire.endTransmission() == 0);
}

void xl9555_init()
{
  Wire.begin(XL9555_SDA_PIN, XL9555_SCL_PIN);
  Wire.setClock(400000);

  Wire.beginTransmission(XL9555_ADDR);

  if (Wire.endTransmission() == 0)
  {
    xl9555OK = true;
  }
  else
  {
    xl9555OK = false;
    Serial.println("XL9555 未检测到，请检查 I2C 接线");
    return;
  }

  uint8_t cfg0 = 0xFF;
  cfg0 &= ~(1 << BUZZER_BIT);
  xl9555_writeReg(XL9555_REG_CFG0, cfg0);

  xl9555_outp0 = 0xFF;
  buzzer_set(false);

  uint8_t cfg1 = 0xFF;
  cfg1 &= ~(1 << 2);
  cfg1 &= ~(1 << 3);
  xl9555_writeReg(XL9555_REG_CFG1, cfg1);
  xl9555_writeReg(XL9555_REG_OUTP1, 0xFF);

  Serial.println("XL9555 初始化成功，蜂鸣器与背光已就绪");
}

void buzzer_set(bool on)
{
  if (!xl9555OK)
    return;

  bool level = on ? BUZZER_ON_LEVEL : BUZZER_OFF_LEVEL;

  if (level)
  {
    xl9555_outp0 |= (1 << BUZZER_BIT);
  }
  else
  {
    xl9555_outp0 &= ~(1 << BUZZER_BIT);
  }

  xl9555_writeReg(XL9555_REG_OUTP0, xl9555_outp0);
}

void buzzer_start_alarm(int beeps)
{
  if (!xl9555OK)
    return;

  buzzerAlarmActive = true;
  buzzerBeepTotal = beeps;
  buzzerBeepDone = 0;
  buzzerOnState = true;
  buzzerLastMs = millis();

  buzzer_set(true);
}

void buzzer_stop()
{
  buzzerAlarmActive = false;
  buzzerOnState = false;

  buzzer_set(false);
}

void buzzer_update()
{
  if (!buzzerAlarmActive)
    return;

  if (!xl9555OK)
    return;

  if (millis() - buzzerLastMs >= BUZZER_BEEP_MS)
  {
    buzzerLastMs = millis();
    buzzerOnState = !buzzerOnState;

    buzzer_set(buzzerOnState);

    if (!buzzerOnState)
    {
      buzzerBeepDone++;

      if (buzzerBeepDone >= buzzerBeepTotal)
      {
        buzzer_stop();
      }
    }
  }
}

/*************************************************
 *
 * BOOT Key Module
 *
 *************************************************/

void key_init()
{
  pinMode(BOOT_KEY_PIN, INPUT_PULLUP);

  keyStableState = digitalRead(BOOT_KEY_PIN);
  keyLastReading = keyStableState;
  keyLastDebounceTime = 0;

  keyPressed = false;
  longPressTriggered = false;
  waitingSecondClick = false;
}

void key_scan()
{
  bool reading = digitalRead(BOOT_KEY_PIN);

  if (reading != keyLastReading)
  {
    keyLastDebounceTime = millis();
    keyLastReading = reading;
  }

  if ((millis() - keyLastDebounceTime) > KEY_DEBOUNCE_MS)
  {
    if (reading != keyStableState)
    {
      keyStableState = reading;

      if (keyStableState == KEY_ACTIVE_LEVEL)
      {
        keyPressed = true;
        longPressTriggered = false;
        keyPressStartTime = millis();
      }
      else
      {
        if (keyPressed)
        {
          keyPressed = false;

          if (!longPressTriggered)
          {
            if (waitingSecondClick)
            {
              if (millis() - firstClickReleaseTime <= KEY_DOUBLE_CLICK_MS)
              {
                waitingSecondClick = false;
                onDoubleClick();
              }
            }
            else
            {
              waitingSecondClick = true;
              firstClickReleaseTime = millis();
            }
          }
        }
      }
    }
  }

  if (keyPressed && !longPressTriggered)
  {
    if (millis() - keyPressStartTime >= KEY_LONG_PRESS_MS)
    {
      longPressTriggered = true;
      waitingSecondClick = false;
      onLongPress();
    }
  }

  if (waitingSecondClick)
  {
    if (millis() - firstClickReleaseTime > KEY_DOUBLE_CLICK_MS)
    {
      waitingSecondClick = false;
      onSingleClick();
    }
  }
}

void onSingleClick()
{
  Serial.println("检测到单击");

  if (buzzerAlarmActive)
  {
    buzzer_stop();
    Serial.println("已手动关闭报警");
    return;
  }

  Serial.println("刷新温湿度数据");
  read_sensor();
  tft_updateDynamicUI();
}

void onDoubleClick()
{
  Serial.println("检测到双击，输出调试信息");

  Serial.print("温度：");
  if (isnan(temperature))
  {
    Serial.println("无效");
  }
  else
  {
    Serial.print(temperature, 1);
    Serial.println(" C");
  }

  Serial.print("湿度：");
  if (isnan(humidity))
  {
    Serial.println("无效");
  }
  else
  {
    Serial.print(humidity, 1);
    Serial.println(" %");
  }

  Serial.print("WiFi 状态：");
  Serial.println(wifiOK ? "已连接" : "未连接");

  Serial.print("MQTT 状态：");
  Serial.println(mqttOK ? "已连接" : "未连接");

  Serial.print("报警状态：");
  Serial.println(buzzerAlarmActive ? "报警中" : "正常");
}

void onLongPress()
{
  Serial.println("检测到长按，立即上报一次状态");
  publishStatus();
  tft_updateDynamicUI();
}

/*************************************************
 *
 * DHT11 Sensor Module
 *
 *************************************************/

void sensor_init()
{
  dht.begin();
  delay(2000);
  read_sensor();
}

void read_sensor()
{
  float h = dht.readHumidity();
  float t = dht.readTemperature();

  if (isnan(h) || isnan(t))
  {
    delay(100);
    h = dht.readHumidity();
    t = dht.readTemperature();
  }

  if (!isnan(h))
  {
    humidity = h;
    humidityPercent = constrain((int)round(humidity), 0, 100);
  }

  if (!isnan(t))
  {
    temperature = t;
  }

  Serial.println("======= 温湿度读取 =======");

  Serial.print("温度：");
  if (isnan(temperature))
  {
    Serial.println("无效");
  }
  else
  {
    Serial.print(temperature, 1);
    Serial.println(" C");
  }

  Serial.print("湿度：");
  if (isnan(humidity))
  {
    Serial.println("无效");
  }
  else
  {
    Serial.print(humidity, 1);
    Serial.println(" %");
  }

  if (humidityPercent <= HUMI_ALARM_PERCENT)
  {
    if (!humidityAlarmed)
    {
      humidityAlarmed = true;
      Serial.println("湿度低于演示阈值，触发报警");
      buzzer_start_alarm(BUZZER_ALARM_BEEPS);
    }
  }
  else if (humidityPercent >= HUMI_RECOVER_PERCENT)
  {
    humidityAlarmed = false;
  }
}

/*************************************************
 *
 * TFT Display Module
 *
 *************************************************/

void tft_init()
{
  Serial.println("开始初始化屏幕");

  tft.init();
  tft.setRotation(1);
  tft.fillScreen(TFT_BLACK);

  tft_drawStaticUI();
  tft_updateDynamicUI();

  Serial.println("屏幕初始化完成");
}

void tft_drawStaticUI()
{
  tft.fillScreen(TFT_BLACK);

  tft.fillRect(0, 0, TFT_SCREEN_WIDTH, 34, TFT_NAVY);
  tft.setTextColor(TFT_WHITE, TFT_NAVY);
  tft.drawString("PLANT MONITOR", 10, 9, 2);

  tft.drawRoundRect(10, 44, 300, 110, 10, TFT_DARKGREY);

  tft.setTextColor(TFT_DARKGREY, TFT_BLACK);
  tft.drawString("HUMIDITY", 24, 56, 2);
  tft.drawString("TEMP", 218, 56, 2);

  tft.drawRoundRect(24, 166, 272, 18, 8, TFT_DARKGREY);

  tft.setTextColor(TFT_DARKGREY, TFT_BLACK);
  tft.drawString("DRY", 24, 190, 2);
  tft.drawString("WET", 266, 190, 2);

  tft.fillRect(0, 216, TFT_SCREEN_WIDTH, 24, TFT_DARKGREY);
  tft.setTextColor(TFT_BLACK, TFT_DARKGREY);
  tft.drawString("WiFi:", 8, 221, 2);
  tft.drawString("MQTT:", 92, 221, 2);
  tft.drawString("plant/status", 190, 221, 2);
}

void drawStatusBadge(const String &text, uint16_t color)
{
  tft.fillRoundRect(210, 7, 96, 20, 10, color);
  tft.setTextColor(TFT_BLACK, color);

  if (text == "ALARM")
  {
    tft.drawString(text, 228, 10, 2);
  }
  else
  {
    tft.drawString(text, 224, 10, 2);
  }
}

void drawHumidityBar(int value)
{
  value = constrain(value, 0, 100);

  int barX = 26;
  int barY = 168;
  int barW = 268;
  int barH = 14;

  tft.fillRoundRect(barX, barY, barW, barH, 7, TFT_BLACK);

  int fillW = map(value, 0, 100, 0, barW);

  uint16_t barColor;

  if (value <= HUMI_ALARM_PERCENT)
  {
    barColor = TFT_RED;
  }
  else if (value < HUMI_RECOVER_PERCENT)
  {
    barColor = TFT_ORANGE;
  }
  else
  {
    barColor = TFT_GREEN;
  }

  if (fillW > 0)
  {
    tft.fillRoundRect(barX, barY, fillW, barH, 7, barColor);
  }

  tft.drawRoundRect(24, 166, 272, 18, 8, TFT_DARKGREY);
}

void tft_updateDynamicUI()
{
  String humidityText;

  if (isnan(humidity))
  {
    humidityText = "--%";
  }
  else
  {
    humidityText = String(humidityPercent) + "%";
  }

  String temperatureText;

  if (isnan(temperature))
  {
    temperatureText = "--.-C";
  }
  else
  {
    temperatureText = String(temperature, 1) + "C";
  }

  String alarmText = (humidityPercent <= HUMI_ALARM_PERCENT) ? "ALARM" : "NORMAL";
  String wifiText = wifiOK ? "OK" : "OFF";
  String mqttText = mqttOK ? "OK" : "OFF";

  if (alarmText != lastAlarmText)
  {
    drawStatusBadge(alarmText, alarmText == "ALARM" ? TFT_RED : TFT_GREEN);
    lastAlarmText = alarmText;
  }

  if (humidityText != lastHumidityText)
  {
    tft.fillRect(24, 78, 160, 58, TFT_BLACK);

    uint16_t humiColor = (humidityPercent <= HUMI_ALARM_PERCENT) ? TFT_RED : TFT_CYAN;

    tft.setTextColor(humiColor, TFT_BLACK);
    tft.drawString(humidityText, 28, 82, 7);

    lastHumidityText = humidityText;
  }

  if (temperatureText != lastTemperatureText)
  {
    tft.fillRect(208, 82, 92, 32, TFT_BLACK);

    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.drawString(temperatureText, 210, 88, 4);

    lastTemperatureText = temperatureText;
  }

  if (humidityPercent != lastBarValue)
  {
    drawHumidityBar(humidityPercent);
    lastBarValue = humidityPercent;
  }

  if (wifiText != lastWifiText)
  {
    tft.fillRect(48, 221, 40, 16, TFT_DARKGREY);

    tft.setTextColor(wifiOK ? TFT_GREEN : TFT_RED, TFT_DARKGREY);
    tft.drawString(wifiText, 48, 221, 2);

    lastWifiText = wifiText;
  }

  if (mqttText != lastMqttText)
  {
    tft.fillRect(140, 221, 42, 16, TFT_DARKGREY);

    tft.setTextColor(mqttOK ? TFT_GREEN : TFT_RED, TFT_DARKGREY);
    tft.drawString(mqttText, 140, 221, 2);

    lastMqttText = mqttText;
  }
}

/*************************************************
 *
 * MQTT Status Publish
 *
 *************************************************/

void publishStatus()
{
  if (!client.connected())
  {
    mqttOK = false;
    return;
  }

  JsonDocument doc;

  doc["device_id"] = MQTT_CLIENT_ID;
  doc["temperature"] = isnan(temperature) ? 0 : temperature;
  doc["humidity"] = isnan(humidity) ? 0 : humidity;
  doc["humidity_alarm"] = (humidityPercent <= HUMI_ALARM_PERCENT);
  doc["wifi"] = wifiOK;
  doc["mqtt"] = mqttOK;

  char buf[256];
  size_t len = serializeJson(doc, buf);

  bool ok = client.publish(MQTT_TOPIC_PUB, (uint8_t *)buf, len);

  Serial.println("======= 状态上报 =======");
  Serial.println(buf);
  Serial.println(ok ? "状态上报成功" : "状态上报失败");
}

/*************************************************
 *
 * Timer Callback
 *
 *************************************************/

void sample_callback()
{
  sampleFlag = true;
}
