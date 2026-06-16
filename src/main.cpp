#include <Arduino.h>
#include <Wire.h> // ★ 新增：XL9555 走 I2C
#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <Ticker.h>
#include <TFT_eSPI.h>

/*************************************************
 * ① 硬件配置
 *************************************************/
// 板载LED
#define STATUS_LED_PIN 1
// BOOT按键，低电平有效
#define BOOT_KEY_PIN 0
#define KEY_ACTIVE_LEVEL LOW
// 土壤湿度传感器（模拟输入）
#define SOIL_PIN 15

// ★ 新增：XL9555 I2C 扩展芯片（正点原子 DNESP32S3：IIC_SDA=IO41, IIC_SCL=IO42）
#define XL9555_ADDR 0x20
#define XL9555_SDA_PIN 41
#define XL9555_SCL_PIN 42
#define XL9555_REG_OUTP0 0x02 // 输出端口0寄存器
#define XL9555_REG_OUTP1 0x03 // 输出端口1寄存器
#define XL9555_REG_CFG0 0x06  // 配置端口0寄存器(0=输出,1=输入)
#define XL9555_REG_CFG1 0x07  // 配置端口1寄存器

// ★ 新增：蜂鸣器接在 XL9555 的 P0_3（Port0 第3位）
#define BUZZER_BIT 3
#define BUZZER_ON_LEVEL 0 // 实测：P0_3=0 响，=1 停（若相反就把这两个值对调）
#define BUZZER_OFF_LEVEL 1
#define BUZZER_ALARM_BEEPS 5 // 缺水报警响几声
#define BUZZER_BEEP_MS 300   // 每段响/停时长(ms)

/*************************************************
 * ② TFT布局参数
 *************************************************/
#define TFT_SCREEN_WIDTH 320
#define TFT_SCREEN_HEIGHT 240

/*************************************************
 * ③ WiFi配置
 *************************************************/
const char *WIFI_SSID = "Rei";
const char *WIFI_PASSWORD = "200562hm";

/*************************************************
 * ④ MQTT配置
 *************************************************/
const char *MQTT_SERVER = "broker.emqx.io";
const int MQTT_PORT = 1883;
const char *MQTT_USER = "arduino";
const char *MQTT_PASSWORD = "123456";
const char *MQTT_TOPIC_PUB = "plant/status";
const char *MQTT_TOPIC_SUB = "plant/control";
const char *MQTT_CLIENT_ID = "Plant_Monitor_001";

/*************************************************
 * ⑤ 按键参数
 *************************************************/
#define KEY_DEBOUNCE_MS 30
#define KEY_DOUBLE_CLICK_MS 350
#define KEY_LONG_PRESS_MS 800

/*************************************************
 * ⑥ 土壤湿度标定参数
 *************************************************/
#define SOIL_AIR_VALUE 3200
#define SOIL_WATER_VALUE 1400
// ★ 新增：缺水报警阈值（带回滞，避免在阈值附近反复报警）
#define SOIL_DRY_PERCENT 20 // 低于此湿度触发报警
#define SOIL_DRY_RECOVER 25 // 回升到此湿度才解除报警标志

/*************************************************
 * ⑦ 全局对象
 *************************************************/
WiFiClient espClient;
PubSubClient client(espClient);
Ticker sampleTicker;
TFT_eSPI tft = TFT_eSPI();

/*************************************************
 * ⑧ 系统状态变量
 *************************************************/
bool wifiOK = false;
bool mqttOK = false;
int soilRaw = 0;
int soilPercent = 0;
volatile bool sampleFlag = false;
unsigned long lastWiFiRetryMs = 0;
unsigned long lastMqttRetryMs = 0;

// ★ 新增：XL9555 / 蜂鸣器状态变量
bool xl9555OK = false;
uint8_t xl9555_outp0 = 0xFF; // 输出端口0缓存
bool buzzerAlarmActive = false;
bool buzzerOnState = false;
int buzzerBeepDone = 0;
int buzzerBeepTotal = 0;
unsigned long buzzerLastMs = 0;
bool soilDryAlarmed = false; // 缺水报警边沿标志

/*************************************************
 * ⑨ BOOT按键状态机
 *
 * 单击：刷新土壤湿度（报警时单击先停报警）
 * 双击：打印调试信息
 * 长按：立即上报一次MQTT状态
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
 * ⑩ UI缓存变量
 *************************************************/
String lastSoilText = "";
String lastWifiText = "";
String lastMqttText = "";
String lastRawText = "";

/*************************************************
 * ⑪ 函数声明
 *************************************************/
void wifi_init();
void wifi_check();
void mqtt_init();
void mqtt_reconnect();
void mqtt_callback(char *, byte *, unsigned int);
void led_init();
void led_update();
// ★ 新增：XL9555 + 蜂鸣器
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
void read_soil();
int convert_soil_to_percent(int raw);
void tft_init();
void tft_drawStaticUI();
void tft_updateDynamicUI();
void drawValueField(int x, int y, int w, int h, const String &value, uint16_t color, int font = 2);
void publishStatus();
void sample_callback();

/*************************************************
 * setup
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

  sensor_init();
  Serial.println("土壤湿度传感器初始化完成");

  xl9555_init(); // ★ 新增：点亮背光 + 初始化蜂鸣器（必须在 tft_init 之前）
  tft_init();
  wifi_init();
  mqtt_init();

  sampleTicker.attach(3.0, sample_callback);
  Serial.println("系统启动完成");
}

/*************************************************
 * loop
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
    read_soil();
    publishStatus();
    tft_updateDynamicUI();
  }

  buzzer_update(); // ★ 新增：非阻塞驱动蜂鸣器报警
  led_update();
}

/*************************************************
 * WiFi模块
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
 * MQTT模块
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
  {
    return;
  }
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
    read_soil();
  }
  else if (cmd == "ping")
  {
    Serial.println("收到远程 ping");
  }
  else if (cmd == "stop_alarm") // ★ 新增：远程关闭蜂鸣器
  {
    buzzer_stop();
    Serial.println("已通过 MQTT 关闭蜂鸣器");
  }
  else if (cmd == "beep") // ★ 新增：远程测试蜂鸣器
  {
    Serial.println("收到远程蜂鸣器测试");
    buzzer_start_alarm(BUZZER_ALARM_BEEPS);
  }

  publishStatus();
  tft_updateDynamicUI();
}

/*************************************************
 * LED状态模块
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
    // 网络正常：常亮
    digitalWrite(STATUS_LED_PIN, HIGH);
  }
  else if (wifiOK)
  {
    // 仅WiFi正常：慢闪
    if (millis() - lastBlink >= 800)
    {
      lastBlink = millis();
      ledState = !ledState;
      digitalWrite(STATUS_LED_PIN, ledState);
    }
  }
  else
  {
    // 网络未连接：快闪
    if (millis() - lastBlink >= 250)
    {
      lastBlink = millis();
      ledState = !ledState;
      digitalWrite(STATUS_LED_PIN, ledState);
    }
  }
}

/*************************************************
 * ★ XL9555 + 蜂鸣器模块（移植自参考仓库）
 *
 * 作用1：点亮 TFT 背光（背光挂在 XL9555 的 P1 口）
 * 作用2：通过 P0_3 控制蜂鸣器，用于缺水报警
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
    Serial.println("XL9555 未检测到，请检查 I2C 接线(SDA=41/SCL=42)");
    return;
  }

  // ---- Port0：蜂鸣器 P0_3 配成输出 ----
  uint8_t cfg0 = 0xFF;
  cfg0 &= ~(1 << BUZZER_BIT); // P0_3 = 输出
  xl9555_writeReg(XL9555_REG_CFG0, cfg0);
  xl9555_outp0 = 0xFF;
  buzzer_set(false); // 蜂鸣器先关

  // ---- Port1：LCD背光 P1_2 / P1_3 配成输出并拉高点亮 ----
  uint8_t cfg1 = 0xFF;
  cfg1 &= ~(1 << 2); // P1_2 = 输出
  cfg1 &= ~(1 << 3); // P1_3 = 输出
  xl9555_writeReg(XL9555_REG_CFG1, cfg1);
  xl9555_writeReg(XL9555_REG_OUTP1, 0xFF); // Port1 全部输出高 -> 背光点亮

  Serial.println("XL9555 初始化成功，蜂鸣器与背光已就绪");
}

void buzzer_set(bool on)
{
  if (!xl9555OK)
    return;
  bool level = on ? BUZZER_ON_LEVEL : BUZZER_OFF_LEVEL;
  if (level)
    xl9555_outp0 |= (1 << BUZZER_BIT);
  else
    xl9555_outp0 &= ~(1 << BUZZER_BIT);
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

    // 每完成一次“响->停”算一声
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
 * BOOT按键模块
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
  // ★ 新增：若正在报警，单击先停报警
  if (buzzerAlarmActive)
  {
    buzzer_stop();
    Serial.println("已手动关闭缺水报警");
    return;
  }

  Serial.println("刷新土壤湿度");
  read_soil();
  tft_updateDynamicUI();
}

void onDoubleClick()
{
  Serial.println("检测到双击，输出调试信息");
  Serial.print("当前土壤原始值：");
  Serial.println(soilRaw);
  Serial.print("当前土壤湿度：");
  Serial.print(soilPercent);
  Serial.println("%");
  Serial.print("WiFi 状态：");
  Serial.println(wifiOK ? "已连接" : "未连接");
  Serial.print("MQTT 状态：");
  Serial.println(mqttOK ? "已连接" : "未连接");
  Serial.print("缺水报警：");
  Serial.println(buzzerAlarmActive ? "报警中" : "正常");
}

void onLongPress()
{
  Serial.println("检测到长按，立即上报一次状态");
  publishStatus();
  tft_updateDynamicUI();
}

/*************************************************
 * 传感器模块
 *************************************************/
void sensor_init()
{
  pinMode(SOIL_PIN, INPUT);
  read_soil();
}

void read_soil()
{
  soilRaw = analogRead(SOIL_PIN);
  soilPercent = convert_soil_to_percent(soilRaw);

  Serial.println("======= 土壤湿度读取 =======");
  Serial.print("原始值：");
  Serial.println(soilRaw);
  Serial.print("湿度：");
  Serial.print(soilPercent);
  Serial.println("%");

  // ★ 新增：缺水报警逻辑（带回滞，避免反复触发）
  if (soilPercent <= SOIL_DRY_PERCENT)
  {
    if (!soilDryAlarmed)
    {
      soilDryAlarmed = true;
      Serial.println("土壤过干，触发缺水报警，请浇水");
      buzzer_start_alarm(BUZZER_ALARM_BEEPS);
    }
  }
  else if (soilPercent >= SOIL_DRY_RECOVER)
  {
    // 湿度恢复，解除报警标志
    soilDryAlarmed = false;
  }
}

int convert_soil_to_percent(int raw)
{
  int value = map(raw, SOIL_AIR_VALUE, SOIL_WATER_VALUE, 0, 100);
  value = constrain(value, 0, 100);
  return value;
}

/*************************************************
 * TFT显示模块
 *************************************************/
void tft_init()
{
  Serial.println("开始初始化屏幕");
  tft.init();
  Serial.println("tft.init 完成");
  tft.setRotation(1);
  Serial.println("tft.setRotation 完成");
  tft.fillScreen(TFT_BLACK);
  tft_drawStaticUI();
  tft_updateDynamicUI();
  Serial.println("屏幕初始化完成");
}

void tft_drawStaticUI()
{
  tft.fillScreen(TFT_BLACK);

  tft.setTextColor(TFT_YELLOW, TFT_BLACK);
  tft.drawString("Smart Plant Monitor", 8, 8, 2);
  tft.drawFastHLine(0, 28, TFT_SCREEN_WIDTH, TFT_DARKGREY);

  tft.setTextColor(TFT_CYAN, TFT_BLACK);
  tft.drawString("湿度:", 10, 42, 2);
  tft.drawString("原始值:", 10, 72, 2);
  tft.drawString("WiFi:", 10, 102, 2);
  tft.drawString("MQTT:", 10, 132, 2);

  tft.drawFastHLine(180, 100, 130, TFT_DARKGREY);
  tft.setTextColor(TFT_LIGHTGREY, TFT_BLACK);
  tft.drawString("单击: 刷新湿度", 190, 115, 2);
  tft.drawString("双击: 串口调试", 190, 140, 2);
  tft.drawString("长按: 立即上报", 190, 165, 2);

  tft.drawFastHLine(0, 228, TFT_SCREEN_WIDTH, TFT_DARKGREY);
  tft.setTextColor(TFT_DARKGREY, TFT_BLACK);
  tft.drawString("上报: plant/status", 10, 232, 2);
  tft.drawString("订阅: plant/control", 170, 232, 2);
}

void drawValueField(int x, int y, int w, int h, const String &value, uint16_t color, int font)
{
  tft.fillRect(x, y, w, h, TFT_BLACK);
  tft.setTextColor(color, TFT_BLACK);
  tft.drawString(value, x, y, font);
}

void tft_updateDynamicUI()
{
  String soilText = String(soilPercent) + "%";
  String rawText = String(soilRaw);
  String wifiText = wifiOK ? "已连接" : "未连接";
  String mqttText = mqttOK ? "已连接" : "未连接";

  if (soilText != lastSoilText)
  {
    // ★ 新增：湿度过低显示红色提示缺水
    uint16_t soilColor = (soilPercent <= SOIL_DRY_PERCENT) ? TFT_RED : TFT_WHITE;
    drawValueField(80, 42, 80, 20, soilText, soilColor, 2);
    lastSoilText = soilText;
  }

  if (rawText != lastRawText)
  {
    drawValueField(80, 72, 80, 20, rawText, TFT_WHITE, 2);
    lastRawText = rawText;
  }

  if (wifiText != lastWifiText)
  {
    drawValueField(80, 102, 80, 20, wifiText, wifiOK ? TFT_GREEN : TFT_RED, 2);
    lastWifiText = wifiText;
  }

  if (mqttText != lastMqttText)
  {
    drawValueField(80, 132, 80, 20, mqttText, mqttOK ? TFT_GREEN : TFT_RED, 2);
    lastMqttText = mqttText;
  }
}

/*************************************************
 * MQTT状态上传
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
  doc["soil_raw"] = soilRaw;
  doc["soil"] = soilPercent;
  doc["dry_alarm"] = buzzerAlarmActive; // ★ 新增：上报缺水报警状态
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
 * 定时回调
 *************************************************/
void sample_callback()
{
  sampleFlag = true;
}
