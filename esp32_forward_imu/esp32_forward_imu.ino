/*
 * ESP32 TCP IMU + ADC 数据上传示例
 * — 对接 UART IMU （协议 0x49 … 0x4D），
 * — 在每帧 IMU payload 末尾追加 2 字节 ADC 原始读数，
 * — 通过 TCP 发给 PC （保持起始/结束标志、更新长度和校验）。
 */

#include <Arduino.h>
#include <WiFi.h>
#include "driver/uart.h"
#include <lwip/sockets.h>

//================ WiFi 与服务器配置 =================
#define WIFI_SSID       "iotwifi01"
#define WIFI_PASS       "ece@3040"
#define SERVER_IP       "192.168.1.63"
#define SERVER_PORT     9999

//================ UART 配置 =================
#define UART_PORT_NUM      UART_NUM_1
#define UART_BAUD_RATE     115200
#define UART_TX_PIN        GPIO_NUM_21
#define UART_RX_PIN        GPIO_NUM_20
#define UART_BUF_SIZE      1024

//================ IMU 数据包协议 =================
#define CMD_PACKET_BEGIN        0x49
#define CMD_PACKET_END          0x4D
#define CMD_PACKET_MAX_DATALEN  75

//================ ADC 配置 =================
const int ANALOG_PIN = 0;  // ADC1_CH6 (GPIO34)

//---------------- 全局变量 ----------------
static uint8_t  g_RxState      = 0;
static uint8_t  g_packetIndex  = 0;
static uint16_t g_CS           = 0;
static uint8_t  g_packetBuffer[5 + CMD_PACKET_MAX_DATALEN + 2]; // 多 +2 空间给 ADC
volatile bool   packet_ready   = false;
QueueHandle_t   uart_queue     = NULL;

//---------------- 函数原型 ----------------
void wifi_init_sta();
void uart_init_custom();
bool process_UART_byte(uint8_t byte);
void send_command_packet(const uint8_t *payload, uint8_t payload_len);
void configure_imu();
void print_acceleration();
static void uart_event_task(void *pvParameters);
static void tcp_client_task(void *pvParameters);

//==================================================================
// WiFi 初始化
//==================================================================
void wifi_init_sta() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println();
  Serial.print("Connected. IP: ");
  Serial.println(WiFi.localIP());
}

//==================================================================
// UART 初始化
//==================================================================
void uart_init_custom() {
  uart_config_t cfg = {
    .baud_rate    = UART_BAUD_RATE,
    .data_bits    = UART_DATA_8_BITS,
    .parity       = UART_PARITY_DISABLE,
    .stop_bits    = UART_STOP_BITS_1,
    .flow_ctrl    = UART_HW_FLOWCTRL_DISABLE,
    .source_clk   = UART_SCLK_DEFAULT,
  };
  uart_driver_install(UART_PORT_NUM, UART_BUF_SIZE*2, UART_BUF_SIZE*2, 20, &uart_queue, 0);
  uart_param_config(UART_PORT_NUM, &cfg);
  uart_set_pin(UART_PORT_NUM, UART_TX_PIN, UART_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
  Serial.println("UART initialized.");
}

//==================================================================
// 逐字节解析 IMU 数据包
//==================================================================
bool process_UART_byte(uint8_t byte) {
  switch (g_RxState) {
    case 0: // 等待起始码
      if (byte == CMD_PACKET_BEGIN) {
        g_packetIndex = 0;
        g_packetBuffer[g_packetIndex++] = byte;
        g_CS = 0;
        g_RxState = 1;
      }
      break;
    case 1: // 地址
      g_packetBuffer[g_packetIndex++] = byte;
      if (byte == 255) { g_RxState = 0; }
      else            { g_CS += byte; g_RxState = 2; }
      break;
    case 2: // 数据长度
      g_packetBuffer[g_packetIndex++] = byte;
      if (byte == 0 || byte > CMD_PACKET_MAX_DATALEN) { g_RxState = 0; }
      else                                           { g_CS += byte; g_RxState = 3; }
      break;
    case 3: { // 数据体
      g_packetBuffer[g_packetIndex++] = byte;
      g_CS += byte;
      uint8_t len = g_packetBuffer[2];
      if (g_packetIndex == 3 + len) g_RxState = 4;
      break;
    }
    case 4: // 校验
      g_packetBuffer[g_packetIndex++] = byte;
      if ((g_CS & 0xFF) == byte) g_RxState = 5;
      else                       g_RxState = 0;
      break;
    case 5: // 结束码
      g_packetBuffer[g_packetIndex++] = byte;
      if (byte == CMD_PACKET_END) {
        g_RxState = 0;
        packet_ready = true;
        return true;
      }
      else {
        g_RxState = 0;
      }
      break;
  }
  return false;
}

//==================================================================
// 发送 UART 命令包（配置 IMU）
//==================================================================
void send_command_packet(const uint8_t *payload, uint8_t payload_len) {
  uint8_t address = 0x01;
  uint8_t pkt[20];
  pkt[0] = CMD_PACKET_BEGIN;
  pkt[1] = address;
  pkt[2] = payload_len;
  uint16_t cs = address + payload_len;
  for (uint8_t i = 0; i < payload_len; i++) {
    pkt[3 + i] = payload[i];
    cs += payload[i];
  }
  pkt[3 + payload_len]     = cs & 0xFF;
  pkt[3 + payload_len + 1] = CMD_PACKET_END;
  uart_write_bytes(UART_PORT_NUM, (const char*)pkt, payload_len + 5);
}

//==================================================================
// 配置 IMU 示例：帧率、滤波等
//==================================================================
void configure_imu() {
  uint8_t params[11] = {0x12, 5,255,0, (0<<1)|0,100,1,3,5,0xFF,0x0F};
  send_command_packet(params, 11);
  vTaskDelay(200/portTICK_PERIOD_MS);
  uint8_t note = 0x19;
  send_command_packet(&note, 1);
  vTaskDelay(200/portTICK_PERIOD_MS);
}

//==================================================================
// 打印加速度数据（可选）
//==================================================================
void print_acceleration() {
  uint8_t len = g_packetBuffer[2];
  if (len < 13) return;
  uint8_t *p = &g_packetBuffer[3];
  if (p[0] != 0x11) return;
  uint16_t ctl = (p[2]<<8)|p[1];
  if (ctl & 0x0001) {
    int16_t rawAx = (p[8]<<8)|p[7];
    int16_t rawAy = (p[10]<<8)|p[9];
    int16_t rawAz = (p[12]<<8)|p[11];
    float sf = 0.00478515625;
    Serial.printf("aX=%.3f aY=%.3f aZ=%.3f\n",
                  rawAx*sf, rawAy*sf, rawAz*sf);
  }
}

//==================================================================
// UART 事件任务：读取并解析，每帧完成设置 packet_ready
//==================================================================
static void uart_event_task(void *pvParameters) {
  uart_event_t event;
  uint8_t buf[128];
  while (true) {
    if (xQueueReceive(uart_queue, &event, portMAX_DELAY)) {
      if (event.type == UART_DATA) {
        int len = uart_read_bytes(UART_PORT_NUM, buf, sizeof(buf), 10/portTICK_PERIOD_MS);
        for (int i = 0; i < len; i++) {
          process_UART_byte(buf[i]);
        }
      }
    }
  }
}

//==================================================================
// TCP 客户端任务：检测 packet_ready，插入 ADC，重算长度/校验，发送
//==================================================================
static void tcp_client_task(void *pvParameters) {
  struct sockaddr_in destAddr;
  destAddr.sin_family = AF_INET;
  destAddr.sin_addr.s_addr = inet_addr(SERVER_IP);
  destAddr.sin_port = htons(SERVER_PORT);

  while (true) {
    int sock = socket(AF_INET, SOCK_STREAM, IPPROTO_IP);
    if (sock < 0) { vTaskDelay(2000/portTICK_PERIOD_MS); continue; }
    if (connect(sock, (struct sockaddr*)&destAddr, sizeof(destAddr)) != 0) {
      close(sock); vTaskDelay(2000/portTICK_PERIOD_MS); continue;
    }
    Serial.println("TCP connected.");

    unsigned long lastFrame = millis();
    while (true) {
      if (packet_ready) {
        int origLen     = g_packetIndex;      // 含头尾总长
        uint8_t origPL  = g_packetBuffer[2];  // 原 payload 长度

        // 读取 ADC
        uint16_t adcRaw = analogRead(ANALOG_PIN);
        uint8_t hi = adcRaw >> 8, lo = adcRaw & 0xFF;

        // 在校验前插入
        int csPos  = origLen - 2;
        int endPos = origLen - 1;
        // 后移原校验、结束码
        g_packetBuffer[csPos + 2] = g_packetBuffer[csPos];
        g_packetBuffer[csPos + 3] = g_packetBuffer[endPos];
        // 插入 ADC bytes
        g_packetBuffer[csPos]     = hi;
        g_packetBuffer[csPos + 1] = lo;

        // 更新长度 & 重算校验
        uint8_t newPL = origPL + 2;
        g_packetBuffer[2] = newPL;
        uint16_t cs = 0;
        for (int i = 1; i < 3 + newPL; i++) cs += g_packetBuffer[i];
        g_packetBuffer[3 + newPL] = cs & 0xFF;

        // 更新总长度
        g_packetIndex = origLen + 2;
        packet_ready = false;

        // 发送
        int sent = send(sock, g_packetBuffer, g_packetIndex, 0);
        if (sent < 0) break;
        Serial.printf("Sent %d bytes (ADC=0x%04X)\n", g_packetIndex, adcRaw);

        // 限制帧率 ≥10ms
        unsigned long now = millis();
        if (now - lastFrame < 10) {
          vTaskDelay((10 - (now - lastFrame))/portTICK_PERIOD_MS);
        }
        lastFrame = millis();
      }
      vTaskDelay(1/portTICK_PERIOD_MS);
    }

    close(sock);
    Serial.println("TCP disconnected, retrying...");
    vTaskDelay(2000/portTICK_PERIOD_MS);
  }
}

//==================================================================
// setup & loop
//==================================================================
void setup() {
  Serial.begin(115200);
  delay(500);
  wifi_init_sta();
  uart_init_custom();

  // ADC 初始化
  analogSetPinAttenuation(ANALOG_PIN, ADC_11db);
  analogReadResolution(12);

  // 配置 IMU
  configure_imu();

  // 启动任务
  xTaskCreate(uart_event_task, "uart_evt", 4096, NULL, 12, NULL);
  xTaskCreate(tcp_client_task,  "tcp_cli", 4096, NULL, 10, NULL);
}

void loop() {
  // all work done in tasks
  delay(1000);
}
