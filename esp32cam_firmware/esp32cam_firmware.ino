/*
  Robotic Arm - ESP32-CAM Firmware
  - Streams MJPEG camera feed over HTTP (port 80)
  - Receives servo commands via WebSocket (port 81)
  - Controls 6 servos: base, j1, j2, j3, j4, gripper

  Board: AI-Thinker ESP32-CAM
  Required libs: ESP32 Arduino Core, WebSockets by Links2004
*/

#include "esp_camera.h"
#include <WiFi.h>
#include <WebServer.h>
#include <WebSocketsServer.h>
#include <ESP32Servo.h>

// ── WiFi ──────────────────────────────────────────────────────────────────────
const char* SSID     = "YOUR_SSID";
const char* PASSWORD = "YOUR_PASSWORD";

// ── Camera pins (AI-Thinker module) ──────────────────────────────────────────
#define CAM_PIN_PWDN    32
#define CAM_PIN_RESET   -1
#define CAM_PIN_XCLK     0
#define CAM_PIN_SIOD    26
#define CAM_PIN_SIOC    27
#define CAM_PIN_D7      35
#define CAM_PIN_D6      34
#define CAM_PIN_D5      39
#define CAM_PIN_D4      36
#define CAM_PIN_D3      21
#define CAM_PIN_D2      19
#define CAM_PIN_D1      18
#define CAM_PIN_D0       5
#define CAM_PIN_VSYNC   25
#define CAM_PIN_HREF    23
#define CAM_PIN_PCLK    22

// ── Servo pins (use free GPIO; avoid 0,2,15 strapping pins) ──────────────────
// Note: ESP32-CAM has limited free GPIOs. Use IO12,13,14,15,16 carefully.
// If using external servo driver (PCA9685 via I2C) this section changes.
#define PIN_BASE    12
#define PIN_J1      13
#define PIN_J2      14
#define PIN_J3      15
#define PIN_J4      16   // may need to adjust per board
#define PIN_GRIP     2   // flash LED pin repurposed — disable flash first

Servo servoBase, servoJ1, servoJ2, servoJ3, servoJ4, servoGrip;

// ── Servers ───────────────────────────────────────────────────────────────────
WebServer       httpServer(80);
WebSocketsServer wsServer(81);

// ── Current angles ────────────────────────────────────────────────────────────
int angles[6] = {90, 90, 90, 90, 90, 0};   // base,j1,j2,j3,j4,grip

// ─────────────────────────────────────────────────────────────────────────────
void setupCamera() {
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer   = LEDC_TIMER_0;
  config.pin_d0       = CAM_PIN_D0;
  config.pin_d1       = CAM_PIN_D1;
  config.pin_d2       = CAM_PIN_D2;
  config.pin_d3       = CAM_PIN_D3;
  config.pin_d4       = CAM_PIN_D4;
  config.pin_d5       = CAM_PIN_D5;
  config.pin_d6       = CAM_PIN_D6;
  config.pin_d7       = CAM_PIN_D7;
  config.pin_xclk     = CAM_PIN_XCLK;
  config.pin_pclk     = CAM_PIN_PCLK;
  config.pin_vsync    = CAM_PIN_VSYNC;
  config.pin_href     = CAM_PIN_HREF;
  config.pin_sscb_sda = CAM_PIN_SIOD;
  config.pin_sscb_scl = CAM_PIN_SIOC;
  config.pin_pwdn     = CAM_PIN_PWDN;
  config.pin_reset    = CAM_PIN_RESET;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;
  config.frame_size   = FRAMESIZE_QVGA;   // 320x240 — low latency
  config.jpeg_quality = 15;               // 0=best, 63=worst
  config.fb_count     = 1;

  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed: 0x%x\n", err);
    ESP.restart();
  }
}

// ── MJPEG stream handler ──────────────────────────────────────────────────────
void handleStream() {
  WiFiClient client = httpServer.client();
  String boundary = "frame";

  httpServer.sendContent(
    "HTTP/1.1 200 OK\r\n"
    "Content-Type: multipart/x-mixed-replace;boundary=" + boundary + "\r\n\r\n"
  );

  while (client.connected()) {
    camera_fb_t* fb = esp_camera_fb_get();
    if (!fb) continue;

    String header = "--" + boundary + "\r\n"
                    "Content-Type: image/jpeg\r\n"
                    "Content-Length: " + String(fb->len) + "\r\n\r\n";
    client.print(header);
    client.write(fb->buf, fb->len);
    client.print("\r\n");
    esp_camera_fb_return(fb);
    delay(30);   // ~30 fps cap
  }
}

// ── WebSocket: parse command "base,j1,j2,j3,j4,grip" ─────────────────────────
void onWsEvent(uint8_t num, WStype_t type, uint8_t* payload, size_t length) {
  if (type == WStype_TEXT) {
    String msg = String((char*)payload);
    // Expected format: "90,45,120,90,60,1"
    int idx = 0;
    char* token = strtok((char*)msg.c_str(), ",");
    while (token && idx < 6) {
      angles[idx++] = constrain(atoi(token), 0, 180);
      token = strtok(NULL, ",");
    }
    applyServos();
  }
}

void applyServos() {
  servoBase.write(angles[0]);
  servoJ1.write(angles[1]);
  servoJ2.write(angles[2]);
  servoJ3.write(angles[3]);
  servoJ4.write(angles[4]);
  servoGrip.write(angles[5] ? 60 : 0);   // 60° = closed grip
}

// ─────────────────────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  setupCamera();

  // Servos
  servoBase.attach(PIN_BASE);
  servoJ1.attach(PIN_J1);
  servoJ2.attach(PIN_J2);
  servoJ3.attach(PIN_J3);
  servoJ4.attach(PIN_J4);
  servoGrip.attach(PIN_GRIP);
  applyServos();   // go to home position

  // WiFi
  WiFi.begin(SSID, PASSWORD);
  Serial.print("Connecting");
  while (WiFi.status() != WL_CONNECTED) { delay(500); Serial.print("."); }
  Serial.printf("\nIP: %s\n", WiFi.localIP().toString().c_str());
  Serial.printf("Stream : http://%s/stream\n", WiFi.localIP().toString().c_str());
  Serial.printf("WS     : ws://%s:81\n",       WiFi.localIP().toString().c_str());

  // HTTP
  httpServer.on("/stream", handleStream);
  httpServer.begin();

  // WebSocket
  wsServer.begin();
  wsServer.onEvent(onWsEvent);
}

void loop() {
  httpServer.handleClient();
  wsServer.loop();
}
