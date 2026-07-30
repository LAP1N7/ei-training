// day4 — 네트워크 진단. 카메라도 EI 도 없이 Wi-Fi 와 TCP 만 확인한다.
//
// 시각 노드가 "[WIFI] ok IP=192.168.0.11" 을 찍는데도 노트북에서 ping/ARP 가 전혀
// 안 되는 원인을 가르기 위한 스케치. 두 가지를 동시에 확인한다:
//   1) 어느 AP(BSSID/채널/RSSI)에 붙었고 게이트웨이·서브넷이 무엇인지
//      -> 노트북과 다른 AP 라면 그게 원인
//   2) 카메라를 안 돌려도 브로커로 TCP 가 열리는지
//      -> 여기서 되면 카메라(전원/PSRAM)가 범인, 안 되면 망 문제
//
// loop 에서 반복 출력한다 (setup 에서만 찍으면 USB CDC 재연결 전에 날아간다).
#include <WiFi.h>

const char *WIFI_SSID = "projectbee";
const char *WIFI_PASS = "honeybear!";
const char *MQTT_HOST = "192.168.0.27";
const int   MQTT_PORT = 1883;

void setup() {
  Serial.begin(115200);
  Serial.setTxTimeoutMs(0);
  delay(500);
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
}

void loop() {
  Serial.println("\n===== NET PROBE =====");
  Serial.printf("status   : %d (%s)\n", WiFi.status(),
                WiFi.status() == WL_CONNECTED ? "CONNECTED" : "not connected");

  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("Wi-Fi 미연결 - 재시도");
    WiFi.disconnect();
    WiFi.begin(WIFI_SSID, WIFI_PASS);
    delay(5000);
    return;
  }

  Serial.printf("SSID     : %s\n", WiFi.SSID().c_str());
  Serial.printf("BSSID    : %s   <- 붙은 AP. 노트북과 다르면 이게 원인\n",
                WiFi.BSSIDstr().c_str());
  Serial.printf("channel  : %d\n", WiFi.channel());
  Serial.printf("RSSI     : %d dBm\n", WiFi.RSSI());
  Serial.printf("IP       : %s\n", WiFi.localIP().toString().c_str());
  Serial.printf("gateway  : %s\n", WiFi.gatewayIP().toString().c_str());
  Serial.printf("subnet   : %s\n", WiFi.subnetMask().toString().c_str());
  Serial.printf("DNS      : %s\n", WiFi.dnsIP().toString().c_str());
  Serial.printf("MAC      : %s\n", WiFi.macAddress().c_str());
  Serial.printf("free heap: %u B (max block %u B)\n",
                ESP.getFreeHeap(), ESP.getMaxAllocHeap());

  // 게이트웨이가 먼저 살아있는지 본 다음 브로커를 찔러본다.
  // 게이트웨이도 안 되면 망 자체가 안 붙은 것이고, 게이트웨이만 되면 브로커/방화벽 문제다.
  WiFiClient c;
  Serial.printf("TCP %s:80   -> ", WiFi.gatewayIP().toString().c_str());
  Serial.println(c.connect(WiFi.gatewayIP(), 80, 3000) ? "OK" : "FAIL");
  c.stop();

  WiFiClient m;
  Serial.printf("TCP %s:%d -> ", MQTT_HOST, MQTT_PORT);
  Serial.println(m.connect(MQTT_HOST, MQTT_PORT, 3000) ? "OK  <- 브로커 도달 가능" : "FAIL");
  m.stop();

  delay(5000);
}
