#include <WiFi.h>
#include <WiFiUdp.h>
#include <Adafruit_PCA9685.h>

// WiFi credentials
const char* ssid = "YOUR_SSID";
const char* password = "YOUR_PASSWORD";

WiFiUDP Udp;
unsigned int localUdpPort = 4210;  // Port to listen on
char incomingPacket[128];

#define PCA9685_ADDR 0x40
Adafruit_PCA9685 pwm;
#define SERVOMIN  120
#define SERVOMAX  600

void setup() {
  Serial.begin(115200);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi connected");
  Udp.begin(localUdpPort);
  pwm.begin(PCA9685_ADDR);
  pwm.setPWMFreq(60);
}

void setServoAngle(uint8_t channel, float angle) {
  int pulse = map((int)angle, 0, 180, SERVOMIN, SERVOMAX);
  pwm.setPWM(channel, 0, pulse);
}

void loop() {
  int packetSize = Udp.parsePacket();
  if (packetSize) {
    int len = Udp.read(incomingPacket, 128);
    if (len > 0) incomingPacket[len] = 0;
    // Expected format: "channel angle\n" e.g. "3 90\n"
    int ch; float ang;
    if (sscanf(incomingPacket, "%d %f", &ch, &ang) == 2) {
      setServoAngle(ch, ang);
      Serial.printf("Servo %d -> %.1f\n", ch, ang);
    }
  }
}
