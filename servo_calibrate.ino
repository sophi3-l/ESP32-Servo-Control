#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm0 = Adafruit_PWMServoDriver(0x40);
Adafruit_PWMServoDriver pwm1 = Adafruit_PWMServoDriver(0x41);

uint16_t degreesToPulse(uint8_t deg) {
  return map(deg, 0, 180, 102, 512);
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  Wire.begin(21, 22);

  pwm0.begin(); pwm0.reset(); delay(100);
  pwm0.setOscillatorFrequency(25000000);
  pwm0.setPWMFreq(50);

  pwm1.begin(); pwm1.reset(); delay(100);
  pwm1.setOscillatorFrequency(25000000);
  pwm1.setPWMFreq(50);

  delay(100);

  Serial.println("Ready — enter: <channel> <degrees>");
  Serial.println("Ch 0-15  → board 0x40");
  Serial.println("Ch 16-31 → board 0x41 (mapped to ch 0-15)");
  Serial.println("Example: 0 90  or  16 90");
}

void loop() {
  if (Serial.available()) {
    String input = Serial.readStringUntil('\n');
    input.trim();
    int spaceIdx = input.indexOf(' ');
    if (spaceIdx > 0) {
      int ch  = input.substring(0, spaceIdx).toInt();
      int deg = input.substring(spaceIdx + 1).toInt();
      if (ch >= 0 && ch <= 31 && deg >= 0 && deg <= 180) {
        uint16_t pulse = degreesToPulse(deg);
        if (ch <= 15) {
          pwm0.setPWM(ch, 0, pulse);
          Serial.printf("Board 0x40 Ch%d → %d° (pulse:%d)\n", ch, deg, pulse);
        } else {
          uint8_t localCh = ch - 16;
          pwm1.setPWM(localCh, 0, pulse);
          Serial.printf("Board 0x41 Ch%d → %d° (pulse:%d)\n", localCh, deg, pulse);
        }
      } else {
        Serial.println("Invalid — channel 0-31, degrees 0-180");
      }
    }
  }
}