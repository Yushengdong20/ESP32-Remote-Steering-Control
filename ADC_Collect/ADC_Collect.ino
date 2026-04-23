#include <Wire.h>

#define PCF8591_ADDR 0x48

// SDA21, SCL22
uint8_t readADC(uint8_t ch) {
    Wire.beginTransmission(PCF8591_ADDR);
    Wire.write(0x04 | ch);
    Wire.endTransmission();
    delay(5);
    Wire.requestFrom(PCF8591_ADDR, 2);
    Wire.read();
    return Wire.read();
}

void setup() {
    Serial.begin(115200);
    // SDA21, SCL22
    Wire.begin(21, 22);
}

void loop() {
    uint8_t val = readADC(0);
    Serial.print("ADC: "); Serial.print(val);
    Serial.print(" | "); Serial.print(val * 3.3 / 255.0);
    Serial.println("V");
    delay(300);
}