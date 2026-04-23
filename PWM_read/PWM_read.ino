// 读取接收机数据
/*
 * HOTRC F-06A 接收机 - ESP32 单通道PWM解析 (CH1)
 */
#define CH1_PIN 16

volatile uint32_t pwmStart = 0;
volatile uint16_t pwmValue = 1425;

void IRAM_ATTR onPWM() {
  if (digitalRead(CH1_PIN) == HIGH) {
    pwmStart = micros();
  } else {
    uint32_t pulse = micros() - pwmStart;
    if (pulse > 800 && pulse < 2200) {
      pwmValue = (uint16_t)pulse;
    }
  }
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("=== HOTRC F-06A CH1 Receiver Decoder ===");
  Serial.println("PWM Input Mode");
  Serial.println("=====================================");

  pinMode(CH1_PIN, INPUT_PULLDOWN);
  attachInterrupt(digitalPinToInterrupt(CH1_PIN), onPWM, CHANGE);

  Serial.println("Setup complete. Waiting for signal...");
  Serial.println("Format: CH1");
}

void loop() {
  static uint32_t lastPrint = 0;
  
  if (millis() - lastPrint >= 100) {
    lastPrint = millis();
    
    uint16_t value = pwmValue;

    // 打印格式：PWM: CH1:1425 | MAP: +0% [OK]
    Serial.printf("PWM: CH1:%4d | MAP: ", value);
    
    // 校准映射（基于你的实际测量值）
    const int RAW_MIN = 925;
    const int RAW_MAX = 1925;
    const int RAW_MID = 1425;
    
    int mapped;
    if (value < RAW_MID) {
      mapped = map(value, RAW_MIN, RAW_MID, -100, 0);
    } else {
      mapped = map(value, RAW_MID, RAW_MAX, 0, 100);
    }
    mapped = constrain(mapped, -100, 100);
    
    Serial.printf("%+4d ", mapped);
    
    bool valid = (value >= 900 && value <= 2100);
    Serial.print(valid ? "[OK]" : "[LOST]");
    Serial.println();
  }

  delay(1);
}