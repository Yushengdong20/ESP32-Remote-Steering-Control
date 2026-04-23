/*
 * ESP32 50 Hz PWM 控制台（0~100% 版）
 * GPIO25 → 模块 PT
 * 串口输入 0~100 整数，回车生效
 */

// 顺时针
const uint8_t PWM_PIN   = 25;
// 逆时针
// const uint8_t PWM_PIN   = 34;

const uint8_t PWM_CH    = 0;
const uint32_t PWM_FREQ = 50;   // 50 Hz
const uint8_t PWM_RES   = 8;    // 8 bit → 0-255

// 把 0-100 映射到 0-255
inline uint32_t percentToDuty(uint8_t pct) {
  return map(pct, 0, 100, 0, 255);
}

void setup() {
  Serial.begin(115200);
  ledcSetup(PWM_CH, PWM_FREQ, PWM_RES);
  ledcAttachPin(PWM_PIN, PWM_CH);
  Serial.println("输入 0~100 设置占空比（0=停，100=全速）");
}

void loop() {
  if (Serial.available()) {
    int pct = Serial.parseInt();          // 读数字
    if (Serial.read() == '\n') {          // 等完整一行
      pct = constrain(pct, 0, 100);       // 钳位
      uint32_t duty = percentToDuty(pct);
      ledcWrite(PWM_CH, duty);
      Serial.print("占空比 → ");
      Serial.print(pct);
      Serial.println('%');
    }
    while (Serial.read() != -1);          // 清空残留
  }
}