/*
 * ESP32 双路 50 Hz PWM 控制台（0~100% 版）
 * GPIO25 → 模块 PT（电机A）
 * GPIO34 → 模块 PT（电机B）
 * 串口输入格式: "数字 数字"（如 "50 80"），空格分隔，回车生效
 */

// 电机A - GPIO25（顺时针）
const uint8_t PWM_PIN_A   = 25;
// 电机B - GPIO34（逆时针）
const uint8_t PWM_PIN_B   = 26;
const uint8_t PWM_CH_0    = 0;
const uint8_t PWM_CH_1    = 1;
const uint32_t PWM_FREQ   = 50;   // 50 Hz
const uint8_t PWM_RES     = 8;    // 8 bit → 0-255

// 把 0-100 映射到 0-255
inline uint32_t percentToDuty(uint8_t pct) {
  return map(pct, 0, 100, 0, 255);
}

void setup() {
  Serial.begin(115200);

  // 初始化通道1
  ledcSetup(PWM_CH_0, PWM_FREQ, PWM_RES);
  ledcAttachPin(PWM_PIN_A, PWM_CH_0);
  
  // 初始化通道2
  ledcSetup(PWM_CH_1, PWM_FREQ, PWM_RES);
  ledcAttachPin(PWM_PIN_B, PWM_CH_1);


  Serial.println("双路PWM控制器已启动");
  Serial.println("输入格式: 数字A 数字B 如 50 80 ");
  Serial.println("范围: 0~100（0=停，100=全速）");
}

void loop() {
  if (Serial.available()) {
    // 读取第一个数字（电机A）
    int pctA = Serial.parseInt();

    // 读取第二个数字（电机B）
    int pctB = Serial.parseInt();

    // 等待换行符表示输入结束
    if (Serial.read() == '\n') {
      // 钳位到有效范围
      pctA = constrain(pctA, 0, 100);
      pctB = constrain(pctB, 0, 100);

      // 计算占空比
      uint32_t dutyA = percentToDuty(pctA);
      uint32_t dutyB = percentToDuty(pctB);

      // Arduino 3.0+ API: ledcWrite(pin, duty)
      ledcWrite(PWM_CH_0, dutyA);
      ledcWrite(PWM_CH_1, dutyB);

      // 打印状态
      Serial.print("电机A: ");
      Serial.print(pctA);
      Serial.print("%  |  电机B: ");
      Serial.print(pctB);
      Serial.println("%");
    }

    // 清空串口缓冲区残留数据
    while (Serial.read() != -1);
  }
}