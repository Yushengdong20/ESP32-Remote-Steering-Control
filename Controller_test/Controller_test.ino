#include <Wire.h>

// ============== 硬件定义 ==============
#define CH1_PIN 16
#define PCF8591_ADDR 0x48

// 电机A - GPIO25（顺时针/右转）
const uint8_t PWM_PIN_A   = 25;
// 电机B - GPIO34（逆时针/左转）
const uint8_t PWM_PIN_B   = 26;

const uint8_t PWM_CH_0    = 0;
const uint8_t PWM_CH_1    = 1;
const uint32_t PWM_FREQ   = 50;   // 50 Hz
const uint8_t PWM_RES     = 8;    // 8 bit → 0-255

// ============== 遥控映射参数 ==============
#define RAW_MIN   970
#define RAW_MID   1470
#define RAW_MAX   1970
#define DEAD_BAND 30

// ============== ADC映射参数 ==============
#define ADC_LEFT   0
#define ADC_RIGHT  255
#define ADC_CENTER 127

// ============== PID参数 ==============
#define KP 2.0f
#define KI 0.1f
#define KD 0.5f
#define OUTPUT_LIMIT 100

// ============== 全局变量 ==============
volatile uint32_t pwmStart = 0;
volatile uint16_t pwmValue = RAW_MID;

float integral = 0;
float lastError = 0;
uint32_t lastTime = 0;

// 把 0-100 映射到 0-255
inline uint32_t percentToDuty(uint8_t pct) {
  return map(pct, 0, 100, 0, 255);
}

// 将PID输出(-100~+100)转换为双路PWM控制
// 输出-100: 左转 → A关0%, B开100%
// 输出+100: 右转 → A开100%, B关0%
// 输出0: 停止 → 都关0%
void applyPIDOutput(float pidOutput) {
    // 钳位到有效范围
    pidOutput = constrain(pidOutput, -OUTPUT_LIMIT, OUTPUT_LIMIT);
    
    uint8_t pctA = 0;  // 电机A百分比（右转）
    uint8_t pctB = 0;  // 电机B百分比（左转）
    
    if (pidOutput > 0) {
        // 右转：A开，B关
        pctA = (uint8_t)pidOutput;
        pctB = 0;
    } else if (pidOutput < 0) {
        // 左转：A关，B开
        pctA = 0;
        pctB = (uint8_t)(-pidOutput);  // 取绝对值
    }
    // pidOutput == 0 时，两者都为0（停止）
    
    // 计算占空比并输出
    uint32_t dutyA = percentToDuty(pctA);
    uint32_t dutyB = percentToDuty(pctB);
    
    ledcWrite(PWM_CH_0, dutyA);
    ledcWrite(PWM_CH_1, dutyB);
}

void IRAM_ATTR onPWM() {
    if (digitalRead(CH1_PIN) == HIGH) {
        pwmStart = micros();
    } else {
        uint32_t pulse = micros() - pwmStart;
        if (pulse > 800 && pulse < 2200) pwmValue = pulse;
    }
}

uint8_t readADC(uint8_t ch) {
    Wire.beginTransmission(PCF8591_ADDR);
    Wire.write(0x04 | ch);
    Wire.endTransmission();
    delayMicroseconds(200);
    Wire.requestFrom(PCF8591_ADDR, 2);
    Wire.read();
    return Wire.read();
}

float getTargetPosition() {
    uint16_t raw = pwmValue;
    if (abs((int)raw - RAW_MID) < DEAD_BAND) return 0;
    
    if (raw < RAW_MID) {
        return map(raw, RAW_MIN, RAW_MID, -100, 0);
    } else {
        return map(raw, RAW_MID, RAW_MAX, 0, 100);
    }
}

float getCurrentPosition() {
    uint8_t adc = readADC(0);
    return map(constrain(adc, ADC_LEFT, ADC_RIGHT), ADC_LEFT, ADC_RIGHT, -100, 100);
}

float calculatePID(float target, float current) {
    uint32_t now = millis();
    float dt = (now - lastTime) / 1000.0f;
    lastTime = now;
    if (dt < 0.001f) dt = 0.001f;
    
    float error = target - current;
    
    float outputBeforeIntegral = KP * error + KD * ((error - lastError) / dt);
    
    if (abs(error) < 20.0f && abs(outputBeforeIntegral) < OUTPUT_LIMIT * 0.8f) {
        integral += error * dt;
    } else {
        integral *= 0.95f;
    }
    
    integral = constrain(integral, -30.0f, 30.0f);
    
    if (abs(error) < 0.5f) {
        integral = 0;
        lastError = 0;
        return 0;
    }
    
    float derivative = (error - lastError) / dt;
    lastError = error;
    
    float output = KP * error + KI * integral + KD * derivative;
    return constrain(output, -OUTPUT_LIMIT, OUTPUT_LIMIT);
}

const char* interpretOutput(float output, float threshold) {
    if (abs(output) < threshold) return "停止";
    return (output > 0) ? "右转" : "左转";
}

void setup() {
    Serial.begin(115200);
    Wire.begin(21, 22);
    
    // 初始化通道1
    ledcSetup(PWM_CH_0, PWM_FREQ, PWM_RES);
    ledcAttachPin(PWM_PIN_A, PWM_CH_0);
    
    // 初始化通道2
    ledcSetup(PWM_CH_1, PWM_FREQ, PWM_RES);
    ledcAttachPin(PWM_PIN_B, PWM_CH_1);

    
    delay(1000);
    Serial.println("=== PID位置闭环 + 双路PWM控制 ===");
    Serial.println("格式: 目标% | 实际% | 误差 | 积分项 | PID输出 | 动作 | A% | B% | ADC");
    
    pinMode(CH1_PIN, INPUT_PULLDOWN);
    attachInterrupt(digitalPinToInterrupt(CH1_PIN), onPWM, CHANGE);
    
    lastTime = millis();
}

void loop() {
    static uint32_t lastPrint = 0;
    
    if (millis() - lastPrint >= 50) {
        lastPrint = millis();
        
        noInterrupts();
        uint16_t rawPwm = pwmValue;
        interrupts();
        
        float target = getTargetPosition();
        float current = getCurrentPosition();
        uint8_t rawADC = readADC(0);
        
        float output = calculatePID(target, current);
        const char* action = interpretOutput(output, 2.0f);
        
        // 应用PID输出到双路PWM
        applyPIDOutput(output);
        
        // 计算当前PWM百分比用于显示
        uint8_t pctA = (output > 0) ? (uint8_t)output : 0;
        uint8_t pctB = (output < 0) ? (uint8_t)(-output) : 0;
        
        Serial.printf("目标:%+5.1f | 实际:%+5.1f | 误差:%+5.1f | 积分项:%+5.1f | PID输出:%+5.1f | 动作:%s | A:%3d%% | B:%3d%% | %3d\n", 
                      target, current, target - current, integral, output, action, pctA, pctB, rawADC);
    }
    
    delay(1);
}