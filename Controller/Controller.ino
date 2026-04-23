#include <Wire.h>

// ============== 硬件定义 ==============
#define CH1_PIN 16
#define PCF8591_ADDR 0x48

// ============== 遥控映射参数 ==============
#define RAW_MIN   970
#define RAW_MID   1470
#define RAW_MAX   1970
#define DEAD_BAND 30

// ============== ADC映射参数 ==============
// 电位器：0V(-100%) → 3.3V(+100%)，中间1.65V对应0%
// ADC值0-255，0对应-100%，255对应+100%
#define ADC_LEFT   0    // 实际最左端ADC值（略大于0）
#define ADC_RIGHT  255   // 实际最右端ADC值（略小于255）
#define ADC_CENTER 127   // 理论中点

// ============== PID参数 ==============
#define KP 2.0f          // 比例系数
#define KI 0.1f          // 积分系数  
#define KD 0.5f          // 微分系数
#define OUTPUT_LIMIT 100 // 输出限幅 ±100%

// ============== 全局变量 ==============
volatile uint32_t pwmStart = 0;
volatile uint16_t pwmValue = RAW_MID;


float integral = 0;
float lastError = 0;
uint32_t lastTime = 0;

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

// 改进版PID：带条件积分和积分分离
float calculatePID(float target, float current) {
    uint32_t now = millis();
    float dt = (now - lastTime) / 1000.0f;
    lastTime = now;
    if (dt < 0.001f) dt = 0.001f;
    
    float error = target - current;
    
    // 条件积分：仅当误差较小(|e|<20)且未饱和时才累积
    // 同时引入积分分离：大误差时不积分
    float outputBeforeIntegral = KP * error + KD * ((error - lastError) / dt);
    
    if (abs(error) < 20.0f && abs(outputBeforeIntegral) < OUTPUT_LIMIT * 0.8f) {
        integral += error * dt;
    } else {
        // 大误差或饱和时，逐步衰减积分（而不是清零，避免突变）
        integral *= 0.95f;
    }
    
    // 严格的积分限幅
    integral = constrain(integral, -30.0f, 30.0f);
    
    // 如果误差几乎为零，强制清零积分（彻底消除残差）
    if (abs(error) < 0.5f) {
        integral = 0;
        lastError = 0;
        return 0;  // 死区内直接输出0
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
    
    delay(1000);
    Serial.println("=== PID位置闭环===");
    Serial.println("格式: 目标% | 实际% | 误差 | 积分项 | PID输出 | 动作 | ADC");
    
    pinMode(CH1_PIN, INPUT_PULLDOWN);
    attachInterrupt(digitalPinToInterrupt(CH1_PIN), onPWM, CHANGE);
    
    lastTime = millis();
}

void loop() {
    static uint32_t lastPrint = 0;
    
    if (millis() - lastPrint >= 50) {  // 20Hz显示
        lastPrint = millis();
        
        noInterrupts();
        uint16_t rawPwm = pwmValue;
        interrupts();
        
        float target = getTargetPosition();
        float current = getCurrentPosition();
        uint8_t rawADC = readADC(0);
        
        float output = calculatePID(target, current);
        const char* action = interpretOutput(output, 2.0f);  // 阈值±2%
        
        Serial.printf("目标:%+5.1f | 实际:%+5.1f | 误差:%+5.1f | 积分项:%+5.1f | PID输出:%+5.1f | 动作:%s | %3d\n", 
                      target, current, target - current, integral, output, action, rawADC);
    }
    
    delay(1);
}