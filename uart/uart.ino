// 串口回传数据
void setup() {
  // 初始化串口，波特率 115200（常用且较快）
  Serial.begin(115200);
  
  // 等待串口连接（仅用于 Leonardo、Micro、ESP32 等 USB 原生串口）
  while (!Serial) {
    ; // 等待串口监视器打开
  }
  
  Serial.println("串口回传程序已启动");
  Serial.println("发送任意数据，将原样返回");
  Serial.println("----------------------------");
}

void loop() {
  // 检查是否有数据可读
  if (Serial.available() > 0) {
    // 读取一个字节
    char incomingByte = Serial.read();
    
    // 立即回传（Echo）
    Serial.write(incomingByte);
    
    // 可选：如果是换行符，额外打印提示
    if (incomingByte == '\n') {
      Serial.println("[回传结束]");
    }
  }
}