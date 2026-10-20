#include <Arduino.h>
#define DEBUG_BAUDRATE 115200 //串口IMU波特率

#define DEBUG_RX PB12 //串口RX
#define DEBUG_TX PB13 //串口TX

// 定义 ADC 引脚
const int voltagePin = PC1;

// 定义分压系数，根据实际电阻计算
const float voltageDividerRatio = 11.0;

// 定义 ADC 分辨率
const int adcResolution = 4096; // 12 位 ADC

// 定义参考电压
const float referenceVoltage = 3.3;
//                              RX          TX
HardwareSerial Serial_DEBUG(DEBUG_RX, DEBUG_TX);
void setup() {
 Serial_DEBUG.begin(DEBUG_BAUDRATE);
  // 初始化 ADC 引脚
  pinMode(voltagePin, INPUT);
   analogReadResolution(12);
}

void loop() {
  // 读取 ADC 值
  int adcValue = analogRead(voltagePin);
    // 计算电压值
  float voltage = (adcValue * referenceVoltage / adcResolution) * voltageDividerRatio;
    // 输出电压值
  Serial_DEBUG.print("Battery Voltage: ");
  Serial_DEBUG.print(voltage);
  Serial_DEBUG.println(" V");
  delay(1000);
}