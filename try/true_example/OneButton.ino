//当前bug:打开电源顺序bug，先开总电源， 再开stm32板电源，程序可能会卡住。出现这种情况时，关闭stm32板电源会导致舵机驱动板电源指示灯熄灭。怀疑是从舵机驱动板取了电。
#include "OneButton.h"
#define LED_PIN PA15 //PA15 黄灯  PB4 蓝灯

#define START_BTN PB9                        //v1 PB8 V2 PB9 V3 PB9/PB4根据实际按键引脚修改
OneButton start_btn(START_BTN, true, true);  // true:按下为低电平
bool start_flag = 0;

void start_click() {
  start_flag = 1;
  // digitalWrite(EN_PIN, LOW);
  // save_flag=1;
}

void setup() {

 pinMode(LED_PIN, OUTPUT);
  start_btn.reset();  // 清除一下按钮状态机的状态
  start_btn.attachClick(start_click);
  delay(100);

}

void loop() {
  //检测按键是否按下
  start_btn.tick();
  //如果未曾扫码成功则接收串口扫码值,只接收一次
  if (start_flag == 1) {

digitalWrite(LED_PIN, HIGH);
      }
    
  }