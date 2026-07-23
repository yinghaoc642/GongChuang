
/*****************************************************************************

//stm32 控制四个轮子
*********************/
#include <DDSM210.h>


DDSM_CTRL dc;

// device settings.  uart2
#define M0603C_RX PA3
#define M0603C_TX PA2

//                        RX          TX
HardwareSerial Serial_M0603C(M0603C_RX, M0603C_TX);

/****************************电机驱动函数*********************************************
函数功能：驱动两路电机运动
入口参数：motora a电机驱动PWM[-210,210]；motorb  b电机驱动PWM[-210,210]
**************************************************************************/
void RunMotors(int motora, int motorb, int motorc, int motord) {
  // a电机驱动PWM[-2100,2100]；b电机驱动PWM[-2100,2100]
  dc.ddsm210_ctrl_4(motora * 10, -motorb * 10, motorc * 10, -motord * 10);
}




void setup() {


  //串口初始化
  //Serial.begin(115200);


  //电机串口初始化
  // M0603C init.
  Serial_M0603C.begin(DDSM_BAUDRATE);
  dc.pSerial = &Serial_M0603C;
  delay(100);
  // clear M0603C serial buffer.
  dc.clear_ddsm_buffer();
  /*
	// M0603C must be stoped to change mode.
	// delay 5ms after changing mode.
	// M0603C MODE_CODE:
	// 0 - open loop
	// 2 - speed loop
	// 3 - position loop
	// args: change_mode(ID, MODE_CODE)
	dc.change_mode(1, 2);
	// change M0603C mode to speed loop.
	delay(5);
  	dc.change_mode(2, 2);
	// change M0603C mode to speed loop.
	delay(5);
  */
}

void loop() {
  RunMotors(50, -50, 50, -50);
}