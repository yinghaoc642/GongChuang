
/*****************************************************************************

//stm32 控制四个轮子
*********************/
#include <DDSM210.h>


DDSM_CTRL dc;
#define TJCHMI_BAUDRATE 115200
// device settings.  
#define M0603C_RX PA3
#define M0603C_TX PA2

#define TJCHMI_RX PB15
#define TJCHMI_TX PB14

//                        RX          TX
HardwareSerial Serial_M0603C(M0603C_RX, M0603C_TX);
//                        RX          TX
HardwareSerial Serial_TJCHMI(TJCHMI_RX, TJCHMI_TX);

/****************************电机驱动函数*********************************************
函数功能：驱动两路电机运动
入口参数：motora a电机驱动PWM[-210,210]；motorb  b电机驱动PWM[-210,210]
**************************************************************************/
void RunMotors(int motora, int motorb,int motorc, int motord) {
 // a电机驱动PWM[-2100,2100]；b电机驱动PWM[-2100,2100]
dc.ddsm210_ctrl_4(motora*10,-motorb*10,motorc*10,-motord*10);
}

int a=0;
 unsigned long nowtime;

void setup() {


  //串口初始化
  Serial.begin(115200);


  //电机串口初始化
	// M0603C init.
	Serial_M0603C.begin(DDSM_BAUDRATE);
	//串口屏串口初始化
	Serial_TJCHMI.begin(TJCHMI_BAUDRATE);
	dc.pSerial = &Serial_M0603C;
delay(100);
	// clear M0603C serial buffer.
	dc.clear_ddsm_buffer();
	//串口屏串口清除缓存
	//因为串口屏开机会发送88 ff ff ff,所以要清空串口缓冲区
   while (Serial_TJCHMI.read() >= 0); //清空串口缓冲区
   //Serial_TJCHMI.print("page main\xff\xff\xff"); //发送命令让屏幕跳转到main页面
   nowtime = millis(); //获取当前已经运行的时间

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
  RunMotors(200,-200,200,-200);
 
  char str[100];
  //每100ms更新一次串口屏
   if (millis() >= nowtime + 100) {
     nowtime = millis(); //获取当前已经运行的时间

     //用sprintf来格式化字符串，给n1的val属性赋值
     sprintf(str, "n1.val=%d\xff\xff\xff",  dc.speed_data_4[0]);
     //把字符串发送出去
     Serial_TJCHMI.print(str);

          //用sprintf来格式化字符串，给n1的val属性赋值
     //sprintf(str, "n2.val=%d\xff\xff\xff", a);
     //把字符串发送出去
     //Serial_TJCHMI.print(str);

     a++;
   }
}