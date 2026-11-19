/********************************************
//  tips: 使用mega2560的硬件串口三与hwt101进行通讯
//  参考文章： https://zhuanlan.zhihu.com/p/367614669
//  联系方式： 知乎 “Poao”
//                    2021/5/11  —— by Poao
**********************************************/

/************ HWT101- Serial2 *************/

#include <JY901.h>
#include <DDSM210.h>


#define TJCHMI_BAUDRATE 115200 //串口屏幕波特率
#define WTIMU_BAUDRATE 115200 //串口IMU波特率
#define SERVO_BAUDRATE 115200 //串口舵机波特率
// device settings.  
#define M0603C_RX PA3
#define M0603C_TX PA2

#define TJCHMI_RX PB15
#define TJCHMI_TX PB14

#define WTIMU_RX  PD9
#define WTIMU_TX PD8

#define SERVO_RX  PC7
#define SERVO_TX PC6

//                               RX          TX
HardwareSerial Serial_M0603C(M0603C_RX, M0603C_TX);
//                              RX          TX
HardwareSerial Serial_TJCHMI(TJCHMI_RX, TJCHMI_TX);
//                              RX          TX
HardwareSerial Serial_WTIMU(WTIMU_RX, WTIMU_TX);
//             串口舵机          RX          TX
HardwareSerial Serial_SERVO(SERVO_RX, SERVO_TX);

DDSM_CTRL dc;
float yaw;   // 当前角度数据  
int yaw100;//虚拟浮点数串口屏显示用
float Gyro;  // 当前角加速度  
int a=0;
unsigned long nowtime;
/****************************电机驱动函数*********************************************
函数功能：驱动两路电机运动
入口参数：motora a电机驱动PWM[-210,210]；motorb  b电机驱动PWM[-210,210]
**************************************************************************/
void RunMotors(int motora, int motorb,int motorc, int motord) {
 // a电机驱动PWM[-2100,2100]；b电机驱动PWM[-2100,2100]
dc.ddsm210_ctrl_4(motora*10,-motorb*10,motorc*10,-motord*10);
}



void setup() {

  /*--------------硬件串口初始化----------------*/
  Serial.begin(115200);

  //   hwt101 
  Serial_WTIMU.begin(WTIMU_BAUDRATE);
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
   Serial_TJCHMI.print("page main\xff\xff\xff"); //发送命令让屏幕跳转到main页面
   nowtime = millis(); //获取当前已经运行的时间
  
}

void loop() {

  /***  hwt101只能输出z轴角度，角加速度  ***/
  // 串口接收到数据后，进行数据的读取与存储。
   while (Serial_WTIMU.available()) 
  {
    JY901.CopeSerialData(Serial_WTIMU.read()); //Call JY901 data cope function
   }
   
  // 储存数据 角度值
  yaw = (float)JY901.stcAngle.Angle[2]/32768*180;
 yaw100=(int)(yaw*100);




  RunMotors(50,-50,50,-50);
 

  //每100ms更新一次串口屏
   if (millis() >= nowtime + 100) {
     nowtime = millis(); //获取当前已经运行的时间
  char str[100];
     //用sprintf来格式化字符串，给n1的val属性赋值
     sprintf(str, "n1.val=%d\xff\xff\xff",  dc.speed_data_4[0]);
     //把字符串发送出去
     Serial_TJCHMI.print(str);

          //用sprintf来格式化字符串，给x0的val属性赋值
     sprintf(str, "x0.val=%d\xff\xff\xff",  yaw100);
     //把字符串发送出去
     Serial_TJCHMI.print(str);

          //用sprintf来格式化字符串，给n1的val属性赋值
     //sprintf(str, "n2.val=%d\xff\xff\xff", a);
     //把字符串发送出去
     //Serial_TJCHMI.print(str);

     a++;
   }

}
