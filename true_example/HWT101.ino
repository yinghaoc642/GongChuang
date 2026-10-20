#include <Arduino.h>
#include <JY901.h>
//#include <AccelStepper.h>              //https://github.com/waspinator/AccelStepper

#define TJCHMI_BAUDRATE 115200 //串口屏幕波特率
#define WTIMU_BAUDRATE 115200 //串口IMU波特率

// device settings.  


#define TJCHMI_RX PB15
#define TJCHMI_TX PB14

#define WTIMU_RX  PD9
#define WTIMU_TX PD8

//                              RX          TX
HardwareSerial Serial_TJCHMI(TJCHMI_RX, TJCHMI_TX);
//                              RX          TX
HardwareSerial Serial_WTIMU(WTIMU_RX, WTIMU_TX);



float yaw;   // 当前角度数据  
int yaw100;//虚拟浮点数串口屏显示用
float Gyro;  // 当前角加速度  
int a=0;
unsigned long nowtime;

void setup() {


  //   hwt101 
  Serial_WTIMU.begin(WTIMU_BAUDRATE);
    //电机串口初始化

	//串口屏串口初始化
	Serial_TJCHMI.begin(TJCHMI_BAUDRATE);

delay(100);

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

  //每100ms更新一次串口屏
   if (millis() >= nowtime + 100) {
      char str[100];
     nowtime = millis(); //获取当前已经运行的时间
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
