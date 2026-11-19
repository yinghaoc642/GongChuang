//使用V2版库，可自定义通讯串口
//当前bug:打开电源顺序bug，先开总电源， 再开stm32板电源，程序可能会卡住。出现这种情况时，关闭stm32板电源会导致舵机驱动板电源指示灯熄灭。怀疑是从舵机驱动板取了电。
#include <JY901.h>
#include <DDSM210.h>
#include <FashionStar_SmartGripper.h>  // Fashion Star智能夹具
#include "OneButton.h"
#include <HardwareTimer.h>

#define TJCHMI_BAUDRATE 115200  //串口屏幕波特率
#define WTIMU_BAUDRATE 115200   //串口IMU波特率
#define SERVO_BAUDRATE 115200   //串口舵机波特率
#define QR_BAUDRATE 9600        //串口扫码模块波特率 默认波特率9600
// device settings.
#define M0603C_RX PA3
#define M0603C_TX PA2

#define TJCHMI_RX PB15
#define TJCHMI_TX PB14

#define WTIMU_RX PD9
#define WTIMU_TX PD8

#define SERVO_RX PC7
#define SERVO_TX PC6

#define QR_RX PE0
#define QR_TX PE1
//舵机编号常量符号化
// 串口总线舵机配置
#define ARM_BASE_SERVO_ID 0  // 舵机0的ID号 基座舵机
#define GRIPPER_SERVO_ID 4   // 舵机4的ID号 手爪
#define STORAGE_SERVO_ID 5   // 舵机5的ID号 载物盘舵机
#define USE_ARM_A            //使用主力机械臂A



/**主力机械臂A舵机参数****************************************/
// 爪子的配置
#ifdef USE_ARM_A
#define GRIPPER_OPEN_ANGLE -10.0   // 爪子张开时的角度
#define GRIPPER_CLOSE_ANGLE -96.0  // 爪子闭合时的角度
#define GRIPPER_OPEN_MAX_ANGLE 30  //爪子张开最大大角度

#define ARM_BASE_SERVO_HOME_ANGLE -90  //机械臂发车初始角度
#define ARM_BASE_SERVO_OUT_ANGLE 0     //机械臂底部舵机朝车外角度
#define ARM_BASE_SERVO_IN_ANGLE -108   //机械臂底部舵机朝车内的角度

#endif

#define START_BTN PB9  //v1 PB8 V2 PB9 根据实际按键引脚修改
// 创建一个HardwareTimer对象，选择使用TIM3
HardwareTimer myTimer(TIM3);
// 定义任务的时间间隔（以毫秒为单位）
#define TASK_TJCHMI_INTERVAL_MS 100
#define TASK_M0603C_INTERVAL_MS 20
#define TASK3_INTERVAL_MS 1000
// 定义任务计数器
volatile unsigned long task_TJCHMI_Counter = 0;
volatile unsigned long task_M0603C_Counter = 0;
volatile unsigned long task3Counter = 0;
// 定义任务标志，用于在主循环中执行任务
volatile bool task_TJCHMI_Flag = false;
volatile bool task_M0603C_Flag = false;
volatile bool task3Flag = false;


// 定时器中断回调函数
void TimerCallback() {
  // 每1ms调用一次
  task_TJCHMI_Counter++;
  task_M0603C_Counter++;
  task3Counter++;

  // 检查是否达到任务1的执行间隔
  if (task_TJCHMI_Counter >= TASK_TJCHMI_INTERVAL_MS) {
    task_TJCHMI_Flag = true;
    task_TJCHMI_Counter = 0;
  }

  // 检查是否达到任务2的执行间隔
  if (task_M0603C_Counter >= TASK_M0603C_INTERVAL_MS) {
    task_M0603C_Flag = true;
    task_M0603C_Counter = 0;
  }

  // 检查是否达到任务3的执行间隔
  if (task3Counter >= TASK3_INTERVAL_MS) {
    task3Flag = true;
    task3Counter = 0;
  }
}


//  explicit OneButton(const int pin, const bool activeLow = true, const bool pullupActive = true);
OneButton start_btn(START_BTN, true, true);  // true:按下为低电平
bool start_flag = 0;

//        0出发位置   1R  2G  3B  储物盘三个盘位正对机械臂的角度
int storage[4] = { -3, 9, -81, -171 };

//                               RX          TX
HardwareSerial Serial_M0603C(M0603C_RX, M0603C_TX);
//                              RX          TX
HardwareSerial Serial_TJCHMI(TJCHMI_RX, TJCHMI_TX);
//                              RX          TX
HardwareSerial Serial_WTIMU(WTIMU_RX, WTIMU_TX);
//             串口舵机          RX          TX
HardwareSerial Serial_SERVO(SERVO_RX, SERVO_TX);
//         串口扫码模块     RX     TX
HardwareSerial Serial_QR(QR_RX, QR_TX);
/**舵机***************************************************/
// 创建舵机的通信协议对象
FSUS_Protocol protocol(&Serial_SERVO, SERVO_BAUDRATE);  //协议V2版本新增

//FSUS_Servo armBaseServo(ARM_BASE_SERVO_ID, &protocol);  // 机械臂舵机

//FSUS_Servo storageServo(STORAGE_SERVO_ID, &protocol);  // 载物盘舵机

FSUS_Servo gripperServo(GRIPPER_SERVO_ID, &protocol);  // 手爪

// 创建智能机械爪实例
FSGP_Gripper gripper(&gripperServo, GRIPPER_OPEN_ANGLE, GRIPPER_CLOSE_ANGLE);

DDSM_CTRL dc;
float yaw;   // 当前角度数据
int yaw100;  //虚拟浮点数串口屏显示用
float Gyro;  // 当前角加速度
int a = 0;
unsigned long nowtime;

//扫码模块相关
// 33 32 31 2B 31 32 33 0D 0A
// GM75默认是CR
// 设置后可改为CRLF
// CR（Carriage Return），回车符，用符号’\r’表示， 十进制ASCII代码是13，16进制0x0D；
// LF（Line Feed），换行符，用符号’\n’表示，十进制ASCII代码是10，16进制0x0A；
const int bufferSize = 8;       // 7字节数据 + 1字节回车符+（1字节换行符）
char receivedData[bufferSize];  // 存储接收到的数据
char strQR[] = "000+000";
bool scanFlag = false;
int dataIndex = 0;  // 数据索引
/****************************电机驱动函数*********************************************
函数功能：驱动两路电机运动
入口参数：motora a电机驱动PWM[-210,210]；motorb  b电机驱动PWM[-210,210]
**************************************************************************/
void RunMotors(int motora, int motorb, int motorc, int motord) {
  // a电机驱动PWM[-2100,2100]；b电机驱动PWM[-2100,2100]
  dc.ddsm210_ctrl_4(motora * 10, -motorb * 10, motorc * 10, -motord * 10);
}
void arm_go_home();
void arm_out();
void arm_in();

void start_click() {
  start_flag = 1;
  // digitalWrite(EN_PIN, LOW);
  // save_flag=1;
}
// 定义任务函数
void Task_TJCHMI() {
  char str[100];
  //用sprintf来格式化字符串，给n1的val属性赋值
  //todo:如果值不变不刷新
  sprintf(str, "n1.val=%d\xff\xff\xff", dc.speed_data_4[0]);
  //把字符串发送出去
  Serial_TJCHMI.print(str);
  sprintf(str, "n2.val=%d\xff\xff\xff", dc.speed_data_4[1]);
  //把字符串发送出去
  Serial_TJCHMI.print(str);
  sprintf(str, "n3.val=%d\xff\xff\xff", dc.speed_data_4[2]);
  //把字符串发送出去
  Serial_TJCHMI.print(str);
  sprintf(str, "n4.val=%d\xff\xff\xff", dc.speed_data_4[3]);
  //把字符串发送出去
  Serial_TJCHMI.print(str);

  //用sprintf来格式化字符串，给x0的val属性赋值
  sprintf(str, "x0.val=%d\xff\xff\xff", yaw100);
  //把字符串发送出去
  Serial_TJCHMI.print(str);

  //用sprintf来格式化字符串，给n1的val属性赋值
  //sprintf(str, "n2.val=%d\xff\xff\xff", a);
  //把字符串发送出去
  //Serial_TJCHMI.print(str);
  if (start_flag) {
    start_flag=false;
    Serial_TJCHMI.print("t7.txt=\"V2_ON\"\xff\xff\xff");
  }

  if (scanFlag) {
    scanFlag=false;
    Serial_TJCHMI.print("t1.txt=\"QROK\"\xff\xff\xff");
    //刷新屏幕显示
    char strTemp[20];
    sprintf(strTemp, "t3.txt=\"%s\"\xff\xff\xff", receivedData);
    //把字符串发送出去
    Serial_TJCHMI.print(strTemp);
  }

  else {
    Serial_TJCHMI.print("t3.txt=\"123+123\"\xff\xff\xff");
  }
}

void Task_M0603C() {

  RunMotors(50, -50, 50, -50);
}
void setup() {

  /*--------------硬件串口初始化----------------*/
  //Serial.begin(115200);
  protocol.init(&Serial_SERVO, SERVO_BAUDRATE);  // 舵机通信协议初始化
  //armBaseServo.init();                           // 机械臂旋转基座舵机初始化
  //storageServo.init();                           // 储物盘舵机初始化
  gripper.init();  // 手爪舵机初始化，原始程序爪子会开启

  // 参数配置
  gripper.setMaxPower(400);  // 设置最大功率，单位mW
  //armBaseServo.setSpeed(500);  // 舵机0初始化 机械臂舵机
  //storageServo.setSpeed(500);  // 舵机1初始化 储物盘


  //   hwt101
  Serial_WTIMU.begin(WTIMU_BAUDRATE);
  //电机串口初始化
  // M0603C init.
  Serial_M0603C.begin(DDSM_BAUDRATE);
  //串口屏串口初始化
  Serial_TJCHMI.begin(TJCHMI_BAUDRATE);
  Serial_QR.begin(QR_BAUDRATE);
  dc.pSerial = &Serial_M0603C;

  protocol.init(&Serial_SERVO, SERVO_BAUDRATE);  // 舵机通信协议初始化
  //armBaseServo.init();                           // 机械臂旋转基座舵机初始化
  //storageServo.init();                           // 储物盘舵机初始化
  // 参数配置
  gripper.setMaxPower(400);  // 设置最大功率，单位mW
  gripper.init();            // 手爪舵机初始化，原始程序爪子会开启

  start_btn.reset();  // 清除一下按钮状态机的状态
  start_btn.attachClick(start_click);
  delay(100);


  //armBaseServo.setSpeed(500);  // 舵机0初始化 机械臂舵机
  //storageServo.setSpeed(500);  // 舵机1初始化 储物盘

  arm_go_home();
  delay(400);

  // clear M0603C serial buffer.
  dc.clear_ddsm_buffer();
  //串口屏串口清除缓存
  //因为串口屏开机会发送88 ff ff ff,所以要清空串口缓冲区
  while (Serial_TJCHMI.read() >= 0)
    ;                                            //清空串口缓冲区
  Serial_TJCHMI.print("page main\xff\xff\xff");  //发送命令让屏幕跳转到main页面
                                                 // 配置定时器为1kHz（1ms周期）
  myTimer.setOverflow(1000, HERTZ_FORMAT);       // 1kHz
  myTimer.attachInterrupt(TimerCallback);        // 附加中断回调
  myTimer.setInterruptPriority(1, 0);            // 设置中断优先级（可选）
  myTimer.resume();                              // 启动定时器

  nowtime = millis();  //获取当前已经运行的时间
}

void loop() {
  //检测按键是否按下
  start_btn.tick();
  //如果未曾扫码成功则接收串口扫码值,只接收一次
  if (scanFlag == false) {
    while (Serial_QR.available()) {
      char incomingByte = Serial_QR.read();  // 读取一个字节数据

      // 检查是否接收到换行符，如果是换行符则重新开始
      if (incomingByte == 0x0A) {
        dataIndex = 0;  // 重置数据索引
      } else {
        // 保存字符
        if (dataIndex < (bufferSize - 1)) {
          receivedData[dataIndex] = incomingByte;  // 将数据存储到数组中
          dataIndex++;
        }
        if (incomingByte == 0x0D) {
          receivedData[dataIndex] = '\0';  // 在数据末尾添加字符串结束符
          dataIndex = 0;                   // 重置数据索引
          scanFlag = true;                 //数据接收成功
                                           // 处理接收到的数据，可以在这里添加你的处理逻辑

          //strcpy(strQR, receivedData);
        }
      }
    }
  }

  /***  hwt101只能输出z轴角度，角加速度  ***/
  // 串口接收到数据后，进行数据的读取与存储。
  while (Serial_WTIMU.available()) {
    JY901.CopeSerialData(Serial_WTIMU.read());  //Call JY901 data cope function
  }

  // 储存数据 角度值
  yaw = (float)JY901.stcAngle.Angle[2] / 32768 * 180;
  yaw100 = (int)(yaw * 100);







  //每100ms更新一次串口屏,定时器版
  if (task_TJCHMI_Flag) {
    task_TJCHMI_Flag = false;
    Task_TJCHMI();
  }
    //每20ms更新一次电机速度
  if (task_M0603C_Flag) {
    task_M0603C_Flag = false;
    Task_M0603C();
  }
}



void arm_go_home() {


  gripper.close();
  //armBaseServo.setAngle(ARM_BASE_SERVO_HOME_ANGLE);
  //storageServo.setAngle(storage[0]);
  //armBaseServo.wait();
  //storageServo.wait();
}

void arm_out() {

  //stepper.runToNewPosition();
  gripper.open(GRIPPER_OPEN_MAX_ANGLE);
  //armBaseServo.setAngle(ARM_BASE_SERVO_OUT_ANGLE);
  //armBaseServo.wait();
}

void arm_in() {

  //stepper.runToNewPosition();
  //gripper.close();
  //armBaseServo.setAngle(ARM_BASE_SERVO_IN_ANGLE);
  //armBaseServo.wait();
}