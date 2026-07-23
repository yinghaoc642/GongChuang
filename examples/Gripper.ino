//使用V2版库，可自定义通讯串口
//当前bug:打开电源顺序bug，先开总电源， 再开stm32板电源，程序可能会卡住。出现这种情况时，关闭stm32板电源会导致舵机驱动板电源指示灯熄灭。怀疑是从舵机驱动板取了电。
#include <Arduino.h>
#include <FashionStar_SmartGripper.h>  // Fashion Star智能夹具


#define SERVO_BAUDRATE 115200   //串口舵机波特率

#define SERVO_RX PC7
#define SERVO_TX PC6

//舵机编号常量符号化
// 串口总线舵机配置
#define GRIPPER_SERVO_ID 4   // 舵机4的ID号 手爪
#define STORAGE_SERVO_ID 5   // 舵机5的ID号 载物盘舵机
#define USE_GRIPPER_A            //使用主力爪A
//#define USE_GRIPPER_B            //使用备用爪B

/**主力主力爪A舵机参数****************************************/
// 爪子的配置
#ifdef USE_GRIPPER_A
#define GRIPPER_OPEN_ANGLE 0.0   // 爪子张开时的角度
#define GRIPPER_CLOSE_ANGLE -90.0  // 爪子闭合时的角度
#define GRIPPER_OPEN_MAX_ANGLE 30  //爪子张开最大大角度
#endif
/**主力备用爪B舵机参数****************************************/
#ifdef USE_GRIPPER_B
#define GRIPPER_OPEN_ANGLE -10.0   // 爪子张开时的角度
#define GRIPPER_CLOSE_ANGLE -96.0  // 爪子闭合时的角度
#define GRIPPER_OPEN_MAX_ANGLE 30  //爪子张开最大大角度
#endif
//                 [0]出发位置   [1]R  [2]G  [3]B  储物盘三个盘位正对机械臂的角度
int storage[4] = { -3, 9, -81, -171 };


//             串口舵机          RX          TX
HardwareSerial Serial_SERVO(SERVO_RX, SERVO_TX);

/**舵机***************************************************/
// 创建舵机的通信协议对象
FSUS_Protocol protocol(&Serial_SERVO, SERVO_BAUDRATE);  //协议V2版本新增

//FSUS_Servo storageServo(STORAGE_SERVO_ID, &protocol);  // 载物盘舵机

FSUS_Servo gripperServo(GRIPPER_SERVO_ID, &protocol);  // 手爪

// 创建智能机械爪实例
FSGP_Gripper gripper(&gripperServo, GRIPPER_OPEN_ANGLE, GRIPPER_CLOSE_ANGLE);


unsigned long nowtime;

//void storage_go_home();

void setup() {

  /*--------------硬件串口初始化----------------*/
  //Serial.begin(115200);
  protocol.init(&Serial_SERVO, SERVO_BAUDRATE);  // 舵机通信协议初始化
  //storageServo.init();                           // 储物盘舵机初始化
  gripper.init();                                // 手爪舵机初始化，原始程序爪子会开启

  // 参数配置
  gripper.setMaxPower(400);    // 设置最大功率，单位mW
  //storageServo.setSpeed(500);  // 舵机1初始化 储物盘


 delay(100);
 
    //storageServo.setSpeed(500);  // 舵机1初始化 储物盘

//storage_go_home();

    nowtime = millis();                            //获取当前已经运行的时间
}

void loop() {
 
  //gripper.open();
  gripper.open(GRIPPER_OPEN_MAX_ANGLE);
  delay(2000);
  gripper.close();
  delay(2000);

}
