#include <Arduino.h>

#define TJCHMI_BAUDRATE 115200  //串口屏幕波特率
#define QR_BAUDRATE 9600        //串口扫码模块波特率 默认波特率9600
// device settings.
#define TJCHMI_RX PB15
#define TJCHMI_TX PB14

#define QR_RX PE0
#define QR_TX PE1

//                              RX          TX
HardwareSerial Serial_TJCHMI(TJCHMI_RX, TJCHMI_TX);
//         串口扫码模块     RX     TX
HardwareSerial Serial_QR(QR_RX, QR_TX);
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
bool firstFlag=false;
int dataIndex = 0;  // 数据索引



void setup() {
      //串口屏串口初始化
  Serial_TJCHMI.begin(TJCHMI_BAUDRATE);

  /*--------------硬件串口初始化----------------*/
  Serial_QR.begin(QR_BAUDRATE);
  //等待屏幕和扫码模块启动完成
  delay(200);
  //串口屏串口清除缓存
  //因为串口屏开机会发送88 ff ff ff,所以要清空串口缓冲区
  while (Serial_TJCHMI.read() >= 0)    ;                                            //清空串口缓冲区
  Serial_TJCHMI.print("page main\xff\xff\xff");  //发送命令让屏幕跳转到main页面
  nowtime = millis();                            //获取当前已经运行的时间
}

void loop() {
  //如果未曾扫码成功则接收串口扫码值,只扫一次
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


if(scanFlag)
{
  Serial_TJCHMI.print("t1.txt=\"QROK\"\xff\xff\xff");
      //刷新屏幕显示
    char strTemp[20];
    sprintf(strTemp, "t3.txt=\"%s\"\xff\xff\xff", receivedData);
    //把字符串发送出去
    Serial_TJCHMI.print(strTemp);
}

else{
    if(!firstFlag){
  Serial_TJCHMI.print("t3.txt=\"111+111\"\xff\xff\xff");
  firstFlag=true;
  }
}


   // a++;
 }

