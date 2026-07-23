
/*****************************************************************************
Copyright: 2024,Sunnybot.
File name: ESP32Car_STU.ino
Description: wifi遥控小车
Author: Sunny
Version: 1.0
Date: 20240903
History: 修改历史记录列表， 每条修改记录应包括修改日期、修改者及修改内容简述。
20220301 Sunny 规范注释
20240612 Sunny ESP32环境3.0.1
*****************************************************************************/

//******************预编译包含使用库的头文件***************************//
#include <WiFi.h>               //WiFi库
#include <ArduinoWebsockets.h>  //Websocket库 双向通信协议
#include <ESPAsyncWebServer.h>  //异步服务器库https://github.com/me-no-dev/ESPAsyncWebServer ESPAsyncWebServer.h
#include <DDSM210.h>
#include "config.h"       //服务器实例配置
#include "web.h"          //网页文件

//******************WiFi热点配置***************************//
const char* ssid = "ESP32_Web";     // WiFi热点名称，请修改为"ESP32_学号"
const char* password = "12345678";  // WiFi密码
//启动后使用组内一台手机连接以下名称的Wifi
//连接后使用浏览器打开192.168.4.1
//使用网页摇杆进行控制
//******************串口电机设置***************************//
DDSM_CTRL dc;

// device settings.
#define M0603C_RX 19
#define M0603C_TX 23


//******************变量***************************//
int M1Speed, M2Speed;
int FBValue, LRValue, commaIndex;

/****************************电机驱动函数*********************************************
函数功能：驱动两路电机运动
入口参数：motora a电机驱动PWM[-210,210]；motorb  b电机驱动PWM[-210,210]
**************************************************************************/
void RunMotors(int motora, int motorb) {
 // a电机驱动PWM[-2100,2100]；b电机驱动PWM[-2100,2100]
dc.ddsm210_ctrl_2(motora*10,-motorb*10);
}


//******************处理http消息并控制电机 **********************//

void handle_message(WebsocketsMessage msg) {
  commaIndex = msg.data().indexOf(',');
  LRValue = msg.data().substring(0, commaIndex).toInt();  //获得摇杆左右方向返回值
  FBValue = msg.data().substring(commaIndex + 1).toInt();  //获得摇杆上下方向返回值

  //控制器 轮速PWM和摇杆值映射
  //参考映射规则 ：摇杆前后FBValue映射刚体Vx（油门），LRValue映射为刚体角速度w（方向）
  //摇杆左右LRValue代表映射w，该方向与小车坐标系相反故取反
  //对小车进行运动学分析并结合现场调试结果设置合理的参数。

  if((LRValue==100||LRValue==-100)&&(FBValue<=10&&FBValue>=-10))
  {  
    M1Speed = (FBValue * 0 + LRValue * 2);
    M2Speed = (FBValue * 0 - LRValue * 2);
  }
  else
  {
    M1Speed = (FBValue * 1.8 + LRValue * 0.4);
    M2Speed = (FBValue * 1.8 - LRValue * 0.4);
}
  //驱动车轮转动
  RunMotors(M1Speed, M2Speed);
  
}

void setup() {


  //串口初始化
  Serial.begin(115200);

  // 建立WiFi热点 Create AP
  WiFi.softAP(ssid, password);
  //打印热点IP
  IPAddress IP = WiFi.softAPIP();
  Serial.print("AP IP address: ");
  Serial.println(IP);

  //电机串口初始化
	// M0603C init.
	Serial1.begin(DDSM_BAUDRATE, SERIAL_8N1, M0603C_RX, M0603C_TX);
	dc.pSerial = &Serial1;

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
  // HTTP handler assignment 方法注册链接与回调函数
  webserver.on("/", HTTP_GET, [](AsyncWebServerRequest* request) {
    AsyncWebServerResponse* response = request->beginResponse_P(
        200, "text/html", index_html_gz, sizeof(index_html_gz));
    response->addHeader("Content-Encoding", "gzip");
    request->send(response);
  });

  // 启动服务start server
  webserver.begin();
  server.listen(82);
  Serial.print("Is server live? ");
  Serial.println(server.available());

}

void loop() {
  //接受客户端连接
  auto client = server.accept();
  //等待消息
  client.onMessage(handle_message);

  while (client.available()) {
    client.poll();
  }
}