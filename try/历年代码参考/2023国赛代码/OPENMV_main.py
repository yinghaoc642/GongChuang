import sensor, image, time, json, pyb, ustruct, display
from pyb import Pin, Timer, LED, UART

uart = UART(3, 115200)
clock = time.clock()
# 定义指令
MODE_COLOR = 67
MODE_ERROR = 69
MODE_IDLE = 70


#led&lcd
light = Timer(2, freq=50000).channel(1, Timer.PWM, pin=Pin("P6"))
#led = pyb.LED(1) # Red LED = 1, Green LED = 2, Blue LED = 3, IR LEDs = 4.

# 初始化摄像头
sensor.reset()                      # Reset and initialize the sensor.
sensor.set_pixformat(sensor.RGB565) # Set pixel format to RGB565 (or GRAYSCALE)
sensor.set_framesize(sensor.QQVGA)#QQVGA？？   # Set frame size to QVGA (320x240)
sensor.skip_frames(time = 2000)     # Wait for settings take effect.
sensor.set_auto_gain(False)         # 关闭自动增益
sensor.set_auto_whitebal(False)     # 关闭自动白平衡


# 设置要识别的颜色范围(R_min, R_max, G_min, G_max, B_min, B_max)
#color_thresholds = [
#    (0, 83, 18, 127, -2, 92),  # 红色阈值
#    (4, 59, -54, -18, -1, 41),     # 蓝色阈值
#    (7, 21, 7, 27, -53, -5)     # 绿色阈值
#]

color_thresholds = [
    (22, 60, 22, 85, -30, 70),  # 红色阈值
    (0, 88, -64, -36, -76, 69),     # lv色阈值
    (22, 64, -35, 25, -76, -25)     # 蓝 导带底 色阈值
]
dec_color = 0
byte_color = dec_color.to_bytes(1, 'big')
x=0
y=0
command = MODE_ERROR

# 等待接收数据帧
def monitor_frame():
    if uart.any():
        data = list(uart.read(3))
    else:
        data = []
    return data


# 解析数据帧
def parse_frame(data):
    print(data)
    if len(data) == 3 and data[0] == 0xAA and data[2] == 0xBB:
        return data[1]
    else:
        return None

def Circle_finding():
    light.pulse_width_percent(50) # 控制亮度 0~100
    img_r = sensor.snapshot().lens_corr(1.8)

    for c in img_r.find_circles(threshold =1500, x_margin = 10, y_margin = 10, r_margin = 10,
            r_min = 20, r_max = 40, r_step = 2):
        img_r.draw_circle(c.x(), c.y(), c.r(), color = (255, 0, 0))
#        print(c)

        uart.write(bytes('['.encode()))
        uart.write(str(c.x()).encode())
        uart.write(bytes(','.encode()))
        uart.write(str(c.y()).encode())
        uart.write(bytes(']'.encode()))


def color_identify():
    global dec_color
    light.pulse_width_percent(
                              0) # 控制亮度 0~100

    cx1=0
    cx2=0
    cy1=0
    cy2=0
    dec_color1=0
    dec_color2=0

    img = sensor.snapshot() # 拍摄图像
    detected_colors1 = []  # 存储检测到的颜色 返回值1R 2G 3B
    for idx1, threshold1 in enumerate(color_thresholds):
        blobs1 = img.find_blobs([threshold1], pixels_threshold=200, area_threshold=200, merge=True)

        if blobs1:
            detected_colors1.append(idx1 + 1)  # 将检测到的颜色加入列表
            for blob1 in blobs1:
                # 在图像上绘制矩形和交叉线
                img.draw_rectangle(blob1.rect())
                img.draw_cross(blob1.cx(), blob1.cy())
                print(f"first 颜色 {idx1 + 1} 中心坐标 (x, y):", blob1.cx(), blob1.cy())
                #print("检测到的颜色:", detected_colors1)  # 返回值1R 2G 3B
                cx1=blob1.cx()
                cy1=blob1.cy()
                dec_color1=idx1+1


    time.sleep_ms(300)

    img = sensor.snapshot() # 拍摄图像
    detected_colors2 = []  # 存储检测到的颜色 返回值1R 2G 3B
    for idx2, threshold2 in enumerate(color_thresholds):
        blobs2 = img.find_blobs([threshold2], pixels_threshold=200, area_threshold=200, merge=True)

        if blobs2:
            detected_colors2.append(idx2 + 1)  # 将检测到的颜色加入列表
            for blob2 in blobs2:
                # 在图像上绘制矩形和交叉线
                img.draw_rectangle(blob2.rect())
                img.draw_cross(blob2.cx(), blob2.cy())
                print(f"second颜色 {idx2 + 1} 中心坐标 (x, y):", blob2.cx(), blob2.cy())
                #print("检测到的颜色:", detected_colors2)  # 返回值1R 2G 3B
                cx2=blob2.cx()
                cy2=blob2.cy()
                dec_color2=idx2+1

    if(-3<(cx1-cx2)<3 and -3<(cy1-cy2)<3 and dec_color1 == dec_color2):
        if(dec_color1==0):
            dec_color = 0
            print("没检测到颜色",dec_color)
            uart.write(bytes('{'.encode()))
            uart.write(str(dec_color).encode())
            uart.write(bytes('}'.encode()))
        else:
            dec_color = dec_color1
#                byte_color = dec_color.to_bytes(1, byteorder='big')
            print("静止 颜色为：", dec_color)
            uart.write(bytes('{'.encode()))
            uart.write(str(dec_color).encode())
            uart.write(bytes('}'.encode()))
    else:
        dec_color = 0
        print("运动", dec_color)
        uart.write(bytes('{'.encode()))
        uart.write(str(dec_color).encode())
        uart.write(bytes('}'.encode()))

# 主循环
while True:

    clock.tick()
    # 等待接收数据帧
    received_data = monitor_frame()
#    print("len(received_data) = {}".format(len(received_data)))
    # 解析接收到的数据帧

    if len(received_data)==3:
        temp = parse_frame(received_data)
        if temp != None:
            command = temp
    else:
        pass

        # 根据指令执行相应的操作
    print("状态command = {}".format(command))
    if command == MODE_COLOR:
        color_identify()
#        print("颜色检测模式")
        # 模拟发送数据帧
#            send_data =bytearray([0xaa,b'C',1,0,0,0,0xbb])
#            uart.write(send_data)

    elif command == MODE_ERROR:
#        print("误差检测模式")
        Circle_finding()
#            send_data =bytearray([0xaa,b'E',1,1,1,1,0xbb])
#            uart.write(send_data)

    elif command == MODE_IDLE:
        print("空闲模式")


    else:
        print("状态不变,command = {}".format(command))
        pass

    # 延迟一段时间
#    print("FPS %f" % clock.fps())
