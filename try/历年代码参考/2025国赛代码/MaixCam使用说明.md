## MaixCam使用之Vision说明
---
**author**          ：   邹子睿 
**version**        ：  v1.0
**date**       ：2025/9/1
**environment**      ：MaixCamPro
**brief**              ：用于国赛MaixCam模块程序使用说明
**tips**:
* 使用该说明文档前，你需要对MaixCam的使用有个大致的了解
* 对涉及到的对象、函数、方法知道该如何使用，在哪查阅
* 该Markdown文件可在vscode中下载插件直接打开
* 针对该项目有任何疑问以及更好的想法直接发送邮箱3220102323@zju.edu.cn

**address**:
* MaixPy：https://wiki.sipeed.com/maixpy/doc/zh/index.html
* API手册：https://wiki.sipeed.com/maixpy/api/
---

### 0.项目结构说明
1.**Examples**：全部例程文件
2.**App_xx**：由Examples自动生成的程序文件
3.**model**：存放Yolov5模型
4.**MaixCam使用说明.md**：本文档
5.**xx.txt**：需要用到的配置文件

### 1.Vision
###### APP_Vision为比赛使用的主程序，支持颜色检测，圆环检测，条形码检测，触摸屏交互调节参数，外设使用等
**主要模式**：
* **CC**：用于转盘上的物料检测与抓取，通过颜色识别，并返回移动量
* **EE**：用于圆环检测（定位），通过以下三种方法，同CC模式下的颜色识别、YOLOv5物体检测、轮廓检测（检测圆环的形状）
* **DD**：为2025国赛决赛中新增模式，用于读取条形码，返回条形码对应的物料颜色，可忽略
* **FF**：待机模式，复位状态

**GUI界面**：对应以上不同模式，设置了不同的GUI界面，用于告诉使用者运行信息等，同时支持按钮点按进行部分参数快速调节，具体见下文

**文件配置**：要使用该程序，需对设备进行文件手动配置，手动创建文件并存于指定目录，具体见每个模式说明中关于文件配置部分



### 2.串口通讯
**硬件资源**：使用UART0串口
**通讯协议**：
* 状态通讯模块,帧头为`b'\xAA'`,帧尾为`b'\xBB'`
* Maix接收：状态指令格式为`b'\xAA\x00\x00\xBB'`,中间为状态与颜色，数据类型为int
* Maix发送：指令格式为`b'\xAA\x00\x00\x00\xBB'`，中间为x方向偏差与y方向偏差，数据类型为int,以及当前返回的颜色

**串口接收**：通过串口中断服务函数
```python
def on_received(serial: uart.UART, data: bytes):
    global Color_Index, Maix_State,i
    i+=1
    if data[0] == 0xAA and data[3] == 0xBB:
        Maix_State = data[1]
        if(Maix_State == 0xCC or Maix_State == 0xEE):
            Color_Index = data[2]
        if(Color_Index not in [1,2,3]):
            Color_Index = 0
        #serial.write_str("received")
    else:
        Maix_State = 0xFF
        Color_Index = 0
        #serial.write_str("error")
    time.sleep_ms(10)
```
最后将该串口中断函数与UART0绑定
```python
Serial_Maix0.set_received_callback(on_received)
```
**串口发送**：用于发送偏差与当前识别的颜色
```python
def Send_data_Grab(serial: uart.UART, Output_x, Output_y, color):
    [x_det, y_det] = [
        Constrain(int(hand_x - Output_x)),
        Constrain(int(Output_y - hand_y))
    ]    
    Send_Data = (
        b"\xaa" + struct.pack("b", x_det) + struct.pack("b", y_det) + struct.pack("b", color) + b"\xbb"
    )
    serial.write(Send_Data)
```

### 3.颜色检测
**核心思路**：
* **均值颜色检测**：使用MaixPy中`find_blobs`方法，返回值为`blob`对象的集合；遍历该集合，将每个`blob`对象的x坐标与y坐标求均值，即认为是当前图像中物料的中心位置。关于上述`find_blobs`与`blob`查阅API手册即可，其中`find_blobs`还有很多参数需要视情况调节，具体不在此赘述。下面的`find_color`是我们自己通过上述思路定义的函数。

```python
def find_color(img, Color_Index, Maix_State, Rmax,Rmin):  # 色块找圆函数
```
* **自适应颜色阈值**：实现思路就是当识别到的颜色像素个数小于我们设定值时，将关键参数调大至一定范围（关键参数说明见下文），不能无穷大。在这里定义了一个“阈值类”`Color_LAB_Threshold`，用该类创建了红绿蓝三个对象，每次识别不到时通过调用`Alter_Threshold(self,Step)`方法改变其属性（这部分相关内容可以自行学习一下Python的面向对象）。
```python
class Color_LAB_Threshold:
    def __init__(self,
                RGB,
                L_Min,
                L_Max,
                A_Min,
                A_Max,
                B_Min,
                B_Max):
        self.RGB = RGB
        self.L_Min = L_Min 
        self.L_Max = L_Max
        self.A_Min = A_Min 
        self.A_Max = A_Max 
        self.B_Min = B_Min 
        self.B_Max = B_Max 
    
    def Alter_Threshold(self,Step):
        if self.RGB == 1:
            if self.A_Min in range(10,100): self.A_Min -= Step
            if self.A_Max in range(10,100): self.A_Max += Step
        if self.RGB == 2:
            if self.A_Min in range(-127,-10): self.A_Min -= Step
            if self.A_Max in range(-127,-10): self.A_Max += Step
        elif self.RGB == 3:
            if self.B_Min in range(-127,0): self.B_Min -= Step
            if self.B_Max in range(-127,-20): self.B_Max += Step
red_lab = Color_LAB_Threshold(1, 0, 100, read_file('/root/lab.txt',1), read_file('/root/lab.txt',2), 0, 128)
green_lab = Color_LAB_Threshold(2, 0, 100, read_file('/root/lab.txt',3), read_file('/root/lab.txt',4), 0, 128)
blue_lab = Color_LAB_Threshold(3, 0, 100, -64, 64, read_file('/root/lab.txt',5), read_file('/root/lab.txt',6))

```
**界面说明**：
**ps**.懒得放图片了，把程序烧进去对照看就好了:sunglasses:
* 进入**CC模式**，可通过点击屏幕上方的CC按钮直接进入
* 上方界面为状态切换栏，通过点击可进入不同模式
* 左侧**RGB**为识别颜色切换栏，通过点击可命令识别不同颜色
* 左右两侧上下"+""-"分别用于调节阈值，调节完成后，点击右侧**Saved**即可保存阈值，下次上电能自动加载本次调节的阈值；点击**Clear**即可恢复默认阈值
* 左上角颜色框表示当前识别的颜色，黑色则为未识别到；当你指定了要识别某种颜色时，右上方FPS刷新处会显示当前要识别的颜色，显示为**Color:1**，下方会显示该颜色当前的关键阈值大小

**调试方法**：
* 在现场比赛中，由于光线的变化，通过IDE烧录程序改变阈值是来不及的。因此使用触摸屏调节关键参数时，只需把物料放在下方，点按调节按钮，直到能正常识别即可
* 关键参数是指主要影响识别准确率的参数，该方面涉及LAB颜色表示方法，只说结论的话，红色和绿色受A值影响大，B值影响小，因此A值需要范围小（精细），B值需要范围大；蓝色反之同理。同时，我们要减少光线的干扰，L值都设置为全范围即可，不需要变动。

**文件配置**：
* 这里的文件配置是指使用一个文本文件，储存在MaixCam本地，用于调试完保存颜色识别的阈值，以及上电后读取颜色识别的阈值。
* 文件名及地址：`/root/lab.txt`,文件名和文件地址可以自己修改，只需要在读取文件以及写入文件的地方对应修改即可。
* 文件内容，最好不要修改。由上至下分别是**红色的Amin，Amax，绿色的Amin，Amax，蓝色的Bmin，Bmax**；至于为什么，与前面的关键参数道理一样。

### 4.圆环检测（定位）
**核心思路**:
* **通过颜色检测定位圆环、通过YOLOv5检测圆环、通过形状（轮廓）检测圆环三级流水线制**，检测的顺序即为上述的顺序，为了节约硬件资源，检测流程为串行，即首先选用方法1，方法1行不通选用方法2，以此类推。使用`Detect_Circles(img, Maix_State)`执行上述流程
```python
def Detect_Circles(img, Maix_State):
    yolo_detected = False # 默认没检测到
    color_detected = False
    contour_detected = False
    global Color_Index
    # 优先使用颜色检测
    Target = find_color(img, Color_Index, Maix_State,1000,0)
    if(Target[3] == Color_Index):
        #img.draw_circle((int)(Target[0]), (int)(Target[1]), (int)(Target[2]), color = image.COLOR_BLUE)
        #img.draw_cross((int)(Target[0]), (int)(Target[1]), color = image.COLOR_BLUE)  # 画找到的圆心
        msg = f'Colors'
        img.draw_string(10, 10, msg, color = image.COLOR_WHITE)
        color_detected = True
    # 若yolo失效
    if(color_detected == False):
        objs = detector.detect(img, conf_th = 0.5, iou_th = 0.45)
        for obj in objs: 
            if(obj.class_id == Color_Index - 1):
                yolo_detected = True
                # 储存当前的观测值
                Target = [(obj.x+obj.w/2),(obj.y+obj.h/2),(int)((obj.w+obj.h)/4),Color_Index,0]
                msg = f'{detector.labels[obj.class_id]}: {obj.score:.2f}'
                img.draw_string(obj.x, obj.y-10, msg, color = image.COLOR_GREEN)
                #img.draw_circle((int)(obj.x+obj.w/2), (int)(obj.y+obj.h/2), (int)((obj.w+obj.h)/4), color = image.COLOR_BLACK)
                #img.draw_cross((int)(obj.x+obj.w/2), (int)(obj.y+obj.h/2), color = image.COLOR_BLACK)  # 画找到的圆心
                msg = f'YOLO'
                img.draw_string(10, 10, msg, color = image.COLOR_WHITE)
            else:
                continue
    # 若yolo与颜色检测都失效
    if(color_detected == False and yolo_detected == False):
        # 轮廓检测
        Target,ellipse = find_Circles(img,Color_Index,True)
        if(Target[3] == Color_Index):
                # 画椭圆并标识圆心
                msg = f'ConTours'
                img.draw_string(10, 10, msg, color = image.COLOR_WHITE)
                img_cv = image.image2cv(img)
                cv2.ellipse(img_cv, ellipse, (0, 255, 0), 1)
                img = image.cv2image(img_cv)
                contour_detected = True
    return Target, (yolo_detected or color_detected or contour_detected), img
```
* **颜色检测**与上述颜色检测思路完全一致，甚至阈值都可以不需要改变，直接拿来用，只是由于圆环的像素数少于物料的像素数，因此需要对部分参数手动修改即可。优点是参数调节方便、计算简单，刷新率可达56FPS，**同时不需要图像中完全出现圆环也可以实现初步的定位**，这个很重要，因为如果运动的不准，可能到达圆环时，只能看到他的一个角，这个时候，也会自动移动过去，与之相反，轮廓识别与YOLOv5都需要一个完整的样本才行。缺点是碰到相近的颜色会干扰，比如黄和绿，如果场地灯光偏暖色就会难以工作
* **YOLOv5**：直接调用模型，返回值与颜色检测相同，单独例程具体可见`\Examples\yolov5.py`。优点是可靠性好，受环境干扰小，缺点是无法调节，同时训练一个模型也不容易，计算量也较大，刷新率在19FPS左右。
* **轮廓检测**：见`find_Circles`函数，为自定义函数，算法原理感兴趣B站上搜，这里也不赘述。它唯一的优点就是，精度最高，同时由于实际检测用的是椭圆检测算法，所以哪怕是有偏差，也能检测出中心，缺点是计算量极大，刷新率只有8FPS，并且为了能够识别指定颜色，需要对图像进行二值化处理，所以颜色检测的缺点他也有。
* 必须要强调的是，之所以不赘述不是因为作者懒，而是因为在实际使用过程中，使用颜色检测就足矣，因为你哪怕检测不出来也是可以通过调节参数修复的，再不济使用YOLOv5也能上最后一道保险，轮廓检测几乎没有用到过。并且轮廓检测虽然比颜色检测要准确的多，但是我们放置物料时的误差主要来源于放置时的抖动、移动的误差以及**视觉模块与手爪的对中误差**，因此轮廓检测多出来的精度可以忽略不计

**界面说明**：
* 进入**EE模式**，可通过点击屏幕上方的EE按钮直接进入
* 上方界面为状态切换栏，通过点击可进入不同模式
* 左侧**RGB**为识别颜色切换栏，通过点击可命令检测不同颜色的圆环

**调试方法**：
* 上文所述主要误差主要来自**视觉模块与手爪的对中误差**，消除该误差需要切换到FF模式，调节屏幕中心黑点与手爪中心对齐
* 见待机模式具体说明

**文件配置**：
* 这里的文件配置是指使用一个文本文件，储存在MaixCam本地，用于调试完保存屏幕实际中心与理论中心的偏差，以及上电后读取该偏差值。以及一个YOLOv5模型文件。
* 文件名及地址：误差文件存放在`/root/error.txt`，YOLOv5模型存放在`/root/models/green_circles/yolov5/best.mud`，文件名和文件地址可以自己修改，只需要在读取文件以及写入文件的地方对应修改即可。注意，YOLOv5文件需要将**整个模型文件包中的文件**都存放进去，虽然这里只涉及`.mud`文件
* 误差文件文件内容，最好不要修改。由上至下分别是
**x=4**
**y=-6**


### 5.待机模式
**界面说明**：
* 进入**FF模式**，可通过点击屏幕上方的FF按钮直接进入,为上机默认模式
* 上方界面为状态切换栏，通过点击可进入不同模式
* 左右两侧上下"+""-"分别用于调节X方向的偏差与Y方向的偏差，调节完成后，点击右侧**Saved**即可保存阈值，下次上电能自动加载本次调节的阈值；点击**Clear**即可全部清零
* 左侧中间为当前屏幕理论中心（手爪中心）与屏幕实际中心在地图上的偏差，单位为mm。注意这里不是像素个数的偏差而是经过换算后的实际偏差
* 左侧按钮`On`为选择LED是否打开的开关，打开后会有一个垂直补光灯

**调试方法**：
* 每次重新安装手爪后正式比赛上场前都可以调试一下，在STM32端调用程序，让机械臂执行连续放置三个物料的动作。如果出现了以下这种情况：三个物料位于一条直线，且与自己圆环的偏差相近，说明该误差出现的原因是屏幕中心与手爪中心没有对中。此时点击左右两边的加减号，可以直接根据偏差大小（mm），在屏幕上修改屏幕中心位置。

### 6.GUI界面
**GUI绘制**：使用自定义函数`draw_UI(img)`，主要绘制的元素有，文字，圆，色块，十字，直线等，使用以上这些元素可以方便地查看设备状态，比如说用文字提示当前运行的状态，用圆圈表示当前定位的物体坐标信息，用直线连接屏幕中心与目标点位等等

**按钮操作**：
* 定义了一个按钮类`button`，用该类去创建一个个对象，包括该按钮对象的名称、大小、位置；该按钮对象使用`click`方法进行点击判断，使用`draw_button`方法在屏幕上绘制并显示出来
* **按钮操作函数**：`Butt_Contrl()`

### 7.文件操作
该部分内容学习自行搜索“Python文件操作”即可
* 文件写操作：`change_file`函数
```python
def change_file(file_path,line_num,ins_Cont):
    # 删除某行
    with open(file_path,'r', encoding='utf-8') as read_file:
            lines = read_file.readlines()
            currentLine = 1
    with open(file_path,'w', encoding='utf-8') as write_file:
            for line in lines:
                if currentLine == line_num:
                    pass
                else:
                    write_file.write(line)
                currentLine += 1
    # 插入某行
    with open(file_path, 'r', encoding='utf-8') as file:
        lines = file.readlines()
    # 在指定行插入内容
        lines.insert(line_num-1, ins_Cont + '\n')
    # 写回文件
    with open(file_path, 'w', encoding='utf-8') as file:
            file.writelines(lines)
def write_file(error_x,error_y,file_path):
    with open(file_path, 'w') as file:
        file.write(f"x={error_x}\n")
        file.write(f"y={error_y}")
```
* 文件读操作：`read_file`函数
```python
def read_file(file_path,line_num):
    content = ''
    with open(file_path, 'r') as file:
        # 执行文件操作，例如读取文件内容
        file_content = file.readlines()
        for i in file_content[line_num-1]:
            if(i == '-' or i.isdigit()):
                content = content + i
        read = (int)(content)
        return read
```















