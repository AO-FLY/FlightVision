import image, math, pyb, sensor, struct, time, ustruct
from pyb import UART, LED, Timer, Pin, Servo
import json

output_str = "0"
output_str_fire1=bytearray([0x42, 0x42])
output_str_fire2="0"
output_str_fire3="0"
output_str_fire4=bytearray([0x00, 0x00])
fire_threshold    = (26, 52, 22, 77, -6, 54)

Find_it=0

##############PWM输出#############
#Tim = Timer(2, freq=1) #设置频率，初始化定时器2，将其设置为1HZ，也就是说一个PWM周期为1s
s1 = Servo(1) # 舵机
P6 = pyb.Pin("P6", pyb.Pin.OUT_PP) #绿色激光
P8 = pyb.Pin("P8", pyb.Pin.OUT_PP) #警示灯

#################################

sensor.reset()
sensor.set_pixformat(sensor.RGB565) # 像素格式
sensor.set_framesize(sensor.QVGA) # 帧大小
sensor.set_auto_exposure(False, 10000) # 设置曝光
#sensor.set_windowing((0,0,240,160))#观察窗口
sensor.set_windowing((60,20,120,120))#观察窗口(120,60,120,120)
sensor.skip_frames(10)
sensor.set_auto_whitebal(False) # 关闭自动白平衡
clock = time.clock()

OX = 0
OY = 0
#K=1000#the value should be measufire

########串口接收数据函数处理#########
uart = pyb.UART(3, 115200, timeout_char = 1000)     #定义串口3变量 P4：TX P5：RX

Find_Task = 0
Target_Num = 0
blue_led  = LED(3)
data = [0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00]

def UartReceiveDate():  #这个函数不能运行太快，否则会导致串口读取太快导致出错
    global Find_Task
    global Target_Num

    global data
    data[0] = uart.readchar() #读取一个字符，并返回其整数形式
    data[1] = uart.readchar()
    data[2] = uart.readchar()
    data[3] = uart.readchar()

    if data[0] == 66 and data[1] == 66:
        Find_Task =  data[2]
        Target_Num = data[3]
        Find_Task =  Find_Task
        Target_Num = Target_Num
        print(Find_Task, Target_Num)

    blue_led.toggle
    #print(Find_Task,Target_Num)
###################################

def find_max(blobs):
    max_size=0
    for blob in blobs:
        if blob.pixels() > max_size:
            max_blob=blob
            max_size = blob.pixels()
    return max_blob

def detect(max_blob):#输入的是寻找到色块中的最大色块
    #print(max_blob.solidity(),max_blob.density())
    shape=0
    if max_blob.solidity()>0.8 or max_blob.density()>0.75:
        #img.find_rects(threshold = 10000)：
        img.draw_rectangle(max_blob.rect(),color = (255, 255, 255))
        shape=1#表示方形

    elif max_blob.density()>0.55:
        img.draw_circle((max_blob.cx(), max_blob.cy(),int((max_blob.w()+max_blob.h())/4)))
        shape=2#表示圆形

    elif max_blob.density()>0.30:
        img.draw_rectangle(max_blob.rect(),color = (0, 0, 0))
        shape=3#表示三角形

    return shape

def Find_Fire():
    blobs_fire = img.find_blobs([fire_threshold])
    #blobs_blue = img.find_blobs([blue_threshold])

    if blobs_fire:  #识别火源
       max_blob_fire=find_max(blobs_fire)
       shape_fire=detect(max_blob_fire)
       #img.draw_rectangle(max_blob_fire.rect(),color=(255,0,0))
       img.draw_cross(max_blob_fire.cx(), max_blob_fire.cy(),color=(255,0,0))
       OX = 70-max_blob_fire.cx()
       OY = 70-max_blob_fire.cy()
       real_x = OX * 40 / 70
       real_y = OY * 40 / 70
       real_x = "%.2f" % real_x
       real_y = "%.2f" % real_y
       real_x = float(real_x)
       real_y = float(real_y)

       output_str_fire2 = float_to_hex_bytes(real_x)
       output_str_fire3 = float_to_hex_bytes(real_y)
       if real_x > 2 or real_y > 2:
           uart.write(output_str_fire1 + output_str_fire2 + output_str_fire3 + output_str_fire4 + b'\r\n')#方式3 十六进制浮点
           print(output_str_fire1 + output_str_fire2 + output_str_fire3 + output_str_fire4)
       return 1
    else:
       shape_fire = None
       return 0

def float_to_hex_bytes(value):
    hex_bytes = bytearray(ustruct.pack('<f', value))
    return hex_bytes

for i in range(100): #舵机夹紧
           s1.pulse_width(0)
           time.sleep_ms(10)

while(True):
    clock.tick() # 追踪两个snapshots()之间经过的毫秒数.
    img = sensor.snapshot().lens_corr(0.8)#lens_corr(1.8)畸变矫正 (拍一张照片并返回图像)
    UartReceiveDate()
    #img.draw_cross(70, 70, size=5, color=(0,255,0))
    if Find_Task == 0: #简单复位
       P6.low()  #打开对地激光
       P8.high() #关闭警示灯
       Find_Task = 2

    if Find_Task == 2:
       Find_it = Find_Fire()#寻找对应颜色并识别出相对飞机的位置和形状
       if Find_it == 1:
          P8.low() #打开警示灯
          output_str_fire4=bytearray([0x01, 0x00])

    if Find_Task == 1:
        P8.high() #关闭警示灯
        for i in range(100): #舵机松开
           s1.pulse_width(2000)
           time.sleep_ms(10)



