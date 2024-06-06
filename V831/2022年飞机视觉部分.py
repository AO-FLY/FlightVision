# 1.可识别目标点的形状三种（圆、正方形、三角形）
# 2.颜色有红蓝两种（红--R-255、G-0、B-0   蓝--R-0、G-0、B-255）
# 3.降吊舱后，播放收货语音(改stm32播放)
# 4.在作业区域外学习识别某一指定目标特征，然后寻找到具有此特征的两个目标点


# 库引入
from maix import camera, display, nn, mjpg, utils
from maix.nn import decoder
import time
from evdev import InputDevice
from select import select
import serial

# 0->红色圆形  1->蓝色圆形  2->红色矩形  3->蓝色矩形  4->起点位置  5->红色三角形  6->蓝色三角形
target = [0, 1, 2, 3, 4, 5, 6]
# 坐标：5、6  7、8  9、10  11、12  0  1、2  3、4 (对应标识)
coordinate = [[350, 275], [275, 50], [125, 50], [125, 200], [50, 125], [200, -25], [200, 275], [350, 125], [0, 0],
              [50, 275], [200, 125], [275, 200], [350, -25]]
# 标识
symbol = [5, 6, 7, 8, 9, 10, 11, 12, 0, 1, 2, 3, 4]

model = "/root/self_learn/model-111.awnn.mud"

labels = ['circle', 'circle_blue', 'rectangle', 'rectangle_blue', 'start', 'triangle', 'triangle_blue']

anchors = [3.78, 3.78, 1.78, 1.78, 4.97, 4.97, 0.97, 0.97, 2.72, 2.72]

input_size = (224, 224)


# 目标识别检测类
class YOLOv2:
    def __init__(self, model_path, labels, anchors, net_in_size, net_out_size):
        self.labels = labels
        self.anchors = anchors
        self.model = nn.load(model_path)
        self._decoder = nn.decoder.Yolo2(len(labels), anchors, net_in_size=net_in_size, net_out_size=net_out_size)

    def run(self, img, nms=0.3, threshold=0.5):
        out = self.model.forward(img, layout="hwc")
        boxes, probs = self._decoder.run(out, nms=nms, threshold=threshold, img_size=input_size)
        return boxes, probs
           
    # 对目标画框
    def draw(self, img, boxes, probs, buf):
        for i, box in enumerate(boxes):
            if probs[i][0] == buf:
                class_id = probs[i][0]
                prob = probs[i][1][class_id]
                msg = "{}:{:.2f}%".format(self.labels[class_id], prob * 100)
                img.draw_rectangle(box[0], box[1], box[0]+box[2], box[1]+box[3], (255,255,255), 2)
                img.draw_string(box[0], box[1], msg, scale=1.2, color=(255, 255, 255), thickness=2)
                return box


# 按键和音频类
class funation:
    position_x = 0  # x坐标
    position_y = 0  # y坐标
    position_w = 0  # w坐标
    position_h = 0  # h坐标
    flag=0
    fun_status = 0
    chunk = 1024

    # 初始化
    def __init__(self, device=None):
        self.device = device
        self.keys = InputDevice('/dev/input/event0')
        self.uart = serial.Serial("/dev/ttyS1", 115200,timeout=0,write_timeout=0.05)
        self.queue = mjpg.Queue(maxsize=8)

    # 按键获取
    def get_key(self):
        r, w, x = select([self.keys], [], [], 0)
        if r:
            for event in self.keys.read():
                if event.code == 0x02:  # 右键
                    return 1
                if event.code == 0x03:  # 左键
                    return 2

    # 串口数据打包 （bytearray）
    def pack_data(self, position, flag, num):

        self.flag = flag
        # 物块坐标分解
        self.position_x = position[0]
        self.position_y = position[1]
        self.position_w = position[2]
        self.position_h = position[3]
        # 数据格式：头帧加尾帧  num--1/0   是否识别  x/y--距离中心位移（与实际不匹配）   flag--识别到的标识位
        Data = bytearray([0xAA, num, self.position_x, self.position_y,self.position_w,self.position_h,self.flag,0xFF])
        return Data

    # 串口发送数据
    def uart_send(self,position, flag, num):
        self.uart.write(self.pack_data(position, flag, num))

    # 图传 http://127.0.0.1:18811 未使用
    def transmission(self, img):
        mjpg.MjpgServerThread("0.0.0.0", 18811, mjpg.BytesImageHandlerFactory(q=self.queue)).start()  # 图传
        try:
            jpg = utils.rgb2jpg(img.convert("RGB").tobytes(), img.width, img.height)
            self.queue.put(mjpg.BytesImage(jpg))
        except Exception as e:
            print("transmission error")


# 主程序
def main():
    start = funation()
    camera.config(size=input_size)
    yolov2 = YOLOv2(model, labels, anchors, input_size, (input_size[0] // 32, input_size[1] // 32))
    buf = 0

    while True:
        img = camera.capture()
        key = start.get_key()

        boxes, probs = yolov2.run(img, nms=0.3, threshold=0.5)            
        for i, box in enumerate(boxes):
            class_id = probs[i][0]
            prob = probs[i][1][class_id]
            msg = "{}:{:.2f}%".format(yolov2.labels[class_id], prob * 100)
            img.draw_rectangle(box[0], box[1], box[0]+box[2], box[1]+box[3], (255,255,255), 2)
            img.draw_string(box[0], box[1], msg, scale=1.2, color=(255, 255, 255), thickness=2)
            start.uart_send(box, class_id, 1)  # 返回物体相对坐标，补偿精准定位

            
        if key == 1:  # 按下右键进入第二关
            t = 0
            m = 24
            while t <15:  # 一开始识别需要检测什么物体
                img = camera.capture()
                boxes, probs = yolov2.run(img)                
                if len(boxes):
                    for i, box in enumerate(boxes):  
                        class_id = probs[i][0]
                        if class_id < m:  # 主要是用来区分方形和三角
                            m = class_id
                        prob = probs[i][1][class_id]
                        msg = "{}:{:.2f}%".format(yolov2.labels[class_id], prob * 100)
                        img.draw_rectangle(box[0], box[1], box[0]+box[2], box[1]+box[3], (255, 255, 255), 2)
                        img.draw_string(box[0], box[1], msg, scale=1.2, color=(255, 255, 255), thickness=2)
                        img.draw_string(0, 200, "checking num:" + str(m), scale=1.2, color=(0, 255, 0), thickness=2)  # 显示选择图像的数值
                        display.show(img)
                    start.uart.write(class_id)  # 给32发送命令
                t += 1
            buf = m
            if buf == 24:  # 如果没有识别到
                buf = "None"
            while True:  # 开始物体检测
                img = camera.capture()
                key = start.get_key()
                boxes, probs = yolov2.run(img)
                position=yolov2.draw(img, boxes, probs, buf)
                home = yolov2.draw(img, boxes, probs, 4)
                img.draw_string(0, 200, "num:" + str(buf), scale=1.2, color=(0, 255, 0), thickness=2)  # 显示选择图像的数值
                if position:
                    start.uart_send(position, m, 1)  # 返回物体相对坐标，补偿精准定位
                if home:
                    start.uart_send(home, 4, 1)  # 返回home的坐标
                display.show(img)
                if key == 2:
                    buf = []  # 退出时清除buf的值
                    break

        display.show(img)

main()
