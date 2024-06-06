# 1.��ʶ��Ŀ������״���֣�Բ�������Ρ������Σ�
# 2.��ɫ�к������֣���--R-255��G-0��B-0   ��--R-0��G-0��B-255��
# 3.�����պ󣬲����ջ�����(��stm32����)
# 4.����ҵ������ѧϰʶ��ĳһָ��Ŀ��������Ȼ��Ѱ�ҵ����д�����������Ŀ���


# ������
from maix import camera, display, nn, mjpg, utils
from maix.nn import decoder
import time
from evdev import InputDevice
from select import select
import serial

# 0->��ɫԲ��  1->��ɫԲ��  2->��ɫ����  3->��ɫ����  4->���λ��  5->��ɫ������  6->��ɫ������
target = [0, 1, 2, 3, 4, 5, 6]
# ���꣺5��6  7��8  9��10  11��12  0  1��2  3��4 (��Ӧ��ʶ)
coordinate = [[350, 275], [275, 50], [125, 50], [125, 200], [50, 125], [200, -25], [200, 275], [350, 125], [0, 0],
              [50, 275], [200, 125], [275, 200], [350, -25]]
# ��ʶ
symbol = [5, 6, 7, 8, 9, 10, 11, 12, 0, 1, 2, 3, 4]

model = "/root/self_learn/model-111.awnn.mud"

labels = ['circle', 'circle_blue', 'rectangle', 'rectangle_blue', 'start', 'triangle', 'triangle_blue']

anchors = [3.78, 3.78, 1.78, 1.78, 4.97, 4.97, 0.97, 0.97, 2.72, 2.72]

input_size = (224, 224)


# Ŀ��ʶ������
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
           
    # ��Ŀ�껭��
    def draw(self, img, boxes, probs, buf):
        for i, box in enumerate(boxes):
            if probs[i][0] == buf:
                class_id = probs[i][0]
                prob = probs[i][1][class_id]
                msg = "{}:{:.2f}%".format(self.labels[class_id], prob * 100)
                img.draw_rectangle(box[0], box[1], box[0]+box[2], box[1]+box[3], (255,255,255), 2)
                img.draw_string(box[0], box[1], msg, scale=1.2, color=(255, 255, 255), thickness=2)
                return box


# ��������Ƶ��
class funation:
    position_x = 0  # x����
    position_y = 0  # y����
    position_w = 0  # w����
    position_h = 0  # h����
    flag=0
    fun_status = 0
    chunk = 1024

    # ��ʼ��
    def __init__(self, device=None):
        self.device = device
        self.keys = InputDevice('/dev/input/event0')
        self.uart = serial.Serial("/dev/ttyS1", 115200,timeout=0,write_timeout=0.05)
        self.queue = mjpg.Queue(maxsize=8)

    # ������ȡ
    def get_key(self):
        r, w, x = select([self.keys], [], [], 0)
        if r:
            for event in self.keys.read():
                if event.code == 0x02:  # �Ҽ�
                    return 1
                if event.code == 0x03:  # ���
                    return 2

    # �������ݴ�� ��bytearray��
    def pack_data(self, position, flag, num):

        self.flag = flag
        # �������ֽ�
        self.position_x = position[0]
        self.position_y = position[1]
        self.position_w = position[2]
        self.position_h = position[3]
        # ���ݸ�ʽ��ͷ֡��β֡  num--1/0   �Ƿ�ʶ��  x/y--��������λ�ƣ���ʵ�ʲ�ƥ�䣩   flag--ʶ�𵽵ı�ʶλ
        Data = bytearray([0xAA, num, self.position_x, self.position_y,self.position_w,self.position_h,self.flag,0xFF])
        return Data

    # ���ڷ�������
    def uart_send(self,position, flag, num):
        self.uart.write(self.pack_data(position, flag, num))

    # ͼ�� http://127.0.0.1:18811 δʹ��
    def transmission(self, img):
        mjpg.MjpgServerThread("0.0.0.0", 18811, mjpg.BytesImageHandlerFactory(q=self.queue)).start()  # ͼ��
        try:
            jpg = utils.rgb2jpg(img.convert("RGB").tobytes(), img.width, img.height)
            self.queue.put(mjpg.BytesImage(jpg))
        except Exception as e:
            print("transmission error")


# ������
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
            start.uart_send(box, class_id, 1)  # ��������������꣬������׼��λ

            
        if key == 1:  # �����Ҽ�����ڶ���
            t = 0
            m = 24
            while t <15:  # һ��ʼʶ����Ҫ���ʲô����
                img = camera.capture()
                boxes, probs = yolov2.run(img)                
                if len(boxes):
                    for i, box in enumerate(boxes):  
                        class_id = probs[i][0]
                        if class_id < m:  # ��Ҫ���������ַ��κ�����
                            m = class_id
                        prob = probs[i][1][class_id]
                        msg = "{}:{:.2f}%".format(yolov2.labels[class_id], prob * 100)
                        img.draw_rectangle(box[0], box[1], box[0]+box[2], box[1]+box[3], (255, 255, 255), 2)
                        img.draw_string(box[0], box[1], msg, scale=1.2, color=(255, 255, 255), thickness=2)
                        img.draw_string(0, 200, "checking num:" + str(m), scale=1.2, color=(0, 255, 0), thickness=2)  # ��ʾѡ��ͼ�����ֵ
                        display.show(img)
                    start.uart.write(class_id)  # ��32��������
                t += 1
            buf = m
            if buf == 24:  # ���û��ʶ��
                buf = "None"
            while True:  # ��ʼ������
                img = camera.capture()
                key = start.get_key()
                boxes, probs = yolov2.run(img)
                position=yolov2.draw(img, boxes, probs, buf)
                home = yolov2.draw(img, boxes, probs, 4)
                img.draw_string(0, 200, "num:" + str(buf), scale=1.2, color=(0, 255, 0), thickness=2)  # ��ʾѡ��ͼ�����ֵ
                if position:
                    start.uart_send(position, m, 1)  # ��������������꣬������׼��λ
                if home:
                    start.uart_send(home, 4, 1)  # ����home������
                display.show(img)
                if key == 2:
                    buf = []  # �˳�ʱ���buf��ֵ
                    break

        display.show(img)

main()
