from maix import nn
from PIL import Image, ImageDraw, ImageFont
from maix import  display, camera
import time
from maix.nn import decoder
import serial

ser = serial.Serial("/dev/ttyS1",115200)    # 连接串口

def draw_rectangle_with_title(draw, box, disp_str, bg_color=(255, 0, 0, 255), font_color=(255, 255, 255, 255)):
    # draw = ImageDraw.Draw(img)
    font = ImageFont.load_default()
    font_w, font_h = font.getsize(disp_str)
    draw.rectangle((box[0], box[1], box[0] + box[2], box[1] + box[3]), fill=None, outline=bg_color, width=2)
    draw.rectangle((box[0], box[1] - font_h, box[0] + font_w, box[1]), fill=bg_color)
    draw.text((box[0], box[1] - font_h), disp_str, fill=font_color, font=font)
    
camera.config(size=(224, 224))
labels = ['circle', 'circle_blue', 'rectangle', 'rectangle_blue', 'start', 'triangle', 'triangle_blue']
anchors = [3.78, 3.78, 1.78, 1.78, 4.97, 4.97, 0.97, 0.97, 2.72, 2.72]
model = {
    "param": "/root/app/model-111.awnn.param",
    "bin": "/root/app/model-111.awnn.bin"
}
options = {
    "model_type":  "awnn",
    "inputs": {
        "input0": (224, 224, 3)
    },
    "outputs": {
        "output0": (7, 7, (1+4+len(labels))*5)
    },
    "mean": [127.5, 127.5, 127.5],
    "norm": [0.0078125, 0.0078125, 0.0078125],
}
print("-- load model:", model)
m = nn.load(model, opt=options)
print("-- load ok")
w = options["inputs"]["input0"][1]
h = options["inputs"]["input0"][0]
yolo2_decoder = decoder.Yolo2(len(labels), anchors, net_in_size=(w, h), net_out_size=(7, 7))
while 1:
    img = camera.capture()
    if not img:
        time.sleep(0.01)
        continue
    out = m.forward(img, quantize=True, layout="hwc")
    boxes, probs = yolo2_decoder.run(out, nms=0.3, threshold=0.5, img_size=(240, 240))
    for i, box in enumerate(boxes):
        class_id = probs[i][0]
        prob = probs[i][1][class_id]
        msg = "{}:{:.2f}%".format(labels[class_id], prob*100)
        img.draw_rectangle(box[0], box[1], box[0] + box[2], box[1] + box[3], color=(255, 255, 255), thickness=2)
        img.draw_string(box[0] + 2, box[1] + 2, msg, scale = 1.2, color = (255, 255, 255), thickness = 2)
        ser.write(b"{x:%d, y:%d}\n" % (box[0] + box[2]/2, box[1] + box[3]/2))    #写
    display.show(img)