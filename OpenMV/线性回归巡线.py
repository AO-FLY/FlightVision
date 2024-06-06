THRESHOLD = (2, 49, -128, -2, -31, 28) # Grayscale threshold for dark things...
import sensor, image, time
from pyb import LED
from pyb import UART


uart = UART(3, 115200)
uart.init(115200, bits=8, parity=None, stop=1)  #8位数据位，无校验位，1位停止位

LED(1).on()
LED(2).on()
LED(3).on()

sensor.reset()
sensor.set_vflip(True)
sensor.set_hmirror(True)
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QQVGA) # 80x60 (4,800 pixels) - O(N^2) max = 2,3040,000.
#sensor.set_windowing([0,20,80,40])
sensor.skip_frames(time = 2000)     # WARNING: If you use QQVGA it may take seconds
clock = time.clock()                # to process a frame sometimes.

while(True):
    clock.tick()
    img = sensor.snapshot().binary([THRESHOLD])
    line = img.get_regression([(100,100)], robust = True)
    if (line):
        rho =(int)(abs(line.rho())-img.width()/2)
        if line.theta()>90:
            theta = line.theta()-180
        else:
            theta = line.theta()
        img.draw_line(line.line(), color = 127)
       # print(rho_err,line.magnitude(),rho_err)

        data = bytearray([0xb3,0xb3,rho,theta,0x5b])
        uart.write(data)    #打印rho和偏移坐标
        print("rho:",rho)      #回归线的偏移距离   一般用偏移距离较好
        print("theta:",theta)  #回归线的偏移角度



