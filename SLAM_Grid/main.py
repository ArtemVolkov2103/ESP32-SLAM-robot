import math

import serial
from serial.tools import list_ports
#print(list_ports.comports()[0][0])
# Find and open the COM port
ports = serial.tools.list_ports.comports()
port = next((p.device for p in ports), None)
#ser.close()
import re
from random import randint
import numpy as np
import cv2
import time
import threading

nums = [0, 0] #с ардуино
maxLenght = 150
scale = 0.7
step_count = 200
map = [0] * step_count
for i in range(step_count):
    map[i] = [0] * step_count
height = 1200
width = 1200
cv2.namedWindow('image')
image = np.zeros((height, width, 3), np.uint8)

def drawMesh():
    y_start = 0
    y_end = height
    step_size = int(height / step_count)
    for x in range(0, height, step_size):
        cv2.line(image, (x, y_start), (x, y_end), (50, 50, 50), 1)
    x_start = 0
    x_end = width
    for y in range(0, width, step_size):
        cv2.line(image, (x_start, y), (x_end, y), (100, 100, 100), 1)
    cv2.line(image, (int(0), int(height/2)), (int(width/2), int(height/2)), (100, 2550, 100), 1)
    cv2.imshow('image', image)


def draw(x, y, x1, y1):
    xx = int(height / step_count * x + height / step_count - 1)
    yy = int(width / step_count * y + width / step_count - 1)
    x1x = int(height / step_count * x1 + height / step_count - 1)
    y1y = int(width / step_count * y1 + width / step_count - 1)
    map[x][y] += 40
    map[x1][y1] += 40
    cv2.rectangle(image, (int(height / step_count * x + 1), int(width / step_count * y + 1)),
                  (xx, yy), (50 + map[x][y], 50 + map[x][y], 50 + map[x][y]), -12)
    cv2.rectangle(image, (int(height / step_count * x1 + 1), int(width / step_count * y1 + 1)),
                  (x1x, y1y), (50 + map[x1][y1], 50 + map[x1][y1], 50 + map[x1][y1]), -12)
    #cv2.circle(image, (int(height / step_count * x), int(width / step_count * y)), 5, (0,0,255), -1)

def draw2(x, y):
    xx = int(height / step_count * x + height / step_count - 1)
    yy = int(width / step_count * y + width / step_count - 1)
    map[x][y] += 40
    cv2.rectangle(image, (int(height / step_count * x + 1), int(width / step_count * y + 1)),
                  (xx, yy), (50 + map[x][y], 50 + map[x][y], 50 + map[x][y]), -12)

def getSerialData():
    ser = serial.Serial(list_ports.comports()[0][0], 115200, timeout=1)
    if ser.isOpen():
        while True:
            while ser.inWaiting() > 0:
                data = ser.readline()
                nums = re.findall(r'\d+', data.decode('utf-8'))
                nums = [int(i) for i in nums]
                print(nums)
                if len(nums) < 4:
                    continue
                nums[0] = nums[0]*scale
                if len(nums) > 3 and maxLenght > nums[0] and maxLenght > nums[2]:
                    draw2(int(math.cos((nums[1] + 180) * 0.017) * nums[0] + 99),
                         int(math.sin((nums[1] + 180) * 0.017) * nums[0] + 99))
                    # draw(nums[0]+50, nums[1]+50, 0, 0)
                    '''draw(int(math.cos((nums[1] + 180) * 0.017) * nums[0] + 99),
                         int(math.sin((nums[1] + 180) * 0.017) * nums[0] + 99),
                         int(math.cos((nums[3] + 180) * 0.017) * nums[2] + 99),
                         int(math.sin((nums[3] + 180) * 0.017) * nums[2] + 99))'''

    else:
        print("Serial port error:")
        ser.close()

if __name__ == '__main__':
    drawMesh()
    ser = serial.Serial(list_ports.comports()[0][0], 115200, timeout=1)

    while True:
        cv2.imshow('image', image)
        cv2.waitKey(1)
        if ser.inWaiting() > 0:

            data = ser.readline()
            nums = re.findall(r'\d+', data.decode('utf-8'))
            nums = [int(i) for i in nums]
            print(nums)

            nums[0] = nums[0]*scale
            if len(nums) > 1 and maxLenght > nums[0]:
                draw2(int(math.cos((nums[1]) * 0.017) * nums[0] + 99),
                      int(math.sin((nums[1]) * 0.017) * nums[0] + 99))
                time.sleep(0.02)


    cv2.destroyAllWindows()

    #thread = threading.Thread(target=drawMesh())
    #drawMesh()
    #thread2 = threading.Thread(target=getSerialData())
    #thread.start()
    #thread2.start()
    


    '''
    try:
        if port is None:
            raise ValueError("No COM port found.")
        ser = serial.Serial(list_ports.comports()[0][0], 115200)
        while True:
            while ser.inWaiting() > 0:
                data = ser.readline()
                nums = re.findall(r'\d+', data.decode('utf-8'))
                nums = [int(i) for i in nums]
                print(nums)
                if len(nums) < 4:
                    continue
                #nums[0] = nums[0]*scale
                if len(nums) > 3 and maxLenght > nums[0] and maxLenght > nums[2]:
                    #draw(nums[0]+50, nums[1]+50, 0, 0)
                    draw(int(math.cos((nums[1]+180)*0.017) * nums[0] + 99), int(math.sin((nums[1]+180)*0.017) * nums[0] + 99),
                         int(math.cos((nums[3]+180)*0.017) * nums[2] + 99), int(math.sin((nums[3]+180)*0.017) * nums[2] + 99))


    except serial.SerialException as se:
        print("Serial port error:", str(se))
    except KeyboardInterrupt:
        pass
    finally:
        # Close the serial connection
        if ser.is_open:
            ser.close()
            print("Serial connection closed.")
        #x = randint(0, step_count-1)
        #y = randint(0, step_count-1)
    '''


