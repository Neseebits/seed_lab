# import the necessary packages
from picamera.array import PiRGBArray
from picamera import PiCamera
import time
from time import sleep
import cv2 as cv
import matplotlib.pyplot as plt
import numpy as np
from smbus2 import SMBus
import board
import busio
import adafruit_character_lcd.character_lcd_rgb_i2c as character_lcd

target = 0
split = ['','','','']
read = [];
current_angle = 0

lcd_columns = 16 #initialize the LCD display
lcd_rows = 2
i2c = board.I2C()
lcd = character_lcd.Character_LCD_RGB_I2C(i2c, lcd_columns, lcd_rows)

lcd.clear() #clear display for printing
lcd.color = [100, 0, 50] #make lcd a good color

# initialize the camera and grab a reference to the raw camera capture
camera = PiCamera()
camera.resolution = (640, 480)
camera.framerate = 32
rawCapture = PiRGBArray(camera, size=(640, 480))
# allow the camera to warmup
time.sleep(0.1)

# capture frames from the camera
for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
    # grab the raw NumPy array representing the image, then initialize the timestamp
    # and occupied/unoccupied text
    img = frame.array
    # show the frame
    # cv.imshow("Frame", img )

    # Detect yellow/orblue
    hsv = cv.cvtColor(img, cv.COLOR_BGR2HSV)
    # lower_yellow = np.array([20, 100, 100])
    # upper_yellow = np.array([30, 255, 255])
    lower_blue = np.array([100, 50, 50])
    upper_blue = np.array([120, 255, 255])

    # Createsa a mask wher pixels in range of color are 1(white) and out of range are 0 (black)
    # mask = cv.inRange(hsv, lower_yellow, upper_yellow)
    mask = cv.inRange(hsv, lower_blue, upper_blue)

    # Image refining
    kernel = np.ones((5, 5), np.uint8)
    opening = cv.morphologyEx(mask, cv.MORPH_OPEN, kernel)
    # cv.imshow('opening',opening)

    # bitwise and of the mask
    res = cv.bitwise_and(img, img, mask=opening)

    # cv.imshow('OG',img)
    # cv.imshow('mask',mask)
    
    #If you do not want to see what the camera is detecting just comment this out
    cv.imshow('res', res)

    # This will resize the displayed image however it does slow down the program since this is done for every frame
    
    # scale_percent = 200  # percent of original size
    # width = int(img.shape[1] * scale_percent / 100)
    # height = int(img.shape[0] * scale_percent / 100)
    # dim = (width, height)
    # resized = cv.resize(res, dim, interpolation=cv.INTER_AREA)
    # cv.imshow('smaller.jpg',resized)

    # Threshold to make image binary
    ret, thresh1 = cv.threshold(mask, 1, 255, cv.THRESH_BINARY)
    # Finds nonzero pixels from theshold
    nonzero = cv.findNonZero(mask)
    # Takes the mean of the array for coordinates
    xy = cv.mean(nonzero)
    x = round(xy[0], 2)
    y = round(xy[1], 2)

    # print the dimensions of the shape
    height = img.shape[0]
    width = img.shape[1]
    fieldView = 52
    center = width / 2
    ratio = (center - x) / (width / 2)
    angleToObject = fieldView / 2 * ratio
    angleToObject = round(angleToObject, 2)
    # Convert to radians
    angleToObject = angleToObject

    # print("x"+str(x))
    # print("Ratio"+str(ratio))

    if x == 0 and y == 0:
        print("No Markers Found")
    else:
        #ALEX !!!!!!!!!!!!!!! THIS IS THE ANGLE FROM THE CENTER OF THE CAMERA!
        print("Angle From Center" + str(angleToObject))
        bus = SMBus(1)
        angleToObjectStr = str(round(angleToObject,2))

        split[0] = angleToObjectStr[0]
        split[1] = angleToObjectStr[1]
        split[2] = angleToObjectStr[2]
        split[3] = angleToObjectStr[3]
        bus.write_i2c_block_data(4, 0, split) #write the given number to offset 0
        read = bus.read_i2c_block_data(4, 0)
        current_angle = float(read[0] + read[1] + read[2] + read[3])
        bus.close()
        lcd.message = "Target: " + angleToObject + "\nPosition: " + current_angle
        
        
    
    key = cv.waitKey(1) & 0xFF
    # clear the stream in preparation for the next frame
    rawCapture.truncate(0)
