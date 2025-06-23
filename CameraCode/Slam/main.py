# This work is licensed under the MIT license.
# Copyright (c) 2013-2023 OpenMV LLC. All rights reserved.
# https://github.com/openmv/openmv/blob/master/LICENSE
#
# Hello World Example
#
# Welcome to the OpenMV IDE! Click on the green run arrow button below to run the script!

import sensor, time, ustruct
from pyb import UART
from pyb import LED

red_led = LED(1)
green_led = LED(2)
blue_led = LED(3)

red_led.on()
green_led.on()
blue_led.on()
DEBUG = True

# UART 3, and baudrate.
uart = UART(3, 19200, timeout_char=200)

uart.any()

RED_COLOR = (255, 0, 0)
GREEN_COLOR = (0, 255, 0)
BLUE_COLOR = (0, 0, 255)

red_th = (0, 100, 38, 127, -128, 127)
green_th = (0, 100, -128, -32, -128, 127)
blue_th = (0, 100, -128, 127, -128, -20)

gray_th = (39, 58, -128, 127, -128, 127)
white_th = (75, 100, -128, 127, -128, 127)
black_th = (0, 35, -128, 127, -128, 127)

red_stand_roi = (85, 30, 90, 180)
red_line_roi = (240, 0, 80, 240)

green_stand_roi = (0, 40, 80, 160)
green_stand_left_roi = (70, 170, 160, 70)
green_stand_right_roi = (70, 0, 160, 70)

circle_left_roi = (15, 140, 150, 100)
circle_right_roi = (15, 0, 150, 100)


gray_roi = (38, 52, 183, 114)

red_stand_size = 3500


sensor.reset() # Resets the sensor
sensor.set_pixformat(sensor.RGB565) # Sets the sensor to RGB
sensor.set_framesize(sensor.QVGA) # Sets the resolution to 320x240 px
sensor.set_auto_gain(False, gain_db=13)
sensor.set_auto_whitebal(False, rgb_gain_db=(-4.99755, -6.0206, -3.86792))
sensor.set_auto_exposure(False, exposure_us=10000)
sensor.set_contrast(1) # range -3 to +3
sensor.set_brightness(1) # range -3 to +3
sensor.set_saturation(1) # range -3 to +3
sensor.__write_reg(0x0E, 0b00000000) # Disable night mode
sensor.__write_reg(0x3E, 0b00000000) # Disable BLC
sensor.__write_reg(0x2D, 0b00000000) # LSB Insert Dummy Rows (Set to Default 0x00)
sensor.__write_reg(0x2E, 0b00000000) # MSB Insert Dummy Rows (Set to Default 0x00)
sensor.__write_reg(0x35, 0b10000000) # AD Offset B Chan (Set to Default 0x80)
sensor.__write_reg(0x36, 0b10000000) # AD Offset R Chan (Set to Default 0x80)
sensor.__write_reg(0x37, 0b10000000) # AD Offset Gb Chan (Set to Default 0x80)
sensor.__write_reg(0x38, 0b10000000) # AD Offset Gr Chan (Set to Default 0x80)
sensor.__write_reg(0x39, 0b10000000) # B channel offset (Set to Default 0x80)
sensor.__write_reg(0x3A, 0b10000000) # R channel offset (Set to Default 0x80)
sensor.__write_reg(0x3B, 0b10000000) # Gb channel offset (Set to Default 0x80)
sensor.__write_reg(0x3C, 0b10000000) # Gr channel offset (Set to Default 0x80)


sensor.skip_frames(time=2000)


# sensor.set_auto_exposure(False, exposure_us=5400)
# sensor.set_brightness(0)
# sensor.set_contrast(0)
# sensor.set_saturation(0)
# sensor.set_auto_blc(False, regs=[130, 130, 129, 129, 148, 158, 143, 141])

clock = time.clock()  # Create a clock object to track the FPS.

if uart.any():
    uart.read(uart.any())

while True:

    clock.tick()  # Update the FPS clock.
    img = sensor.snapshot()  # Take a picture and return the image.
    img.lens_corr(1.64)
    data = [0, 0, 0, 0, 0]

    # Ищем красную линию

    # img.draw_rectangle(*red_stand_roi, RED_COLOR)
    red_line = img.find_blobs([red_th], pixels_threshold=1000, roi=red_line_roi)
    red_stand = img.find_blobs([red_th], pixels_threshold=1000, roi=red_stand_roi)
    # print(red_stand)


    green_stand = img.find_blobs([green_th], pixels_threshold=2000, roi=green_stand_roi)
    green_stand_left = img.find_blobs([green_th], pixels_threshold=1000, roi=green_stand_left_roi)
    green_stand_right = img.find_blobs([green_th], pixels_threshold=1000, roi=green_stand_right_roi)

    blue_circle_left = img.find_blobs([blue_th], pixels_threshold=2500, roi=circle_left_roi)
    blue_circle_right = img.find_blobs([blue_th], pixels_threshold=2500, roi=circle_right_roi)
    red_circle_left = img.find_blobs([red_th], pixels_threshold=2500, roi=circle_left_roi)
    red_circle_right = img.find_blobs([red_th], pixels_threshold=2500, roi=circle_right_roi)


    blackBlobs = img.find_blobs([black_th])
    summBlack = 0
    for obj in blackBlobs:
        summBlack += obj.pixels()

    gray_tube = img.find_blobs([gray_th], pixels_threshold=1000, roi=gray_roi)

    summGray = 0
    for obj in gray_tube:
        summGray += obj.pixels()


    if len(red_stand) != 0:
        data[0] = 2

    if len(red_line) != 0:
        data[1] = 1


    if len(green_stand_left) != 0:
        data[2] = 1
    if len(green_stand_right) != 0:
        data[2] = 2
    if len(green_stand) != 0:
        data[2] = 3

    if len(red_stand) == 0 and summGray >5000:
        data[0] = 1

    if (len(blue_circle_left) != 0 or len(blue_circle_right) != 0 ) and (len(red_circle_left) != 0 or len(red_circle_right) != 0):
        data[0] = 1
        data[3] = 1

    if summBlack > 30000:
        data[4] = 1

    # img.binary([gr_th])

    # img.binary([gray_th])
    # print(summGray)


    if uart.any():
        uart.read(uart.any())
        uart.write(ustruct.pack("<bbbbb", *data))
        print("Sent", data)
    else:
        print(data)
    # print(summBlack)
    img.draw_rectangle(*circle_left_roi, (255, 0, 0))
    img.draw_rectangle(*circle_right_roi, (255, 0, 0))
    # img.draw_rectangle(*greenPostRoi, (0, 255, 0))
    # img.draw_rectangle(*greenRightRoi, (0, 255, 255))
    # img.draw_rectangle(*greenLeftRoi, (0, 255, 255))
    # img.draw_rectangle(*lineROI, (255, 0, 0))
    # img.draw_rectangle(*circleLeftRoi, (255, 0, 255))
    # img.draw_rectangle(*circleRightRoi, (255, 0, 255))
