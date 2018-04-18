'''
Author: Rajat Bullakkanavar
Language used: Pythona
Project name: Automated Toll Gate


Note: The following code is compatible with Raspberry pi along with Raspberry Pi Camera only.
Small alterations can be made to make it compatible with OpenCV running in any other processor or with webcam.
'''




# import the necessary packages
from scipy.spatial import distance as dist
from imutils import perspective
from imutils import contours
import numpy as np
import argparse
import imutils
import cv2
import picamera
import RPi.GPIO as GPIO
import time
import Adafruit_CharLCD as LCD

# GPIO Mode (BOARD / BCM)
GPIO.setmode(GPIO.BCM)

# Raspberry Pi pin setup for Ultrasonics sensor
GPIO_TRIGGER = 18
GPIO_ECHO = 24

# Raspberry Pi pin setup for LCD Display
lcd_rs = 25
lcd_en = 21
lcd_d4 = 23
lcd_d5 = 17
lcd_d6 = 26
lcd_d7 = 22
lcd_backlight = 2

# Define LCD column and row size for 16x2 LCD.
lcd_columns = 16
lcd_rows = 2

lcd = LCD.Adafruit_CharLCD(lcd_rs, lcd_en, lcd_d4, lcd_d5, lcd_d6, lcd_d7, lcd_columns, lcd_rows, lcd_backlight)

# set GPIO direction (IN / OUT)
GPIO.setup(GPIO_TRIGGER, GPIO.OUT)
GPIO.setup(GPIO_ECHO, GPIO.IN)


def midpoint(ptA, ptB):
    return ((ptA[0] + ptB[0]) * 0.5, (ptA[1] + ptB[1]) * 0.5)


def default_lcd():
    lcd.message('Automated Toll \n       Gate')
    time.sleep(5.0)
    lcd.clear()
    return;


def classify_lcd(category):
    if category == 0:
        lcd.message(' Vehicle Size :\n')
        lcd.message('     LARGE')
        time.sleep(4, 0)
        lcd.message('Head towards \n')
        lcd.message('     GATE-1')
        time.sleep(4, 0)
        return;
    elif category == 1:
        lcd.message(' Vehicle Size :\n')
        lcd.message('     SMALL')
        time.sleep(4, 0)
        lcd.message('Head towards \n')
        lcd.message('     GATE-2')
        time.sleep(4, 0)
        return;


def distance_ultra():
    # set Trigger to HIGH
    GPIO.output(GPIO_TRIGGER, True)

    # set Trigger after 0.01ms to LOW
    time.sleep(0.00001)
    GPIO.output(GPIO_TRIGGER, False)

    StartTime = time.time()
    StopTime = time.time()

    # save StartTime
    while GPIO.input(GPIO_ECHO) == 0:
        StartTime = time.time()

    # save time of arrival
    while GPIO.input(GPIO_ECHO) == 1:
        StopTime = time.time()

    # time difference between start and arrival
    TimeElapsed = StopTime - StartTime
    # multiply with the sonic speed (34300 cm/s)
    # and divide by 2, because there and back
    distance_ultra = (TimeElapsed * 34300) / 2

    return distance_ultra


default_lcd()
camera = picamera.PiCamera()

if _name_ == '_main_':
    try:
        while True:
            dist_ultra = distance_ultra()
            print ("Measured Distance = %.1f cm" % dist_ultra)
            time.sleep(1)
            if dist_ultra < 6:

                camera.capture('testvehicle.jpg')
                # load the image, convert it to grayscale, and blur it slightly
                image = cv2.imread('testvehicle.jpg', cv2.IMREAD_GRAYSCALE)
                # gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
                gray = cv2.GaussianBlur(image, (7, 7), 0)

                # perform edge detection, then perform a dilation + erosion to
                # close gaps in between object edges
                edged = cv2.Canny(gray, 50, 100)
                edged = cv2.dilate(edged, None, iterations=1)
                edged = cv2.erode(edged, None, iterations=1)

                # find contours in the edge map
                cnts = cv2.findContours(edged.copy(), cv2.RETR_EXTERNAL,
                                        cv2.CHAIN_APPROX_SIMPLE)
                cnts = cnts[0] if imutils.is_cv2() else cnts[1]

                # sort the contours from left-to-right and initialize the
                # 'pixels per metric' calibration variable
                (cnts, _) = contours.sort_contours(cnts)
                pixelsPerMetric = None
                # loop over the contours individually
                for c in cnts:
                    # if the contour is not sufficiently large, ignore it
                    if cv2.contourArea(c) < 100:
                        continue

                    # compute the rotated bounding box of the contour
                    orig = image.copy()
                    box = cv2.minAreaRect(c)
                    box = cv2.cv.BoxPoints(box) if imutils.is_cv2() else cv2.boxPoints(box)
                    box = np.array(box, dtype="int")

                    # order the points in the contour such that they appear
                    # in top-left, top-right, bottom-right, and bottom-left
                    # order, then draw the outline of the rotated bounding
                    # box
                    box = perspective.order_points(box)
                    cv2.drawContours(orig, [box.astype("int")], -1, (0, 255, 0), 2)

                    # loop over the original points and draw them
                    for (x, y) in box:
                        cv2.circle(orig, (int(x), int(y)), 5, (0, 0, 255), -1)

                    # unpack the ordered bounding box, then compute the midpoint
                    # between the top-left and top-right coordinates, followed by
                    # the midpoint between bottom-left and bottom-right coordinates
                    (tl, tr, br, bl) = box
                    (tltrX, tltrY) = midpoint(tl, tr)
                    (blbrX, blbrY) = midpoint(bl, br)

                    # compute the midpoint between the top-left and top-right points,
                    # followed by the midpoint between the top-righ and bottom-right
                    (tlblX, tlblY) = midpoint(tl, bl)
                    (trbrX, trbrY) = midpoint(tr, br)

                    # draw the midpoints on the image
                    cv2.circle(orig, (int(tltrX), int(tltrY)), 5, (255, 0, 0), -1)
                    cv2.circle(orig, (int(blbrX), int(blbrY)), 5, (255, 0, 0), -1)
                    cv2.circle(orig, (int(tlblX), int(tlblY)), 5, (255, 0, 0), -1)
                    cv2.circle(orig, (int(trbrX), int(trbrY)), 5, (255, 0, 0), -1)

                    # draw lines between the midpoints
                    cv2.line(orig, (int(tltrX), int(tltrY)), (int(blbrX), int(blbrY)),
                             (255, 0, 255), 2)
                    cv2.line(orig, (int(tlblX), int(tlblY)), (int(trbrX), int(trbrY)),
                             (255, 0, 255), 2)

                    # compute the Euclidean distance between the midpoints
                    dA = dist.euclidean((tltrX, tltrY), (blbrX, blbrY))
                    dB = dist.euclidean((tlblX, tlblY), (trbrX, trbrY))

                    # if the pixels per metric has not been initialized, then
                    # compute it as the ratio of pixels to supplied metric
                    # (in this case, inches)
                    if pixelsPerMetric is None:
                        pixelsPerMetric = dB / 0.1

                    # compute the size of the object
                    dimA = dA / pixelsPerMetric
                    dimB = dB / pixelsPerMetric

                    if dimA * dimB > 16:
                        print "this is a large vehicle"
                        classify_lcd(category=1)

                    elif dimA * dimB < 16:
                        print "this is a small vehicle"
                        classify_lcd(category=2)

                    # draw the object sizes on the image
                    cv2.putText(orig, "{:.1f}in".format(dimA),
                                (int(tltrX - 15), int(tltrY - 10)), cv2.FONT_HERSHEY_SIMPLEX,
                                0.65, (255, 255, 255), 2)
                    cv2.putText(orig, "{:.1f}in".format(dimB),
                                (int(trbrX + 10), int(trbrY)), cv2.FONT_HERSHEY_SIMPLEX,
                                0.65, (255, 255, 255), 2)

                    # show the output image
                    cv2.imshow("Image", orig)
                    cv2.waitKey(0)
                cv2.destroyAllWindows()

    # Reset by pressing CTRL + C
    except KeyboardInterrupt:
        print("Measurement stopped by User")
        GPIO.cleanup()