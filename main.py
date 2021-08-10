# -*- coding: utf-8 -*-
import serial
import cv2
import numpy
import time
import math

# red square detection flag for stopping
red_square_detect = 0

# ----------------------------------------------------
# direction of movement F(Forward), L(Left), R(Right)
# ----------------------------------------------------
state = 'F'

# ----------------------------------------------------
# driving time (1 - 9)
# ----------------------------------------------------
time_delay = 0

# frame size
weight = 320
height = 240

# width detection zones
edge = 18

# colors in hsv space
low_red = numpy.array((0, 90, 90), numpy.uint8)
high_red = numpy.array((25, 255, 255), numpy.uint8)

low_green = numpy.array((30, 90, 90), numpy.uint8)
high_green = numpy.array((70, 255, 255), numpy.uint8)

low_blue = numpy.array((100, 90, 90), numpy.uint8)
high_blue = numpy.array((140, 255, 255), numpy.uint8)

if __name__ == '__main__':
    def nothing(*arg):
        pass


    # Setting up the camera
    cap = cv2.VideoCapture(0)

    # Adjusting the size of the image window
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, weight)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)

    time_send = 0
    while True:

        # Getting an image from the camera
        flag, img = cap.read()

        try:
            # Converting the image to HSV format
            img_hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

            # Get masks by colors in a certain range
            mask_red1 = cv2.inRange(img_hsv, low_red, high_red)
            mask_green1 = cv2.inRange(img_hsv, low_green, high_green)
            mask_blue1 = cv2.inRange(img_hsv, low_blue, high_blue)

            # Us the median filter
            mask_red1 = cv2.medianBlur(mask_red1, 3)
            mask_green1 = cv2.medianBlur(mask_green1, 3)
            mask_blue1 = cv2.medianBlur(mask_blue1, 3)

            cv2.imshow("Mask_red", mask_red1)
            cv2.imshow("Mask_green", mask_green1)
            cv2.imshow("Mask_blue", mask_blue1)

            # Looking for contours and storing them in the contours variable

            # For the contour, we store only 4 extreme points (cv2. CHAIN_APPROX_SIMPLE)
            contours_red, hierarchy_red = cv2.findContours(mask_red1, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
            contours_green, hierarchy_green = cv2.findContours(mask_green1, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
            contours_blue, hierarchy_blue = cv2.findContours(mask_blue1, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)

            # Looking for a contour for which the condition is met and select it with a red rectangle
            for i in contours_red:
                # s1 contour area
                # s2 the area of the minimum described rectangle
                s1 = cv2.contourArea(i)
                s2 = cv2.minAreaRect(i)[1][0] * cv2.minAreaRect(i)[1][1]

                if s1 >= s2 * 0.8 and s1 <= s2 * 1.2 and s1 >= 20000:
                    red_square_detect = 1
                    rect = cv2.minAreaRect(i)
                    box = cv2.boxPoints(rect)
                    box = numpy.int0(box)
                    cv2.drawContours(img, [box], 0, (0, 0, 255), 2)

            # Looking for a contour for which the condition is met and select it with a green rectangle
            for i in contours_green:
                # s1 contour area
                # s2 the area of the minimum described rectangle
                s1 = cv2.contourArea(i)
                s2 = cv2.minAreaRect(i)[1][0] * cv2.minAreaRect(i)[1][1]

                if s1 >= s2 * 0.8 and s1 <= s2 * 1.2 and s1 >= 20000:
                    red_square_detect = 0
                    rect = cv2.minAreaRect(i)
                    box = cv2.boxPoints(rect)
                    box = numpy.int0(box)
                    cv2.drawContours( img, [box], 0, (0,255,0), 2)


            max_area_blue = 30
            max_contour_blue = 0
            # Looking for the largest contour below the center of the screen and select it with a blue rectangle rectangle
            for i in contours_blue:
                s = cv2.contourArea(i)
                # if s > max_area_blue and cv2.minAreaRect(i)[0][1] >= height/2:
                if s > max_area_blue:
                    max_area_blue = s

            # угол наклона контура (+ вправо, - влево от вертикальной оси)
            angle_rotate = 0

            for i in contours_blue:
                s = cv2.contourArea(i)

                if s == max_area_blue:

                    # минимальный описанный прямоугольник
                    rect = cv2.minAreaRect(i)
                    # координаты 4 сторон
                    box = cv2.boxPoints(rect)
                    # объединяем в целочисленный массив
                    # box = numpy.int0(box)

                    # сортируем координаты в порядке 1 2 3 4 почасовой стрелке
                    # начиная с координаты с самым маленьким x
                    y_min = 240
                    y_max = 0

                    x_min = 320
                    x_max = 0

                    for i in range(4):
                        if box[i][0] < x_min:
                            x_min = box[i][0]

                        if box[i][1] < y_min:
                            y_min = box[i][1]

                        if box[i][0] > x_max:
                            x_max = box[i][0]

                        if box[i][1] > y_max:
                            y_max = box[i][1]

                    x11 = 0;
                    y11 = 0;
                    x22 = 0;
                    y22 = 0;
                    x33 = 0;
                    y33 = 0;
                    x44 = 0;
                    y44 = 0;

                    for i in range(4):
                        if box[i][0] == x_min:
                            x11 = x_min
                            y11 = box[i][1]

                        if box[i][1] == y_min:
                            y22 = y_min
                            x22 = box[i][0]

                        if box[i][0] == x_max:
                            x33 = x_max
                            y33 = box[i][1]

                        if box[i][1] == y_max:
                            y44 = y_max
                            x44 = box[i][0]

                    if (x11 == x44 and y11 == y44):
                        y11 = y_min
                        y44 = y_max

                    # находим линию вдоль одной и другой стороны
                    x1 = x22 + (x33 - x22) / 2
                    y1 = y22 + (y33 - y22) / 2

                    x2 = x11 + (x44 - x11) / 2
                    y2 = y11 + (y44 - y11) / 2

                    dl_1 = math.sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2))

                    x3 = x11 + (x22 - x11) / 2
                    y3 = y22 + (y11 - y22) / 2

                    x4 = x44 + (x33 - x44) / 2
                    y4 = y33 + (y44 - y33) / 2

                    dl_2 = math.sqrt((x3 - x4) * (x3 - x4) + (y3 - y4) * (y3 - y4))

                    time_send += 1

                    if (dl_1 > dl_2):
                        cv2.line(img, (int(x1), int(y1)), (int(x2), int(y2)), (0, 0, 255), 5)
                        cv2.circle(img, (int(x1), int(y1)), 2, (0, 0, 255), -1)
                        cv2.circle(img, (int(x2), int(y2)), 2, (255, 0, 0), -1)

                        if (x2 - x1 == 0):
                            x2 = x1 + 0.01

                        angle_rotate = math.degrees(math.atan((y2 - y1) / (x2 - x1)))

                        if (abs(angle_rotate) > 87):
                            state = 'F'
                        elif (angle_rotate > 0):
                            state = 'L'
                        else:
                            state = 'R'
                        # print (state, int(90 - abs(angle_rotate)))

                    else:
                        cv2.line(img, (int(x3), int(y3)), (int(x4), int(y4)), (0, 0, 255), 5)
                        cv2.circle(img, (int(x3), int(y3)), 2, (0, 0, 255), -1)
                        cv2.circle(img, (int(x4), int(y4)), 2, (255, 0, 0), -1)

                        if (x4 - x3 == 0):
                            x4 = x3 + 0.01

                        angle_rotate = math.degrees(math.atan((y4 - y3) / (x4 - x3)))
                        if (abs(angle_rotate) > 87):
                            state = 'F'
                        elif (angle_rotate > 0):
                            state = 'L'
                        else:
                            state = 'R'
                        # print (state, int(90 - abs(angle_rotate)))

                    '''cv2.circle( img, (int(x_1), int(y_1)), 2, (0,255,0), -1)
                    cv2.circle( img, (int(x_2), int(y_2)), 2, (0,255,0), -1)
                    cv2.circle( img, (int(x_3), int(y_3)), 2, (0,255,0), -1)
                    cv2.circle( img, (int(x_4), int(y_4)), 2, (0,255,0), -1)'''

                    cv2.circle(img, (int(x11), int(y11)), 2, (0, 100, 0), -1)
                    cv2.circle(img, (int(x22), int(y22)), 2, (0, 150, 0), -1)
                    cv2.circle(img, (int(x33), int(y33)), 2, (0, 200, 0), -1)
                    cv2.circle(img, (int(x44), int(y44)), 2, (0, 250, 0), -1)

                    time_delay = int(90 - abs(angle_rotate)) / 22
                    if (time_delay == 0):
                        time_delay = 1

                    # cv2.drawContours( img, [box], 0, (255,0,0), 2)

            # controlling the robot

            # time_send = time.perf_counter() - time_send

            if (time_send % 3 == 0):

                time_send = 1

                if red_square_detect == 1:
                    # stop
                    # port.write(bytes(string, 'utf-8'))
                    port.write(";2" + str(time_delay) + "\n")
                    print("stop")

                elif state == 'R':
                    # moving right
                    port.write(";4" + str(time_delay) + "\n")
                    print(";4" + str(int(time_delay)))

                elif state == 'L':
                    # moving left
                    port.write(";3" + str(time_delay) + "\n")
                    print(";3" + str(int(time_delay)))

                elif state == 'F':
                    # moving forward
                    port.write(";1" + str(time_delay + 1) + "\n")
                    print(";1" + str(int(time_delay + 1)))

            cv2.imshow('contours', img)

        except:
            cap.release()
            raise

        ch = cv2.waitKey(50)
        # to exit, press esc
        if ch == 27:
            break

    cap.release()
    cv2.destroyAllWindows()
