#! /usr/bin/env python3
import roslib
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np
import os
import cv2
import imutils


class RS_Detector():
    def __init__(self):
        self.br = CvBridge()
        # Get the ~private namespace parameters from command line or launch file.
        self.debug = bool(rospy.get_param('~debug', 'true'))

        self.debug_pub = rospy.Publisher('rs_detector/debug', Image, queue_size=10)
        self.sub = rospy.Subscriber('/camera/image', Image, callback=self.process)
        # Set the message to publish as our custom message.
        rospy.loginfo("Roadsign Detector Initialised!")
        
        rospy.spin()

    def process(self, imag):
        rospy.loginfo_throttle(60, "Processing...")

        imag = self.br.imgmsg_to_cv2(imag, "rgb8")
        imag = cv2.cvtColor(imag, cv2.COLOR_RGB2BGR)
        label_list = list()
        cnts_list = list()
        mser_blue = cv2.MSER_create(8, 400, 4000)

        img = imag.copy()
        img_yuv = cv2.cvtColor(img, cv2.COLOR_BGR2YUV)
        
        # equalize the histogram of the Y channel
        img_yuv[:, :, 0] = cv2.equalizeHist(img_yuv[:, :, 0])

        # convert the YUV image back to RGB format
        img_output = cv2.cvtColor(img_yuv, cv2.COLOR_YUV2BGR)

        # convert the image to HSV format for color segmentation
        img_hsv = cv2.cvtColor(imag, cv2.COLOR_BGR2HSV)

        # mask to extract blue
        lower_blue = np.array([94, 127, 20])
        upper_blue = np.array([126, 255, 200])
        mask = cv2.inRange(img_hsv, lower_blue, upper_blue)

        blue_mask = cv2.bitwise_and(img_output, img_output, mask=mask)

        # seperate out the channels
        r_channel = blue_mask[:, :, 2]
        g_channel = blue_mask[:, :, 1]
        b_channel = blue_mask[:, :, 0]

        # filter out
        filtered_r = cv2.medianBlur(r_channel, 5)
        filtered_g = cv2.medianBlur(g_channel, 5)
        filtered_b = cv2.medianBlur(b_channel, 5)

        # create a blue gray space
        filtered_b = -0.5 * filtered_r + 3 * filtered_b - 2 * filtered_g

        # Do MSER
        regions, _ = mser_blue.detectRegions(np.uint8(filtered_b))

        hulls = [cv2.convexHull(p.reshape(-1, 1, 2)) for p in regions]

        blank = np.zeros_like(blue_mask)
        cv2.fillPoly(np.uint8(blank), hulls, (255, 0, 0))


        self.debug_pub.publish(self.br.cv2_to_imgmsg(blank))

        # cv2.imshow("mser_blue", blank)
        # kernel_1 = np.ones((3, 3), np.uint8)
        # kernel_2 = np.ones((5, 5), np.uint8)

        # erosion = cv2.erode(blank, kernel_1, iterations=1)
        # dilation = cv2.dilate(erosion, kernel_2, iterations=1)
        # opening = cv2.morphologyEx(dilation, cv2.MORPH_OPEN, kernel_2)

        # _, b_thresh = cv2.threshold(opening[:, :, 0], 60, 255, cv2.THRESH_BINARY)

        # cnts = cv2.findContours(b_thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        # cnts = imutils.grab_contours(cnts)
        # max_cnts = 3  # no frame we want to detect more than 3

        # if not cnts == []:
        #     cnts_sorted = sorted(cnts, key=cv2.contourArea, reverse=True)
        #     if len(cnts_sorted) > max_cnts:
        #         cnts_sorted = cnts_sorted[:3]

        #     for c in cnts_sorted:
        #         x, y, w, h = cv2.boundingRect(c)
        #         if x < 100:
        #             continue
        #         if h < 20:
        #             continue

        #         if y > 400:
        #             continue

        #         aspect_ratio_1 = w / h
        #         aspect_ratio_2 = h / w
        #         if aspect_ratio_1 <= 0.5 or aspect_ratio_1 > 1.2:
        #             continue
        #         if aspect_ratio_2 <= 0.5:
        #             continue

        #         hull = cv2.convexHull(c)

        #         # cv2.rectangle(imag, (x, y), (int(x+w), int(y+h)), (0, 255, 0), 2)
        #         # cv2.drawContours(imag, [hull], -1, (0, 255, 0), 2)

        #         mask = np.zeros_like(imag)
        #         # cv2.drawContours(mask, [c], -1, (255, 255, 255), -1)  # Draw filled contour in mask
        #         cv2.rectangle(mask, (x, y), (int(x + w), int(y + h)), (255, 255, 255), -1)
        #         out = np.zeros_like(imag)  # Extract out the object and place into output image
        #         out[mask == 255] = imag[mask == 255]

        #         x_pixel, y_pixel, _ = np.where(mask == 255)
        #         (topx, topy) = (np.min(x_pixel), np.min(y_pixel))
        #         (botx, boty) = (np.max(x_pixel), np.max(y_pixel))
        #         if np.abs(topx - botx) <= 25 or np.abs(topy - boty) <= 25:
        #             continue

        #         out = imag[topx:botx + 1, topy:boty + 1]
        #         out_resize = cv2.resize(out, (64, 64), interpolation=cv2.INTER_CUBIC)
        #         predict, prob = train.test_blue(clf_blue, out_resize)
        #         print(np.max(prob))
        #         if np.max(prob) < 0.78:
        #             continue
        #         #cv2.rectangle(imag, (x, y), (int(x + w), int(y + h)), (0, 255, 0), 2)
        #         label = predict[0]
        #         if label == 100:
        #             continue
                
        #         cnts_list.append(c)
        #         label_list.append(label)
        #     return cnts_list, label_list
        # else:
        #     return None, None


# Main function.
if __name__ == '__main__':
    # Initialize the node and name it.
    rospy.init_node('rs_detector')
    # Go to class functions that do all the heavy lifting. Do error checking.
    try:
        rs_detector = RS_Detector()
        
    except rospy.ROSInterruptException: pass