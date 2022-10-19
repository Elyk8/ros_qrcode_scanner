#!/usr/bin/python3

import rospy
import cv2
import os
import time

from pyzbar.pyzbar import decode
from std_msgs.msg import String
from sensor_msgs.msg import Image from cv_bridge import CvBridge, CvBridgeError
import sys

# Preamble
save_path = os.path.expanduser("~/Documents/Capstone/TurtleBot3/qrdata/")
print(save_path)

if not os.path.exists(save_path):
    os.makedirs(save_path)
    print(save_path)


class image_converter:
    def __init__(self):
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/camera/image", Image, self.image_callback)

    # Whenever the image is received from the camera
    def image_callback(self, ros_image):
        # print('got an image')
        global bridge
        # convert ros_image into an opencv-compatible image
        try:
            cv_image = self.bridge.imgmsg_to_cv2(
                ros_image, desired_encoding="passthrough"
            )
        except CvBridgeError as e:
            print(e)

        # OpenCV Code goes after this comment
        f = open(save_path + "data.txt", "a")
        t = time.localtime()
        current_time = time.strftime("%Y-%m-%d %H:%M:%S", t)
        for code in decode(cv_image):
            # Get geometry of identified barcode/qrcode and draw a rectangle around it
            (x, y, w, h) = code.rect
            cv2.rectangle(cv_image, (x, y), (x + w, y + h), (0, 0, 255), 2)

            print(code.type)
            data = code.data.decode("utf-8")
            print(data)
            if not current_time == time.strftime("%Y-%m-%d %H:%M:%S", t):
                f.write(current_time + " ; " + data + "\n")

        f.close()
        cv2.imshow("Image window", cv_image)
        cv2.waitKey(3)


def main(args):
    ic = image_converter()
    rospy.init_node("scanner_node", anonymous=True)

    # Subscribe to the camera node exposed by the turtlebot3 autorace package
    # image_sub = rospy.Subscriber("/camera/image", Image, image_callback)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main(sys.argv)
