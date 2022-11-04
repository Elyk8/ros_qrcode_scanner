#!/usr/bin/python3

import rospy
import cv2
import os
# import time
import numpy as np

from pyzbar.pyzbar import decode
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import sys

# Preamble
save_path = os.path.expanduser("~/Documents/Capstone/TurtleBot3/qrdata/")
print("Save Path" + save_path)

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
        # f = open(save_path + "data.txt", "a")
        # t = time.localtime()
        # current_time = time.strftime("%Y-%m-%d %H:%M:%S", t)
        for code in decode(cv_image):
            # print(code.type)

            data = code.data.decode("utf-8")
            if code.type == "CODE128":
                data = int(data)
                recoveredbytes = data.to_bytes((data.bit_length() + 7) // 8, "little")
                data = recoveredbytes[:-1].decode("utf-8").strip()  # Strip pad after decoding
            # else:
            #     data = code.data.decode("utf-8")

            print(data)

            # Get geometry of identified barcode/qrcode and draw a rectangle around it
            pts = np.array([code.polygon], np.int32)
            pts = pts.reshape((-1, 1, 2))
            cv2.polylines(cv_image, [pts], True, (255, 0, 255), 5)

            pts2 = code.rect
            cv2.putText(
                cv_image,
                data,
                (pts2[0], pts2[1]),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.9,
                (255, 0, 255),
                2,
            )
            # f.write(current_time + " ; " + data + "\n")

        # f.close()
        img = cv2.resize(cv_image, (960, 600))
        cv2.imshow("Image window", img)
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
