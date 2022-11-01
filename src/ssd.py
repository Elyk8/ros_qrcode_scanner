#!/usr/bin/python3

import rospy
import cv2
import numpy as np

from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import sys


# Preamble
thres = 0.45  # Threshold to detect object
nms_threshold = 0.2

classNames = []
classFile = "../lib/coco.names"
with open(classFile, "rt") as f:
    classNames = f.read().rstrip("n").split("n")

# print(classNames)
libpath = "../lib/"
configPath = libpath + "ssd_mobilenet_v3_large_coco_2020_01_14.pbtxt"
weightsPath = libpath + "frozen_inference_graph.pb"

net = cv2.dnn_DetectionModel(weightsPath, configPath)
net.setInputSize(320, 320)
net.setInputScale(1.0 / 127.5)
net.setInputMean((127.5, 127.5, 127.5))
net.setInputSwapRB(True)


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
        classIds, confs, bbox = net.detect(cv_image, confThreshold=thres)
        bbox = list(bbox)
        confs = list(np.array(confs).reshape(1, -1)[0])
        confs = list(map(float, confs))
        # print(type(confs[0]))
        # print(confs)

        indices = cv2.dnn.NMSBoxes(bbox, confs, thres, nms_threshold)
        # print(indices)

        for i in indices:
            i = i[0]
            box = bbox[i]
            x, y, w, h = box[0], box[1], box[2], box[3]
            cv2.rectangle(cv_image, (x, y), (x + w, h + y), color=(0, 255, 0), thickness=2)
            cv2.putText(
                cv_image,
                classNames[classIds[i][0] - 1].upper(),
                (box[0] + 10, box[1] + 30),
                cv2.FONT_HERSHEY_COMPLEX,
                1,
                (0, 255, 0),
                2,
            )

        cv2.imshow("Output", cv_image)
        cv2.waitKey(1)


def main(args):
    ic = image_converter()
    rospy.init_node("ssd_object_detection_node", anonymous=True)

    # Subscribe to the camera node exposed by the turtlebot3 autorace package
    # image_sub = rospy.Subscriber("/camera/image", Image, image_callback)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main(sys.argv)
