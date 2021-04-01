from sensor_msgs.msg import Image as SensorImage

from cv_bridge import CvBridge, CvBridgeError
import cv2 as cv
import numpy as np


# @pil_img: PIL image to be written to ros message
# @frame: parent coordinate frame
def write_pil_img(pil_img, frame, encoding="rgb8"):
    img_msg = SensorImage()
    img_msg.header.frame_id = frame
    img_msg.height = pil_img.height
    img_msg.width = pil_img.width
    img_msg.encoding = "rgb8"
    img_msg.is_bigendian = False
    img_msg.step = 3 * pil_img.width
    img_msg.data = np.array(pil_img).tobytes()
    return img_msg


# https://wiki.ros.org/cv_bridge/Tutorials/ConvertingBetweenROSImagesAndOpenCVImagesPython
def read_img_as_cv2(img, encoding="rgb8"):
    bridge = CvBridge()
    try:
        cv2_img = bridge.imgmsg_to_cv2(img, encoding)
    except CvBridgeError as e:
        print(e)

    (rows, cols, channels) = cv2_img.shape
    if cols > 60 and rows > 60:
        cv.circle(cv2_img, (50, 50), 10, 255)

    return cv2_img


def write_cv2_img(img, encoding="rgb8"):
    bridge = CvBridge()
    try:
        img_msg = bridge.cv2_to_imgmsg(img, encoding)
    except CvBridgeError as e:
        print(e)

    return img_msg
