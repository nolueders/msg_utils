from sensor_msgs.msg import Image as SensorImage

from cv_bridge import CvBridge, CvBridgeError
import numpy as np


# @pil_img: PIL image to be written to ros message
# @frame: parent coordinate frame
# @encoding: encoding of image data
# @return: pil image as sensor_msgs.msg.Image
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
# @img: cv2 image, which shall be converted
# @frame: parent coordinate frame
# @encoding: encoding of image data
# @return: cv2 image as sensor_msgs.msg.Image
def write_cv2_img(img, frame, encoding="rgb8"):
    bridge = CvBridge()
    try:
        img_msg = bridge.cv2_to_imgmsg(img, encoding)
        img_msg.header.frame_id = frame
    except CvBridgeError as e:
        print(e)

    return img_msg


# https://wiki.ros.org/cv_bridge/Tutorials/ConvertingBetweenROSImagesAndOpenCVImagesPython
# @img: image as ros message
# @encoding: encoding of image data
# @return: image as cv2 image
def read_img_as_cv2(img, encoding="rgb8"):
    bridge = CvBridge()
    try:
        cv2_img = bridge.imgmsg_to_cv2(img, encoding)
    except CvBridgeError as e:
        print(e)

    return cv2_img


# @img: image as ros message
# @encoding: encoding of image data
# @return: image as numpy array
def read_img_as_np_arr(img, encoding="rgb8"):
    return np.asarray(read_img_as_cv2(img=img, encoding=encoding))
