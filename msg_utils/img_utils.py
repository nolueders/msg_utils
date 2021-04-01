from sensor_msgs.msg import Image as SensorImage

from PIL import Image
import cv2 as cv
import numpy as np


# @pil_img: PIL image to be written to ros message
# @frame: parent coordinate frame
def write_pil_img(pil_img, frame):
    img_msg = SensorImage()
    img_msg.header.frame_id = frame
    img_msg.height = pil_img.height
    img_msg.width = pil_img.width
    img_msg.encoding = "rgb8"
    img_msg.is_bigendian = False
    img_msg.step = 3 * pil_img.width
    img_msg.data = np.array(pil_img).tobytes()
    return img_msg
