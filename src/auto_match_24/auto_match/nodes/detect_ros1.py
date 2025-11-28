#!/usr/bin/env python
## Author: Rohit
## Date: July, 25, 2017
# Purpose: Ros node to detect objects using tensorflow 2

import os
import sys
import cv2
import numpy as np
import tensorflow as tf

# ROS related imports
import rospy
from std_msgs.msg import String, Header
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose

# Object detection module imports
from object_detection.utils import label_map_util
from object_detection.utils import visualization_utils as vis_util

# SET FRACTION OF GPU YOU WANT TO USE HERE
GPU_FRACTION = 0.5

######### Set model here ############
MODEL_NAME = 'model_export'
# By default models are stored in data/models/
MODEL_PATH = os.path.join(os.path.dirname(sys.path[0]), 'data', 'models', MODEL_NAME, 'saved_model')
######### Set the label map file here ###########
LABEL_NAME = 'myraicom.pbtxt'
# By default label maps are stored in data/labels/
PATH_TO_LABELS = os.path.join(os.path.dirname(sys.path[0]), 'data', 'labels', LABEL_NAME)
######### Set the number of classes here #########
NUM_CLASSES = 3

# Load the TensorFlow model
detect_fn = tf.saved_model.load(MODEL_PATH)

## Loading label map
label_map = label_map_util.load_labelmap(PATH_TO_LABELS)
categories = label_map_util.convert_label_map_to_categories(label_map, max_num_classes=NUM_CLASSES, use_display_name=True)
category_index = label_map_util.create_category_index(categories)

# Set GPU options
gpus = tf.config.experimental.list_physical_devices('GPU')
if gpus:
    try:
        for gpu in gpus:
            tf.config.experimental.set_memory_growth(gpu, True)
    except RuntimeError as e:
        print(e)

@tf.function
def detect_objects(detect_fn, input_tensor):
    return detect_fn(input_tensor)

# Detection class
class Detector:

    def __init__(self):
        self.image_pub = rospy.Publisher("debug_image", Image, queue_size=1)
        self.object_pub = rospy.Publisher("objects", Detection2DArray, queue_size=1)
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("image", Image, self.image_cb, queue_size=1, buff_size=2**24)

    def image_cb(self, data):
        objArray = Detection2DArray()
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)
            return

        # Convert image to RGB and ensure it's uint8
        image_rgb = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
        image_np = np.asarray(image_rgb, dtype=np.uint8)

        # Convert image to tensor
        input_tensor = tf.convert_to_tensor(np.expand_dims(image_np, 0), dtype=tf.uint8)

        # Perform detection
        detections = detect_objects(detect_fn, input_tensor)

        # Extract detection results
        boxes = detections['detection_boxes'][0].numpy()
        classes = detections['detection_classes'][0].numpy().astype(np.int32)
        scores = detections['detection_scores'][0].numpy()

        # Visualize detection results
        vis_util.visualize_boxes_and_labels_on_image_array(
            image_np,
            boxes,
            classes,
            scores,
            category_index,
            use_normalized_coordinates=True,
            line_thickness=2)

        objArray.detections = []
        objArray.header = data.header

        for i in range(len(boxes)):
            if scores[i] > 0.5:  # Only consider detections with score > 0.5
                objArray.detections.append(self.object_predict(boxes[i], scores[i], classes[i], data.header, image_np, cv_image))

        self.object_pub.publish(objArray)

        img_out = cv2.cvtColor(image_np, cv2.COLOR_RGB2BGR)
        image_out = Image()
        try:
            image_out = self.bridge.cv2_to_imgmsg(img_out, "bgr8")
        except CvBridgeError as e:
            print(e)
        image_out.header = data.header
        self.image_pub.publish(image_out)

    def object_predict(self, box, score, class_id, header, image_np, image):
        image_height, image_width, _ = image.shape
        obj = Detection2D()
        obj_hypothesis = ObjectHypothesisWithPose()

        obj.header = header
        obj_hypothesis.id = class_id
        obj_hypothesis.score = score
        obj.results.append(obj_hypothesis)
        obj.bbox.size_y = int((box[2] - box[0]) * image_height)
        obj.bbox.size_x = int((box[3] - box[1]) * image_width )
        obj.bbox.center.x = int((box[1] + box[3]) * image_width / 2)
        obj.bbox.center.y = int((box[0] + box[2]) * image_height / 2)

        return obj

def main(args):
    rospy.init_node('detector_node')
    obj = Detector()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("ShutDown")
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
