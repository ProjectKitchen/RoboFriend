#!/usr/bin/env python3
from imutils.video import VideoStream
import imutils
import rospy
import os
import numpy as np
import tensorflow as tf
import sys
import cv2
import argparse

from utils import label_map_util
from utils import visualization_utils as vis_util

# import ros services
from robofriend.srv import SrvObjectDetection, SrvObjectDetectionResponse
from robofriend.srv import SrvObjectHeartbeatData, SrvObjectHeartbeatDataResponse

# global variables
MJPG_URL = ""
TF = {}
STREAM = {}
IM_WIDTH = 640
IM_HEIGHT = 480

def object_detetcion_init():
    global TF

    model_name = 'ssdlite_mobilenet_v2_coco_2018_05_09'

    # grab path to current working directory
    cwd_path = os.path.dirname(os.path.realpath(__file__))

    # Path to the frozen detection graph .pb file, which contains the model that is
    # for object detecion
    model_path = os.path.join(cwd_path, 'frozen_inference_graph.pb')

    # path to label map file
    label_path = os.path.join(cwd_path, 'mscoco_label_map.pbtxt')

    # number of classes the object detetctor can identify
    num_classes = 90

    label_map = label_map_util.load_labelmap(label_path)
    categories = label_map_util.convert_label_map_to_categories(label_map,
        max_num_classes = num_classes, use_display_name = True)
    TF["category_index"] = label_map_util.create_category_index(categories)

    # Load the Tensorflow model into memory
    detection_graph = tf.Graph()
    with detection_graph.as_default():
        od_graph_def = tf.GraphDef()
        with tf.gfile.GFile(model_path, 'rb') as fid:
            serialized_graph = fid.read()
            od_graph_def.ParseFromString(serialized_graph)
            tf.import_graph_def(od_graph_def, name='')

        TF["sess"] = tf.Session(graph=detection_graph)

    # Input tensor is the image
    TF["image_tensor"] = detection_graph.get_tensor_by_name('image_tensor:0')

    # Output tensors are the detection boxes, scores, and classes
    # Each box represents a part of the image where a particular object was detected
    TF["detection_boxes"] = detection_graph.get_tensor_by_name('detection_boxes:0')

    # Each score represents level of confidence for each of the objects.
    # The score is shown on the result image, together with the class label.
    TF["detection_scores"] = detection_graph.get_tensor_by_name('detection_scores:0')
    TF["detection_classes"] = detection_graph.get_tensor_by_name('detection_classes:0')

    # Number of objects detected
    TF["num_detections"] = detection_graph.get_tensor_by_name('num_detections:0')

def stream_init():
    global MJPG_URL, STREAM
    vs = None

    MJPG_URL = get_url_address()

    vs = cv2.VideoCapture(MJPG_URL)
    if vs.isOpened():
        rospy.logdebug("Pictures are grabed from mjpg-streamer!")
        mjpg_stream = True
    else:
        rospy.logdebug("Pcitures are taken from webcam!")
        vs = VideoStream(src = 0).start()
        mjpg_stream = False
    STREAM["vs"] = vs
    STREAM["mjpg_stream"] = mjpg_stream

def get_url_address():
    ap = argparse.ArgumentParser()
    ap.add_argument("-ip")
    args = vars(ap.parse_args())
    mjpg_url = "http://" + args["ip"] + ":8080/?action=stream"

    return mjpg_url

def service_handler(request):
    retVal = None
    rospy.logdebug("{%s} - Request received!", rospy.get_caller_id())

    if request.detect_obj is True:
        obj = start_object_detecion()
        rospy.logdebug("{%s} - Detected Object: %s",
            rospy.get_caller_id(), str(obj))
        retVal = obj
    else:
        retVal = None
    return SrvObjectDetectionResponse(retVal)

def objectdetection_hb_handler(request):
    rospy.logwarn("Objectdetecion HB request received!")
    return SrvObjectHeartbeatDataResponse(True)

def start_object_detecion():
    global TF, STREAM, MJPG_URL, WIDTH, HEIGHT

    vs = None
    frame = None

    if STREAM["mjpg_stream"] is True:
        vs = cv2.VideoCapture(MJPG_URL)
        stat, frame = vs.read()
    else:
        frame = STREAM["vs"].read()

    if frame is None:
        obj = None
    elif frame.any():
        frame = imutils.resize(frame, width = IM_WIDTH, height = IM_HEIGHT)
        frame_expanded = np.expand_dims(frame, axis = 0)

        # Perform the actual detection by running the model with the image as input
        (boxes, scores, classes, num) = TF["sess"].run(
            [TF["detection_boxes"], TF["detection_scores"], TF["detection_classes"], TF["num_detections"]],
            feed_dict = {TF["image_tensor"]: frame_expanded}
        )

        # Draw the results of the detection (aka 'visulaize the results')
        image, obj = vis_util.visualize_boxes_and_labels_on_image_array(
            frame,
            np.squeeze(boxes),
            np.squeeze(classes).astype(np.int32),
            np.squeeze(scores),
            TF["category_index"],
            use_normalized_coordinates=True,
            line_thickness=8,
            min_score_thresh=0.85)

    return obj

def shutdown():
    rospy.signal_shutdown("controlled shutdown")

def ObjectDetection():
    global MJPG_URL

    rospy.init_node("robofriend_object_detection_node", log_level = rospy.INFO)
    rospy.loginfo("{%s} - starting object detetcion node!", rospy.get_caller_id())
    rospy.on_shutdown(shutdown)

    # initializes all staff regarding tensorflow
    object_detetcion_init()

    # initializes the stream
    stream_init()

    # declare services
    rospy.Service('robofriend/detect_objects', SrvObjectDetection, service_handler)
    rospy.Service('/robofriend/obj_heartbeat', SrvObjectHeartbeatData, objectdetection_hb_handler)

    rospy.spin()

if __name__ == '__main__':
    try:
        ObjectDetection()
    except rospy.ROSInterruptException:
        pass
