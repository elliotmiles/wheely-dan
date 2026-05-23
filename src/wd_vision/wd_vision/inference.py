import os
import sys
import argparse
import glob
import time

import cv2 as cv
import numpy as np
from ultralytics import YOLO

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

def setup_model(model_path):
    # check if model file exists and is valid
    if (not os.path.exists(model_path)):
        print('ERROR: Model path is invalid or model was not found. Make sure the model filename was entered correctly.')
        sys.exit(0)

    # load model 
    model = YOLO(model_path, task='detect')
    return model


def setup_recording(record, resW, resH):
    # set up recording
    if record:
        record_name = 'demo1.avi'
        record_fps = 30
        recorder = cv.VideoWriter(record_name, cv.VideoWriter_fourcc(*'MJPG'), record_fps, (resW,resH))
        return recorder
    return None

# moving average of detections over frames
def ema(prev, new, alpha):
    if new is None:
        return prev
    if prev is None:
        return new
    x = alpha * new[0] + (1 - alpha) * prev[0]
    y = alpha * new[1] + (1 - alpha) * prev[1]
    return (int(x), int(y))


def inference(frame, model, labels, resize, resW, resH, record, recorder, bbox_colours, min_thresh, avg_frame_rate):

    # begin inference loop
    
    if frame is None:
        print('Unable to read frames from the camera. This indicates the camera is disconnected or not working. Exiting program.')

    # resize frame
    if resize == True:
        frame = cv.resize(frame,(resW,resH))

    # run inference on frame
    results = model(frame, verbose=False)

    # extract results
    detections = results[0].boxes

    detections_count = 0


    # go through each detection and get bbox coords, confidence and class
    for i in range(len(detections)):

        # get bounding box coordinates
        xyxy_tensor = detections[i].xyxy.cpu()
        xyxy = xyxy_tensor.numpy().squeeze() # convert tensors to Numpy array
        xmin, ymin, xmax, ymax = xyxy.astype(int) # extract individual coordinates and convert to int

        # get bounding box class ID and name
        classidx = int(detections[i].cls.item())
        classname = labels[classidx]

        # get bounding box confidence
        conf = detections[i].conf.item()


        if conf > min_thresh:

            # draw rectangle box
            colour = bbox_colours[classidx % 10]
            cv.rectangle(frame, (xmin,ymin), (xmax,ymax), colour, 2)

            # draw label and confidence
            label = f'{classname}: {int(conf*100)}%'
            labelSize, baseLine = cv.getTextSize(label, cv.FONT_HERSHEY_SIMPLEX, 0.5, 1) # get font size
            label_ymin = max(ymin, labelSize[1] + 10) # buffer
            cv.rectangle(frame, (xmin, label_ymin-labelSize[1]-10), (xmin+labelSize[0], label_ymin+baseLine-10), colour, cv.FILLED) # draw white box to put label text in
            cv.putText(frame, label, (xmin, label_ymin-7), cv.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1) # draw label text

            detections_count = detections_count + 1

    # draw framerate and resolution
    cv.putText(frame, f'FPS: {avg_frame_rate:0.2f}', (10,20), cv.FONT_HERSHEY_SIMPLEX, .7, (0,255,255), 2) # draw framerate
    cv.putText(frame, f'Resolution: {frame.shape[1]}x{frame.shape[0]}', (10,40), cv.FONT_HERSHEY_SIMPLEX, .7, (0,255,255), 2) # draw resolution
    
    # draw detection results
    cv.putText(frame, f'Number of cards: {detections_count}', (10,60), cv.FONT_HERSHEY_SIMPLEX, .7, (0,255,255), 2) # draw total number of detected cards
    cv.imshow('YOLO detection results',frame) # display frame
    cv.waitKey(1)

    if record: 
        recorder.write(frame)

    return [xmin, ymin, xmax, ymax] if detections_count > 0 else None




class VisionNode(Node):
    def __init__(self, model, labels, resize, resW, resH, record, recorder, detector, bbox_colours, min_thresh, alpha, card_centres, smoothed_detections, smoothed_markers):
        super().__init__('vision_node')

        
        self.model_ = model # YOLO model
        self.labels_ = labels # classs labels
        self.resize_ = resize # bool for whether to resize frames before inference
        self.resW_ = resW # width to resize frames to for inference (if resizing)
        self.resH_ = resH # height to resize frames to for inference (if resizing)
        self.record_ = record # bool for whether to record inference results
        self.recorder_ = recorder # video recorder object (if recording)
        self.bbox_colours_ = bbox_colours # colours to use for bounding boxes of different classes
        self.min_thresh_ = min_thresh # min confidence threshold for detections 
        self.alpha_ = alpha # EMA factor
        self.smoothed_detections_ = smoothed_detections # 
        self.avg_frame_rate_ = 0 # initialise avg frame rate
        self.frame_rate_buffer_ = [] # buffer to hold frame rate results for calculating avg frame rate
        self.fps_avg_len_ = 200 # num of frames to calculate average frame rate over


        self.image_sub_ = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.frame_callback,
            10
        )

        self.depth_sub_ = self.create_subscription(
            Image,
            '/camera/depth/image_raw',
            self.frame_callback,
            10
        )

        self.publisher_ = self.create_publisher(
            Point,
            '/coords',
            10
        )

        self.bridge_ = CvBridge()

        self.busy_ = False # flag to prevent multiple simultaneous inference loops


    def publish_coords(self, coords):
        msg = Point()
        msg.x = coords[0]
        msg.y = coords[1]
        self.publisher_.publish(msg)
        self.get_logger().info(f'Published card coordinates: {msg.x}, {msg.y}')
    
    def frame_callback(self, msg):
        if self.busy_:
            return


        try:
            self.busy_ = True
            frame = self.bridge_.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            # start timer
            t_start = time.perf_counter()
            # run inference and get coords
            coords = inference(frame, self.model_, self.labels_, self.resize_, self.resW_, self.resH_, self.record_, self.recorder_, self.detector_, self.bbox_colours_, self.min_thresh_, self.alpha_, self.card_centres_, self.smoothed_detections_, self.smoothed_markers_, self.avg_frame_rate_)
            
            # calculate fps for this frame
            t_stop = time.perf_counter()
            frame_rate_calc = float(1/(t_stop - t_start))
            
            # append fps result to frame_rate_buffer (for finding average fps over multiple frames)
            if len(self.frame_rate_buffer_) >= self.fps_avg_len_:
                self.frame_rate_buffer_.pop(0)
                self.frame_rate_buffer_.append(frame_rate_calc)
            else:
                self.frame_rate_buffer_.append(frame_rate_calc)

            # mean fps
            self.avg_frame_rate_ = np.mean(self.frame_rate_buffer_)

            if coords is not None:
                self.publish_coords(coords)
    
        finally:
            self.busy_ = False

def main():

    # parse user inputs
    model_path = "yolo26n.pt"
    min_thresh = 0.5
    user_res = None
    record = False


    model = setup_model(model_path)
    labels = model.names


    smoothed_detections = {}
    smoothed_detections = {cls: None for cls in labels.values()}

    alpha = 0.3

    # parse resolution
    resize = False
    if user_res:
        resize = True
        resW, resH = int(user_res.split('x')[0]), int(user_res.split('x')[1])
    else:
        resW = resH = None

    recorder = setup_recording(record, resW, resH) if record else None

    # set bounding box colours
    bbox_colours = [(164,120,87), (68,148,228), (93,97,209), (178,182,133), (88,159,106), 
                (96,202,231), (159,124,168), (169,162,241), (98,118,150), (172,176,184)]
    
    rclpy.init()
    vision_node = VisionNode(model, labels, resize, resW, resH, record, recorder, detector, bbox_colours, min_thresh, alpha, card_centres, smoothed_detections, smoothed_markers)
    rclpy.spin(vision_node)
    vision_node.destroy_node()
    rclpy.shutdown()

    print(f'Average pipeline FPS: {vision_node.avg_frame_rate_:.2f}')
    cv.destroyAllWindows()


if __name__ == "__main__":
    main()
