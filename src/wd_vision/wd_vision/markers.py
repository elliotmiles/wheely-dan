import os
from ament_index_python.packages import get_package_share_directory

import sys
import glob
import time

import cv2 as cv
import numpy as np
from ultralytics import YOLO

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

from message_filters import Subscriber, ApproximateTimeSynchronizer
from sensor_msgs.msg import CameraInfo
from visualization_msgs.msg import Marker

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


def inference(
        frame, 
        model, 
        labels, 
        resize, 
        resW, 
        resH, 
        record, 
        recorder, 
        detector, 
        bbox_colours, 
        min_thresh, 
        alpha, 
        box_centres, 
        smoothed_centres, 
        smoothed_markers, 
        avg_frame_rate
    ):


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

    current_frame_detections = {}

    detected_objects = []

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

        # raw centre coords of the card
        centre = (int((xmax + xmin) / 2), int((ymax + ymin) / 2))

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

            # draw circle at centre of card
            radius = max(5, int(min(xmax - xmin, ymax - ymin) / 4)) # if card is small then rad=5
            cv.circle(frame, centre, radius, colour, -1)

            # apply EMA to smooth card centre over frames
            smoothed_centres[classname] = ema(smoothed_centres[classname], centre, alpha)
            current_frame_detections[classname] = smoothed_centres[classname]

            box_centres[classname] = centre
            detections_count = detections_count + 1

            detected_objects.append({
                'class': classname,
                'centre': centre,
                'bbox': (xmin, ymin, xmax, ymax),
                'confidence': conf,
            })

    box_centres.clear()
    box_centres.update(current_frame_detections)

    #----- ARUCO MARKERS -----

    # get a greyscale version of the frame and extract corners and ids of detected aruco markers
    grey = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
    corners, ids, rejected = detector.detectMarkers(grey)
    
    if ids is not None:
        # draw detected markers on the original frame
        cv.aruco.drawDetectedMarkers(frame, corners, ids)

        marker_centres = {}
        
        for i, corner in enumerate(corners):

            # reshape array (4 rows, 2 columns)
            pts = corner.reshape((4, 2))
            # [[x1, y1],
            #  [x2, y2],
            #  [x3, y3],
            #  [x4, y4]]

            # mean of the 4 corner points is the centre of the aruco marker
            centre_x = int(pts[:, 0].mean())
            centre_y = int(pts[:, 1].mean())
            raw_centre = (centre_x, centre_y)
            
            marker_id = int(ids[i][0])

            smoothed_markers[marker_id] = ema(smoothed_markers.get(marker_id), raw_centre, alpha)

            # add marker centres to dict
            marker_centres[marker_id] = smoothed_markers[marker_id]


            # draw circle at centre of aruco marker
            cv.circle(frame, (centre_x, centre_y), 15, (0, 0, 255), -1)


                            
    # draw framerate and resolution
    cv.putText(frame, f'FPS: {avg_frame_rate:0.2f}', (10,20), cv.FONT_HERSHEY_SIMPLEX, .7, (0,255,255), 2) # draw framerate
    cv.putText(frame, f'Resolution: {frame.shape[1]}x{frame.shape[0]}', (10,40), cv.FONT_HERSHEY_SIMPLEX, .7, (0,255,255), 2) # draw resolution
    
    # draw detection results
    cv.putText(frame, f'Number of detections: {detections_count}', (10,60), cv.FONT_HERSHEY_SIMPLEX, .7, (0,255,255), 2) # draw total number of detections
    cv.imshow('YOLO detection results',frame) # display frame
    cv.waitKey(1)

    if record: 
        recorder.write(frame)

    return detected_objects





class VisionNode(Node):
    def __init__(self, model, labels, resize, 
                 resW, resH, record, recorder, 
                 detector, bbox_colours, min_thresh, alpha, 
                 box_centres, smoothed_centres, smoothed_markers,
                 fx, fy, cx, cy):
        super().__init__('vision_node')

        
        self.model_ = model # YOLO model
        self.labels_ = labels # classs labels
        self.resize_ = resize # bool for whether to resize frames before inference
        self.resW_ = resW # width to resize frames to for inference (if resizing)
        self.resH_ = resH # height to resize frames to for inference (if resizing)
        self.record_ = record # bool for whether to record inference results
        self.recorder_ = recorder # video recorder object (if recording)
        self.detector_ = detector # aruco marker detector object
        self.bbox_colours_ = bbox_colours # colours to use for bounding boxes of different classes
        self.min_thresh_ = min_thresh # min confidence threshold for detections 
        self.alpha_ = alpha # EMA factor
        self.box_centres_ = box_centres # dict that holds class:centre, and updates every frame with latest smoothed centre
        self.smoothed_centres_ = smoothed_centres # 
        self.smoothed_markers_ = smoothed_markers #
        self.avg_frame_rate_ = 0 # initialise avg frame rate
        self.frame_rate_buffer_ = [] # buffer to hold frame rate results for calculating avg frame rate
        self.fps_avg_len_ = 200 # num of frames to calculate average frame rate over
        self.fx_ = None # depth camera intrinsics
        self.fy_ = None
        self.cx_ = None
        self.cy_ = None     
        self.markers_scale_ = 0.08 # scale of the published markers in RViz 

        self.rgb_sub_ = Subscriber(
            self,
            Image,
            '/camera/camera/color/image_raw'
        )

        self.depth_sub_ = Subscriber(
            self,
            Image,
            '/camera/camera/aligned_depth_to_color/image_raw'
        )

        # sync RGB + depth
        self.ts_ = ApproximateTimeSynchronizer(
            [self.rgb_sub_, self.depth_sub_],
            queue_size=10,
            slop=0.05
        )

        self.ts_.registerCallback(self.synced_callback)



        self.camera_info_sub_ = self.create_subscription(
            CameraInfo,
            '/camera/camera/color/camera_info',
            self.camera_info_callback,
            10
        )

        self.publisher_ = self.create_publisher(
            Marker,
            '/obj_markers',
            10
        )

        self.bridge_ = CvBridge()

        self.busy_ = False # initialise flag to prevent multiple simultaneous inference loops


    def camera_info_callback(self, msg):
        self.fx_ = msg.k[0]
        self.fy_ = msg.k[4]
        self.cx_ = msg.k[2]
        self.cy_ = msg.k[5]

    def depth_projection(self, centre, depth_msg):
        
        if self.fx_ is None: 
            return None

        # convert depth message to OpenCV format
        depth_frame = self.bridge_.imgmsg_to_cv2(depth_msg, desired_encoding='passthrough')

        # coords of centre of bbox
        u, v = centre

        # height and width of depth frame
        h, w = depth_frame.shape

        # clamping bounds for 9x9 kernel
        x_min = max(0, u - 4)
        x_max = min(w, u + 5)

        y_min = max(0, v - 4)
        y_max = min(h, v + 5)

        kernel = depth_frame[y_min:y_max, x_min:x_max]

        # remove invalid depth values

        valid_depths = kernel[np.isfinite(kernel) & (kernel > 0)]

        if valid_depths.size == 0 or len(valid_depths) < 20:
            return None
        
        avg_depth = np.median(valid_depths) # using median since it's less affected by outliers

        if avg_depth > 10:
            avg_depth /= 1000.0

        # 3d projection using pinhole model
        x = (u - self.cx_) * avg_depth / self.fx_
        y = (v - self.cy_) * avg_depth / self.fy_
        z = avg_depth

        return (x, y, z) # marker position relative to camera in OPTICAL FRAME COORDS
    
    
    def publish_marker(self, coords, scale, marker_id):
        
        x, y, z = coords
        
        msg = Marker()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "camera_color_optical_frame" # set the frame ID to the optical frame (obtained from /tf_static published by camera driver node)
        msg.type = Marker.SPHERE

        msg.ns = "detections"
        msg.id = marker_id
        msg.action = Marker.ADD

        msg.scale.x = scale
        msg.scale.y = scale
        msg.scale.z = scale

        msg.color.a = 1.0
        msg.color.r = 1.0
        msg.color.g = 0.0
        msg.color.b = 0.0

        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.position.z = z

        self.publisher_.publish(msg)
        self.get_logger().info(f'Published marker: {msg.pose.position.x}, {msg.pose.position.y}')

    
    def synced_callback(self, rgb_msg, depth_msg):
        if self.busy_:
            return


        try:
            self.busy_ = True
            frame = self.bridge_.imgmsg_to_cv2(rgb_msg, desired_encoding='bgr8')

            # start timer
            t_start = time.perf_counter()

            # run inference and get coords
            detections = inference(
                frame, 
                self.model_, 
                self.labels_, 
                self.resize_, 
                self.resW_, 
                self.resH_, 
                self.record_, 
                self.recorder_, 
                self.detector_, 
                self.bbox_colours_, 
                self.min_thresh_, 
                self.alpha_, 
                self.box_centres_, 
                self.smoothed_centres_, 
                self.smoothed_markers_, 
                self.avg_frame_rate_
            )

            
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

            for i, det in enumerate(detections):
                centre = det['centre']
                marker_pos = self.depth_projection(centre, depth_msg)

                if marker_pos is not None:
                    self.publish_marker(marker_pos, self.markers_scale_, i)

                self.get_logger().info(
                    f"3D point: {marker_pos}"
                )
        
        finally:
            self.busy_ = False

def main():
    box_centres = {}
    smoothed_markers = {}

    model_path = os.path.join(get_package_share_directory('wd_vision'), 'models', 'yolo26n.pt') # default model path
    min_thresh = 0.5
    user_res = None
    record = False


    model = setup_model(model_path)
    labels = model.names


    smoothed_centres = {}
    smoothed_centres = {cls: None for cls in labels.values()}

    # ARUCO MARKERS 
    aruco_dict = cv.aruco.getPredefinedDictionary(cv.aruco.DICT_4X4_50)
    parameters = cv.aruco.DetectorParameters()
    parameters.cornerRefinementMethod = cv.aruco.CORNER_REFINE_SUBPIX
    detector = cv.aruco.ArucoDetector(aruco_dict, parameters)

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

    vision_node = VisionNode(
        model, 
        labels, 
        resize, 
        resW, 
        resH, 
        record, 
        recorder, 
        detector, 
        bbox_colours, 
        min_thresh, 
        alpha, 
        box_centres, 
        smoothed_centres, 
        smoothed_markers
    )

    rclpy.spin(vision_node)
    vision_node.destroy_node()
    rclpy.shutdown()

    print(f'Average pipeline FPS: {vision_node.avg_frame_rate_:.2f}')
    cv.destroyAllWindows()


if __name__ == "__main__":
    main()