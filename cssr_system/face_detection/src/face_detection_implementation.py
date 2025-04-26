""""
face_detection_implementation.py Implementation code for running the Face and Mutual Gaze Detection and Localization ROS node.

Author: Yohannes Tadesse Haile
Date: April 18, 2025
Version: v1.0

Copyright (C) 2023 CSSR4Africa Consortium

This project is funded by the African Engineering and Technology Network (Afretec)
Inclusive Digital Transformation Research Grant Programme.

Website: www.cssr4africa.org

This program comes with ABSOLUTELY NO WARRANTY.
"""

import cv2
import mediapipe as mp
import numpy as np
import rospy
import rospkg
import os
import onnxruntime
import multiprocessing
import json
import random
import threading
from math import cos, sin, pi
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge, CvBridgeError
from message_filters import ApproximateTimeSynchronizer, Subscriber
from geometry_msgs.msg import Point
from typing import Tuple, List
from cssr_system.msg import face_detection_msg_file
from face_detection_tracking import Sort, CentroidTracker

class FaceDetectionNode:
    def __init__(self):     
        self.pub_gaze = rospy.Publisher("/faceDetection/data", face_detection_msg_file, queue_size=10)
        self.bridge = CvBridge()
        self.depth_image = None  # Initialize depth_image
        self.color_image = None  # Initialize color_image
        self.use_compressed = rospy.get_param('/faceDetection_config/useCompressed', False)  # Parameter to choose compressed or raw images
        self.camera_type = rospy.get_param('/faceDetection/camera', default="realsense")  # Default camera type
        self.node_name = rospy.get_name().lstrip('/')
        self.timer = rospy.get_time()
        self.verbose_mode = rospy.get_param("/faceDetection_config/verboseMode", False)

        self.last_image_time = rospy.get_time()
        self.image_timeout = rospy.get_param("/faceDetection_config/imageTimeout", 2.0)

    def subscribe_topics(self):
        # Set up for indefinite waiting
        wait_rate = rospy.Rate(1)  # Check once per second
        start_time = rospy.get_time()
        
        if self.camera_type == "realsense":
            self.rgb_topic = self.extract_topics("RealSenseCameraRGB")
            self.depth_topic = self.extract_topics("RealSenseCameraDepth")
        
        elif self.camera_type == "pepper":
            self.rgb_topic = self.extract_topics("PepperFrontCamera")
            self.depth_topic = self.extract_topics("PepperDepthCamera")
        
        elif self.camera_type == "video":
            self.rgb_topic = self.extract_topics("RealSenseCameraRGB")
            self.depth_topic = self.extract_topics("RealSenseCameraDepth") 
        
        else:
            rospy.logerr(f"{self.node_name}: Invalid camera type specified")
            rospy.logerr(f"{self.node_name}: Invalid camera type")
            return
        
        # Check if topics were found
        if not self.rgb_topic or not self.depth_topic:
            rospy.logerr(f"{self.node_name}: Camera topic not found.")
            rospy.logerr(f"{self.node_name}: Camera topic not found")
            return

        # Determine topic names based on compression settings
        if self.use_compressed and self.camera_type == "realsense":
            color_topic = self.rgb_topic + "/compressed"
            depth_topic = self.depth_topic + "/compressedDepth"
        
        elif self.use_compressed and self.camera_type == "pepper":
            # There is no compressed topic for Pepper cameras
            rospy.logwarn(f"{self.node_name}: Compressed images are not available for Pepper cameras.")
            color_topic = self.rgb_topic
            depth_topic = self.depth_topic
        
        elif self.camera_type == "video":
            color_topic = self.rgb_topic + "/compressed"
            depth_topic = self.depth_topic + "/compressedDepth"
        
        else:
            color_topic = self.rgb_topic
            depth_topic = self.depth_topic
        
        # Wait for topics to be available, with indefinite waiting
        rospy.loginfo(f"{self.node_name}: Waiting for topics: {color_topic}, {depth_topic}")
        topics_available = False
        warning_interval = 5.0  # Warn every 5 seconds
        last_warning_time = start_time
        
        while not topics_available and not rospy.is_shutdown():
            published_topics = dict(rospy.get_published_topics())
            
            color_available = color_topic in published_topics
            depth_available = depth_topic in published_topics
            
            if color_available and depth_available:
                topics_available = True
                if self.verbose_mode:
                    rospy.loginfo(f"{self.node_name}: Both topics are available!")
                break
            
            # Generate warning messages periodically
            current_time = rospy.get_time()
            elapsed_time = current_time - start_time
            
            if current_time - last_warning_time >= warning_interval:
                missing_topics = []
                if not color_available:
                    missing_topics.append(color_topic)
                if not depth_available:
                    missing_topics.append(depth_topic)
                    
                rospy.logwarn(f"{self.node_name}: Still waiting for topics after {int(elapsed_time)}s: {', '.join(missing_topics)}")
                last_warning_time = current_time
                
            wait_rate.sleep()
        
        # Subscribe to topics
        if self.use_compressed and self.camera_type == "realsense":
            color_sub = Subscriber(color_topic, CompressedImage)
            depth_sub = Subscriber(depth_topic, CompressedImage)
            rospy.loginfo(f"{self.node_name}: Subscribed to {color_topic}")
            rospy.loginfo(f"{self.node_name}: Subscribed to {depth_topic}")
        
        elif self.use_compressed and self.camera_type == "pepper":
            color_sub = Subscriber(color_topic, Image)
            depth_sub = Subscriber(depth_topic, Image)
            rospy.loginfo(f"{self.node_name}: Subscribed to {color_topic}")
            rospy.loginfo(f"{self.node_name}: Subscribed to {depth_topic}")

        elif self.camera_type == "video":
            color_sub = Subscriber(color_topic, CompressedImage)
            depth_sub = Subscriber(depth_topic, CompressedImage)
            rospy.loginfo(f"{self.node_name}: Subscribed to {color_topic}")
            rospy.loginfo(f"{self.node_name}: Subscribed to {depth_topic}")

        else:
            color_sub = Subscriber(color_topic, Image)
            depth_sub = Subscriber(depth_topic, Image)
            rospy.loginfo(f"{self.node_name}: Subscribed to {color_topic}")
            rospy.loginfo(f"{self.node_name}: Subscribed to {depth_topic}")

        # ApproximateTimeSynchronizer setup
        if self.camera_type == "pepper":
            ats = ApproximateTimeSynchronizer([color_sub, depth_sub], queue_size=10, slop=5)
        else:
            ats = ApproximateTimeSynchronizer([color_sub, depth_sub], queue_size=10, slop=0.1)  
        
        ats.registerCallback(self.synchronized_callback)

    def synchronized_callback(self, color_data, depth_data):
        self.last_image_time = rospy.get_time()
        try:
            # --- Color Image Processing ---
            if isinstance(color_data, CompressedImage):
                np_arr = np.frombuffer(color_data.data, np.uint8)
                self.color_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            else:
                self.color_image = self.bridge.imgmsg_to_cv2(color_data, desired_encoding="bgr8")

            # --- Depth Image Processing ---
            if isinstance(depth_data, CompressedImage):
                # Ensure depth_data.format is valid before accessing it
                if hasattr(depth_data, "format") and depth_data.format and "compressedDepth png" in depth_data.format:
                    try:
                        # Handle PNG compression in compressedDepth format
                        depth_header_size = 12
                        depth_img_data = depth_data.data[depth_header_size:]
                        np_arr = np.frombuffer(depth_img_data, np.uint8)
                        depth_img = cv2.imdecode(np_arr, cv2.IMREAD_ANYDEPTH)

                        if depth_img is not None:
                            self.depth_image = depth_img
                        else:
                            rospy.logerr(f"{self.node_name}: Failed to decode PNG depth image")
                    except Exception as e:
                        rospy.logerr(f"{self.node_name}: Depth decoding error: {str(e)}")
                else:
                    # Regular compressed image
                    np_arr = np.frombuffer(depth_data.data, np.uint8)
                    self.depth_image = cv2.imdecode(np_arr, cv2.IMREAD_UNCHANGED)
            else:
                self.depth_image = self.bridge.imgmsg_to_cv2(depth_data, desired_encoding="passthrough")

            if self.color_image is None or self.depth_image is None:
                rospy.logwarn(f"{self.node_name}: synchronized_callback: Decoded images are None.")
                return

            # Process synchronized images
            self.process_images()
            self.last_image_time = rospy.get_time()

        except CvBridgeError as e:
            rospy.logerr(f"{self.node_name}: synchronized_callback CvBridge Error: {str(e)}")
        except Exception as e:
            rospy.logerr(f"{self.node_name}: synchronized_callback Exception: {str(e)}")

    def start_timeout_monitor(self):
        def monitor():
            rate = rospy.Rate(1)
            while not rospy.is_shutdown():
                time_since_last = rospy.get_time() - self.last_image_time
                if time_since_last > self.image_timeout and self.color_image is not None:
                    rospy.logwarn(f"{self.node_name}: No image received for {self.image_timeout} seconds.Shutting down.")
                    rospy.signal_shutdown("No image data.")
                rate.sleep()

        threading.Thread(target=monitor, daemon=True).start()

    def check_camera_resolution(self, color_image, depth_image):
        """Check if the color and depth images have the same resolution."""
        if color_image is None or depth_image is None:
            rospy.logwarn(f"{self.node_name}: check_camera_resolution: One of the images is None")
            return False
        rgb_h, rgb_w = color_image.shape[:2]
        depth_h, depth_w = depth_image.shape[:2]
        return rgb_h == depth_h and rgb_w == depth_w

    def read_json_file(package_name):
        """
        Read and parse a JSON configuration file from the specified ROS package.
        
        Args:
            package_name (str): Name of the ROS package containing the config file
            
        Returns:
            dict: Configuration data from JSON file, or empty dict if file not found
        """
        node_name = rospy.get_name()
        rospack = rospkg.RosPack()
        try:
            package_path = rospack.get_path(package_name)
            
            # Determine the directory and file name based on the package name
            if package_name == 'unit_tests':
                directory = 'face_detection_test/config'
                config_file = 'face_detection_test_configuration.json'
            else:
                directory = 'face_detection/config'
                config_file = 'face_detection_configuration.json'
            
            config_path = os.path.join(package_path, directory, config_file)
            
            if os.path.exists(config_path):
                with open(config_path, 'r') as file:
                    data = json.load(file)
                    return data
            else:
                rospy.logerr(f"{node_name}: read_json_file: Configuration file not found at {config_path}")
                return {}
                
        except rospkg.ResourceNotFound as e:
            rospy.logerr(f"{node_name}: ROS package '{package_name}' not found: {e}")
            return {}
        except json.JSONDecodeError as e:
            rospy.logerr(f"{node_name}: Error parsing JSON configuration file: {e}")
            return {}
        except Exception as e:
            rospy.logerr(f"{node_name}: Unexpected error reading configuration file: {e}")
            return {}
    
    @staticmethod
    def extract_topics(image_topic):
        node_name = rospy.get_name()
        rospack = rospkg.RosPack()
        try:
            package_path = rospack.get_path('cssr_system')
            config_path = os.path.join(package_path, 'face_detection/data', 'pepper_topics.dat')

            if os.path.exists(config_path):
                with open(config_path, 'r') as file:
                    for line in file:
                        line = line.strip()
                        if not line or line.startswith('#'):
                            continue
                        key, value = line.split(maxsplit=1)
                        if key.lower() == image_topic.lower():
                            return value
            else:
                rospy.logerr(f"{node_name}: extract_topics: Data file not found at {config_path}")
        except rospkg.ResourceNotFound as e:
            rospy.logerr(f"{node_name}: ROS package 'cssr_system' not found: {e}")
        
    def process_images(self):
        if self.color_image is None or self.depth_image is None:
            rospy.logwarn(f"{self.node_name}: process_images: Missing images.")
            return

        if not self.check_camera_resolution(self.color_image, self.depth_image) and self.camera_type != "pepper":
            rospy.logwarn(f"{self.node_name}: process_images: Color and depth image resolutions do not match.")
            pass

        if hasattr(self, 'face_mesh'):  # MediaPipe implementation
            rgb_frame = cv2.cvtColor(self.color_image, cv2.COLOR_BGR2RGB)
            img_h, img_w = self.color_image.shape[:2]
            self.process_face_mesh(self.color_image, rgb_frame, img_h, img_w)

        elif hasattr(self, 'yolo_model'):  # SixDrepNet implementation
            self.latest_frame = self.process_frame(self.color_image)
        else:
            rospy.logwarn(f"{self.node_name}: process_images: No processing method found (face_mesh/yolo_model missing).")

    def display_depth_image(self):
        if self.depth_image is not None:
            try:
                # Convert depth image to float32 for processing
                depth_array = np.array(self.depth_image, dtype=np.float32)

                # Handle invalid depth values (e.g., NaNs, infs)
                depth_array = np.nan_to_num(depth_array, nan=0.0, posinf=0.0, neginf=0.0)

                # Normalize the depth image to the 0-255 range
                normalized_depth = cv2.normalize(depth_array, None, 0, 255, cv2.NORM_MINMAX)

                # Convert to 8-bit image
                normalized_depth = np.uint8(normalized_depth)

                # Apply a colormap for better visualization (optional)
                depth_colormap = cv2.applyColorMap(normalized_depth, cv2.COLORMAP_JET)

                # Display the depth image
                cv2.imshow("Depth Image", depth_colormap)

            except Exception as e:
                rospy.logerr(f"{self.node_name}: {self.node_name}: display_depth_image: Error displaying depth image: {e}")

    def get_depth_at_centroid(self, centroid_x, centroid_y):
        """Get the depth value at the centroid of a face."""
        if self.depth_image is None:
            return None

        height, width = self.depth_image.shape[:2]
        x = int(round(centroid_x))
        y = int(round(centroid_y))

        # Check bounds
        if x < 0 or x >= width or y < 0 or y >= height:
            rospy.logwarn(f"{self.node_name}: Centroid coordinates ({x}, {y}) are out of bounds.")
            return None

        depth_value = self.depth_image[y, x]

        # Handle invalid depth values
        if np.isfinite(depth_value) and depth_value > 0:
            # Convert to meters if necessary
            depth_in_meters = depth_value / 1000.0
            return depth_in_meters
        else:
            return None
        
    def get_depth_in_region(self, centroid_x, centroid_y, box_width, box_height, region_scale=0.1):
        """
        Get the depth value within a scaled region around the centroid of a bounding box.
        Uses median filtering to be more robust against compression artifacts.
        """
        if self.depth_image is None:
            return None

        # Calculate scaled region dimensions
        region_width = int(box_width * region_scale)
        region_height = int(box_height * region_scale)
        
        # Ensure minimum region size (e.g., 5x5 pixels)
        region_width = max(5, region_width)
        region_height = max(5, region_height)

        # Calculate the top-left corner of the scaled region
        x_start = int(round(centroid_x - region_width / 2))
        y_start = int(round(centroid_y - region_height / 2))

        # Calculate the bottom-right corner of the scaled region
        x_end = x_start + region_width
        y_end = y_start + region_height

        # Get image dimensions
        image_height, image_width = self.depth_image.shape[:2]

        # Ensure the region is within bounds
        x_start = max(0, x_start)
        y_start = max(0, y_start)
        x_end = min(image_width, x_end)
        y_end = min(image_height, y_end)

        # If the region is invalid (e.g., zero area), return None
        if x_start >= x_end or y_start >= y_end:
            rospy.logwarn(f"{self.node_name}: Invalid region coordinates ({x_start}, {y_start}, {x_end}, {y_end}).")
            return None

        # Extract the region of interest
        depth_roi = self.depth_image[y_start:y_end, x_start:x_end]

        # Filter out invalid depth values (e.g., zeros or NaNs)
        valid_depth_values = depth_roi[np.isfinite(depth_roi) & (depth_roi > 0)]

        if valid_depth_values.size > 0:
            # Use median instead of mean for more robustness against artifacts
            median_depth_in_meters = np.median(valid_depth_values) / 1000.0
            return median_depth_in_meters
        else:
            return None
        
    def generate_dark_color(self):
        """Generate a dark color that is visible on a white background."""
        while True:
            color = (random.randint(0, 150), random.randint(0, 150), random.randint(0, 150))  # Dark colors (0-150)
            brightness = (0.299 * color[0] + 0.587 * color[1] + 0.114 * color[2])  # Perceived brightness
            if brightness < 130:  # Ensure the color is dark enough
                return color

    def publish_face_detection(self, tracking_data):
        """Publish the face detection results."""
        if not tracking_data:
            # Don't publish empty messages
            return
            
        face_msg = face_detection_msg_file()

        # Initialize lists for each attribute in the message
        face_msg.face_label_id = [data['face_id'] for data in tracking_data]
        face_msg.centroids = [data['centroid'] for data in tracking_data]
        face_msg.width = [data['width'] for data in tracking_data]  # Add width data
        face_msg.height = [data['height'] for data in tracking_data]  # Add height data
        face_msg.mutualGaze = [data['mutual_gaze'] for data in tracking_data]

        # Publish the message
        self.pub_gaze.publish(face_msg)
class MediaPipe(FaceDetectionNode):
    def __init__(self):
        
        super().__init__()
        # Initialize MediaPipe components
        self.mp_face_mesh = mp.solutions.face_mesh
        self.face_mesh = self.mp_face_mesh.FaceMesh(rospy.get_param("/faceDetection_config/mpFacedetConfidence", 0.5), max_num_faces=10)

        self.mp_face_detection = mp.solutions.face_detection
        self.face_detection = self.mp_face_detection.FaceDetection(model_selection=1, min_detection_confidence=0.5)
        
        self.mp_drawing = mp.solutions.drawing_utils
        self.drawing_spec = self.mp_drawing.DrawingSpec(color=(128, 128, 128), thickness=1, circle_radius=1)

        # Initialize the CentroidTracker
        self.centroid_tracker = CentroidTracker(rospy.get_param("/faceDetection_config/centroidMaxDisappeared", 15), rospy.get_param("/faceDetection_config/centroidMaxDistance", 100))
        self.latest_frame = None

        # Subscribe to the image topic
        self.subscribe_topics()

        # check if the depth camera and color camera have the same resolution.
        if self.depth_image is not None:
            if not self.check_camera_resolution(self.color_image, self.depth_image) and self.camera_type != "pepper":
                rospy.logerr(f"{self.node_name}: Color camera and depth camera have different resolutions.")
                rospy.signal_shutdown(f"{self.node_name}: Resolution mismatch")

        self.start_timeout_monitor()

    def spin(self):
        """Main loop to display processed frames and depth images."""
        rate = rospy.Rate(30)  # Adjust the rate as needed
        while not rospy.is_shutdown():
            if self.latest_frame is not None:
                if self.verbose_mode:
                    # Display the processed frame
                    cv2.imshow("Face Detection & Mutual Gaze Estimation", self.latest_frame)

            if rospy.get_time() - self.timer > 10:
                rospy.loginfo(f"{self.node_name}: running.")
                self.timer = rospy.get_time()

            # Display the depth image if verbose mode is enabled
            if self.verbose_mode:
                self.display_depth_image()

            # Wait for GUI events
            if cv2.waitKey(1) & 0xFF == ord("q"):
                rospy.signal_shutdown("User requested shutdown")

            rate.sleep()

        # Clean up OpenCV windows on shutdown
        cv2.destroyAllWindows()

    def process_face_mesh(self, frame, rgb_frame, img_h, img_w):
        results = self.face_mesh.process(rgb_frame)
        centroids = []
        mutualGaze_list = []
        face_widths = []
        face_heights = []
        face_boxes = []  # Store bounding boxes for each face
        tracking_data = []  # Initialize tracking_data here
        
        # Create a copy of the frame to draw on
        display_frame = frame.copy()
        
        # Dictionary to store face ID colors
        if not hasattr(self, "face_colors"):
            self.face_colors = {}
            
        if results.multi_face_landmarks:
            for face_id, face_landmarks in enumerate(results.multi_face_landmarks):
                face_2d, face_3d = [], []
                x_min, y_min, x_max, y_max = img_w, img_h, 0, 0  # Bounding box coordinates
                
                for idx, lm in enumerate(face_landmarks.landmark):
                    x, y = int(lm.x * img_w), int(lm.y * img_h)
                    face_2d.append([x, y])
                    face_3d.append([x, y, lm.z])
                    # Expand bounding box
                    x_min = min(x_min, x)
                    y_min = min(y_min, y)
                    x_max = max(x_max, x)
                    y_max = max(y_max, y)
                
                # Calculate width and height
                width = x_max - x_min
                height = y_max - y_min
                
                # Store bounding box
                face_boxes.append((x_min, y_min, x_max, y_max))
                face_widths.append(width)
                face_heights.append(height)
                
                centroid_x = np.mean([pt[0] for pt in face_2d])
                centroid_y = np.mean([pt[1] for pt in face_2d])
                centroids.append((centroid_x, centroid_y))
                
                face_2d = np.array(face_2d, dtype=np.float64)
                face_3d = np.array(face_3d, dtype=np.float64)
                
                focal_length = 1 * img_w
                cam_matrix = np.array([[focal_length, 0, img_w / 2],
                                    [0, focal_length, img_h / 2],
                                    [0, 0, 1]])
                distortion_matrix = np.zeros((4, 1), dtype=np.float64)
                
                success, rotation_vec, translation_vec = cv2.solvePnP(
                    face_3d, face_2d, cam_matrix, distortion_matrix)
                
                rmat, jac = cv2.Rodrigues(rotation_vec)
                angles, mtxR, mtxQ, Qx, Qy, Qz = cv2.RQDecomp3x3(rmat)
                
                x_angle = angles[0] * 360
                y_angle = angles[1] * 360
                
                mp_angle = rospy.get_param("/faceDetection_config/mpHeadposeAngle", 8)
                mutualGaze = abs(x_angle) <= mp_angle and abs(y_angle) <= mp_angle
                mutualGaze_list.append(mutualGaze)
            
            # Use the centroid tracker to match centroids with object IDs
            centroid_to_face_id = self.centroid_tracker.match_centroids(centroids)
            
            for idx, (centroid, width, height, box) in enumerate(zip(centroids, face_widths, face_heights, face_boxes)):
                centroid_tuple = tuple(centroid)
                face_id = centroid_to_face_id.get(centroid_tuple, None)
                
                # Assign a new dark color for a new face or lost tracking
                if face_id is None or face_id not in self.face_colors:
                    self.face_colors[face_id] = self.generate_dark_color()
                
                face_color = self.face_colors[face_id]
                cz = self.get_depth_at_centroid(centroid[0], centroid[1])
                cz = cz if cz is not None else 0.0  # Default to 0.0 meters
                
                point = Point(x=float(centroid[0]), y=float(centroid[1]), z=float(cz) if cz else 0.0)
                
                # Add width and height to tracking data
                tracking_data.append({
                    'face_id': str(face_id),
                    'centroid': point,
                    'width': float(width),
                    'height': float(height),
                    'mutual_gaze': bool(mutualGaze_list[idx])
                })
                
                # Unpack bounding box coordinates
                x_min, y_min, x_max, y_max = box
                
                # Draw bounding box with assigned color
                cv2.rectangle(display_frame, (x_min, y_min), (x_max, y_max), face_color, 2)
                
                # Add label above bounding box
                label = "Engaged" if mutualGaze_list[idx] else "Not Engaged"
                cv2.putText(display_frame, label, (x_min, y_min - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, face_color, 2)
                cv2.putText(display_frame, f"Face: {face_id}", (x_min, y_min - 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, face_color, 2)
                
                # Draw depth information below the box
                cv2.putText(display_frame, f"Depth: {cz:.2f}m", (x_min, y_max + 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, face_color, 2)
            
        # Save the processed frame for display in spin()
        self.latest_frame = display_frame
        
        # Publish the tracking data
        self.publish_face_detection(tracking_data)
class YOLOONNX:
    def __init__(self, model_path: str, class_score_th: float = 0.65,
        providers: List[str] = ['CUDAExecutionProvider', 'CPUExecutionProvider']):
        self.class_score_th = class_score_th
        session_option = onnxruntime.SessionOptions()
        session_option.log_severity_level = 3
        
        # Optimize ONNX Runtime session options
        session_option.intra_op_num_threads = multiprocessing.cpu_count()
        session_option.graph_optimization_level = onnxruntime.GraphOptimizationLevel.ORT_ENABLE_ALL
        self.onnx_session = onnxruntime.InferenceSession(
            model_path, sess_options=session_option, providers=providers)
        
        self.input_shape = self.onnx_session.get_inputs()[0].shape
        self.input_names = [inp.name for inp in self.onnx_session.get_inputs()]
        self.output_names = [out.name for out in self.onnx_session.get_outputs()]

    def __call__(self, image: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        resized_image = self.__preprocess(image)
        inference_image = resized_image[np.newaxis, ...].astype(np.float32)
        boxes = self.onnx_session.run(
            self.output_names,
            {name: inference_image for name in self.input_names},
        )[0]
        return self.__postprocess(image, boxes)

    def __preprocess(self, image: np.ndarray) -> np.ndarray:
        resized_image = cv2.resize(image, (self.input_shape[3], self.input_shape[2]))
        resized_image = resized_image[:, :, ::-1] / 255.0  # BGR to RGB and normalize
        return resized_image.transpose(2, 0, 1)  # HWC to CHW

    def __postprocess(self, image: np.ndarray, boxes: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        img_h, img_w = image.shape[:2]
        result_boxes = []
        result_scores = []
        if boxes.size > 0:
            scores = boxes[:, 6]
            keep_idxs = scores > self.class_score_th
            boxes_keep = boxes[keep_idxs]
            for box in boxes_keep:
                x_min = int(max(box[2], 0) * img_w / self.input_shape[3])
                y_min = int(max(box[3], 0) * img_h / self.input_shape[2])
                x_max = int(min(box[4], self.input_shape[3]) * img_w / self.input_shape[3])
                y_max = int(min(box[5], self.input_shape[2]) * img_h / self.input_shape[2])
                result_boxes.append([x_min, y_min, x_max, y_max])
                result_scores.append(box[6])
        return np.array(result_boxes), np.array(result_scores)

class SixDrepNet(FaceDetectionNode):
    def __init__(self):
        super().__init__()
        if self.verbose_mode:
            rospy.loginfo(f"{self.node_name}: Initializing SixDrepNet...")

        # Set up model paths
        yolo_model_path = rospkg.RosPack().get_path('cssr_system') + '/face_detection/models/face_detection_goldYOLO.onnx'
        sixdrepnet_model_path = rospkg.RosPack().get_path('cssr_system') + '/face_detection/models/face_detection_sixdrepnet360.onnx'
        
        self.latest_frame = None
        
        # Initialize YOLOONNX model early and check success
        try:
            self.yolo_model = YOLOONNX(model_path=yolo_model_path, class_score_th = rospy.get_param("/faceDetection_config/sixdrepnetConfidence", 0.65))
            if self.verbose_mode:
                rospy.loginfo(f"{self.node_name}: YOLOONNX model initialized successfully.")
        except Exception as e:
            self.yolo_model = None
            rospy.logerr(f"{self.node_name}: Failed to initialize YOLOONNX model: {e}")
            return  # Exit early if initialization fails

        # Initialize SixDrepNet ONNX session
        try:
            session_option = onnxruntime.SessionOptions()
            session_option.log_severity_level = 3
            session_option.intra_op_num_threads = multiprocessing.cpu_count()
            session_option.graph_optimization_level = onnxruntime.GraphOptimizationLevel.ORT_ENABLE_ALL
            self.sixdrepnet_session = onnxruntime.InferenceSession(
                sixdrepnet_model_path,
                sess_options=session_option,
                providers=['CUDAExecutionProvider', 'CPUExecutionProvider']
            )

            active_providers = self.sixdrepnet_session.get_providers()
            if self.verbose_mode:
                rospy.loginfo(f"{self.node_name}: Active providers: {active_providers}")
            if "CUDAExecutionProvider" not in active_providers:
                if self.verbose_mode:
                    rospy.logwarn(f"{self.node_name}: CUDAExecutionProvider is not available. Running on CPU may slow down inference.")
            else:
                if self.verbose_mode:
                    rospy.loginfo(f"{self.node_name}: CUDAExecutionProvider is active. Running on GPU for faster inference.")
        except Exception as e:
            rospy.logerr(f"{self.node_name}: Failed to initialize SixDrepNet ONNX session: {e}")
            return  # Exit early if initialization fails

        # Set up remaining attributes
        self.mean = np.array([0.485, 0.456, 0.406], dtype=np.float32)
        self.std = np.array([0.229, 0.224, 0.225], dtype=np.float32)

        self.sort_max_disappeared = rospy.get_param("/faceDetection_config/sortMaxDisappeared", 5)
        self.sort_min_hits = rospy.get_param("/faceDetection_config/sortMinHits", 3)
        self.sort_iou_threshold = rospy.get_param("/faceDetection_config/sortIOUThreshold", 0.3)

        self.sort_tracker = Sort(max_age=self.sort_max_disappeared, min_hits=self.sort_min_hits, iou_threshold=self.sort_iou_threshold)
        self.tracks = [] 
        
        if self.verbose_mode:
            rospy.loginfo(f"{self.node_name} SixDrepNet initialization complete.")

        self.subscribe_topics()

        # check if the depth camera and color camera have the same resolution.
        if self.depth_image is not None:
            if not self.check_camera_resolution(self.color_image, self.depth_image) and self.camera_type != "pepper":
                rospy.logerr(f"{self.node_name}: Color camera and depth camera have different resolutions.")
                rospy.signal_shutdown(f"{self.node_name}: Resolution mismatch")

        self.start_timeout_monitor()
    
    def draw_axis(self, img, yaw, pitch, roll, tdx=None, tdy=None, size=100):
        pitch = pitch * pi / 180
        yaw = -yaw * pi / 180
        roll = roll * pi / 180
        height, width = img.shape[:2]
        tdx = tdx if tdx is not None else width / 2
        tdy = tdy if tdy is not None else height / 2

        x1 = size * (cos(yaw) * cos(roll)) + tdx
        y1 = size * (cos(pitch) * sin(roll) + sin(pitch) * sin(yaw) * cos(roll)) + tdy
        x2 = size * (-cos(yaw) * sin(roll)) + tdx
        y2 = size * (cos(pitch) * cos(roll) - sin(pitch) * sin(yaw) * sin(roll)) + tdy
        x3 = size * sin(yaw) + tdx
        y3 = size * (-cos(yaw) * sin(pitch)) + tdy

        cv2.line(img, (int(tdx), int(tdy)), (int(x1), int(y1)), (0, 0, 255), 2)
        cv2.line(img, (int(tdx), int(tdy)), (int(x2), int(y2)), (0, 255, 0), 2)
        cv2.line(img, (int(tdx), int(tdy)), (int(x3), int(y3)), (255, 0, 0), 2)

    def process_frame(self, cv_image):
        """
        Process the input frame for face detection and head pose estimation using SORT.
        Args: 
            cv_image: Input frame as a NumPy array (BGR format)
        """
        debug_image = cv_image.copy()
        img_h, img_w = debug_image.shape[:2]
        tracking_data = []

        # Dictionary to store face ID colors
        if not hasattr(self, "face_colors"):
            self.face_colors = {}

        # Object detection (YOLO)
        boxes, scores = self.yolo_model(debug_image)

        # Prepare detections for SORT ([x1, y1, x2, y2, score])
        detections = []
        for box, score in zip(boxes, scores):
            x1, y1, x2, y2 = box
            detections.append([x1, y1, x2, y2, score])

        # Convert detections to NumPy array
        detections = np.array(detections)

        # Update SORT tracker with detections
        if detections.shape[0] > 0:
            self.tracks = self.sort_tracker.update(detections)
        else:
            self.tracks = []  # Reset tracks if no detections

        # Process tracks
        for track in self.tracks:
            x1, y1, x2, y2, face_id = map(int, track)  # SORT returns face_id as the last value
            cx, cy = (x1 + x2) // 2, (y1 + y2) // 2
            
            # Calculate width and height
            width = x2 - x1
            height = y2 - y1

            # Assign a unique color for each face ID
            # Assign a new dark color for a new face or lost tracking
            if face_id is None or face_id not in self.face_colors:
                self.face_colors[face_id] = self.generate_dark_color()

            face_color = self.face_colors[face_id]

            # Crop the face region for head pose estimation
            head_image = debug_image[max(y1, 0):min(y2, img_h), max(x1, 0):min(x2, img_w)]
            if head_image.size == 0:
                continue  # Skip if cropped region is invalid

            # Preprocess for SixDrepNet
            resized_image = cv2.resize(head_image, (224, 224))
            normalized_image = (resized_image[..., ::-1] / 255.0 - self.mean) / self.std
            input_tensor = normalized_image.transpose(2, 0, 1)[np.newaxis, ...].astype(np.float32)

            # Run head pose estimation
            yaw_pitch_roll = self.sixdrepnet_session.run(None, {'input': input_tensor})[0][0]
            yaw_deg, pitch_deg, roll_deg = yaw_pitch_roll

            # Draw head pose axes
            self.draw_axis(debug_image, yaw_deg, pitch_deg, roll_deg, cx, cy, size=100)

            cz = self.get_depth_in_region(cx, cy, width, height)
            cz = cz if cz is not None else 0.0

            # Determine if the person is engaged
            sixdrep_angle = rospy.get_param("/faceDetection_config/sixdrepnetHeadposeAngle", 10)
            mutual_gaze = abs(yaw_deg) < sixdrep_angle and abs(pitch_deg) < sixdrep_angle

            # Add width and height to tracking data
            tracking_data.append({
                'face_id': str(face_id),
                'centroid': Point(x=float(cx), y=float(cy), z=float(cz) if cz else 0.0),
                'width': float(width),
                'height': float(height),
                'mutual_gaze': mutual_gaze
            })

            # Draw bounding box with assigned color
            cv2.rectangle(debug_image, (x1, y1), (x2, y2), face_color, 2)

            # Add labels above bounding box
            label = "Engaged" if mutual_gaze else "Not Engaged"
            cv2.putText(debug_image, label, (x1 + 10, y1 - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, face_color, 2)

            cv2.putText(debug_image, f"Face: {face_id}", (x1 + 10, y1 - 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, face_color, 2)
            
               # Draw depth information below the box
            cv2.putText(debug_image, f"Depth: {cz:.2f}m", (x1 + 10, y2 + 20),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, face_color, 2)

        # Publish tracking data
        self.publish_face_detection(tracking_data)
        return debug_image
 
    def spin(self):
        """Main loop to display processed frames and depth images."""
        rate = rospy.Rate(30)  # Adjust the rate as needed
        while not rospy.is_shutdown():
            if rospy.get_time() - self.timer > 10:
                rospy.loginfo(f"{self.node_name}: running.")
                self.timer = rospy.get_time()

            if self.latest_frame is not None:
                if self.verbose_mode:
                    # Display the processed frame
                    cv2.imshow("Face Detection & Head Pose Estimation", self.latest_frame)

            # Display the depth image if verbose mode is enabled
            if self.verbose_mode:
                self.display_depth_image()

            # Wait for GUI events
            if cv2.waitKey(1) & 0xFF == ord("q"):
                rospy.signal_shutdown("User requested shutdown")

            rate.sleep()

        # Clean up OpenCV windows on shutdown
        cv2.destroyAllWindows()