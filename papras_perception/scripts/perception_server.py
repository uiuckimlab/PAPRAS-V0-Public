#!/usr/bin/env python3

import rospy
from std_msgs.msg import String, Bool
from geometry_msgs.msg import Pose
from sensor_msgs.msg import CameraInfo
import tf2_ros
from tf.transformations import quaternion_matrix
from papras_perception.srv import GetDetectedObjectPose, GetDetectedObjectPoseResponse

import cv2
import numpy as np
import pyrealsense2 as rs2

# pub /found_object when object first detected
# ros srv server to /papras_perception/object_detection to send detected object pose  


class ObjectDetector(object):
    def __init__(self):
        rospy.init_node('object_detector')

        self.init_camera_intrinsics()
        self.init_rs_pipeline()

        self.pub = rospy.Publisher('/found_object', Bool, queue_size=10)
        self.srv = rospy.Service('/papras_perception/GetDetectedObjectPose', GetDetectedObjectPose, self.handle_object_detection)
        self.tfBuffer = tf2_ros.Buffer()
        listener = tf2_ros.TransformListener(self.tfBuffer)
        
        self.object_detected = False
        self.object_pose = None

    def init_rs_pipeline(self):
        # set up realsense pipeline
        print('Initializing realsense pipeline...')
        self.pipeline = rs2.pipeline()
        self.config = rs2.config()
        self.config.enable_stream(rs2.stream.depth, 640, 480, rs2.format.z16, 30)
        self.config.enable_stream(rs2.stream.color, 640, 480, rs2.format.bgr8, 30)
        profile = self.pipeline.start(self.config)

        depth_sensor = profile.get_device().first_depth_sensor()
        depth_sensor.set_option(
            rs2.option.visual_preset, 3
        )  # Set high accuracy for depth sensor
        self.depth_scale = depth_sensor.get_depth_scale()

        align_to = rs2.stream.color
        self.align = rs2.align(align_to)

    def get_rs_frames(self):
        '''
        Gets color and depth frames from realsense
        Returns: color_image, depth_image (numpy arrays)
        '''
        frames = self.pipeline.wait_for_frames()
        aligned_frames = self.align.process(frames)
        aligned_depth_frame = aligned_frames.get_depth_frame()
        color_frame = aligned_frames.get_color_frame()

        depth_image = np.asanyarray(aligned_depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())

        depth_image = depth_image.copy()
        color_image = color_image.copy()

        return color_image, depth_image
    
    def init_camera_intrinsics(self):  
        '''
        Initializes camera intrinsics
        
        Realsense point coordinate system:
            - positive x-axis points to the right, 
            - positive y-axis points down
            - positive z-axis points forward.
        '''
        print('Initializing camera intrinsics...')
        cameraInfo = CameraInfo()
        cameraInfo.width = 640
        cameraInfo.height = 480
        cameraInfo.distortion_model = "plumb_bob"
        cameraInfo.D = [0.094787, -0.230423, 0.005773, 0.007524, 0.000000]
        cameraInfo.K = [599.359419, 0.0, 328.503823, 0.0, 599.123731, 246.768289, 0.0, 0.0, 1.0]

        self.intrinsics = rs2.intrinsics()
        self.intrinsics.width = cameraInfo.width
        self.intrinsics.height = cameraInfo.height
        self.intrinsics.ppx = cameraInfo.K[2]
        self.intrinsics.ppy = cameraInfo.K[5]
        self.intrinsics.fx = cameraInfo.K[0]
        self.intrinsics.fy = cameraInfo.K[4]
        self.intrinsics.model  = rs2.distortion.brown_conrady  
        # self.intrinsics.model  = rs2.distortion.none  
        self.intrinsics.coeffs = [i for i in cameraInfo.D]  

    def handle_object_detection(self, req):
        self.calc_object_pose()
        return GetDetectedObjectPoseResponse(self.object_detected, self.object_pose)

    def detect_blob(self, img, lower_hsv, upper_hsv):
        # convert to HSV
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        # print(hsv[350:360,300:310])
        # Threshold the HSV image to get only yellow colors
        mask = cv2.inRange(hsv, lower_hsv, upper_hsv)

        # create the params and deactivate the 3 filters
        params = cv2.SimpleBlobDetector_Params()
        params.filterByArea = False
        params.filterByInertia = False
        params.filterByConvexity = False


        # Bitwise-AND mask and original image
        res = cv2.bitwise_and(img,img, mask= mask)


        cv2.imwrite('/home/kimlab/catkin_ws/my_images/latest_image_res.png',res);    

        detector = cv2.SimpleBlobDetector_create()

        # Detect blobs.
        keypoints = detector.detect(res)

        if len(keypoints) == 0:
            return None

        blob_keypoint = keypoints[0].pt
        return blob_keypoint

    def detect_yellow_trash(self, img):
        lower_yellow = np.array([15,100,100])
        upper_yellow = np.array([100,200,200])

        blob_keypoint = self.detect_blob(img, lower_yellow, upper_yellow)
        return blob_keypoint
    
    def rs2_deproject_pixel_to_point(self, instrinsics, pixel_value, depth):
        print("pixel_value: ", pixel_value)
        print("depth: ", depth)

        # Deproject pixel to point in 3D
        point_x = (pixel_value[0] - instrinsics.ppx) * depth / instrinsics.fx
        point_y = (pixel_value[1] - instrinsics.ppy) * depth / instrinsics.fy
        point_z = depth

        # Distortion model - Brown Conrady (plumb bob)
        if instrinsics.model == rs2.distortion.brown_conrady:
            for i in range(10):
                r2 = point_x * point_x + point_y * point_y
                f = 1 + instrinsics.coeffs[0] * r2 + instrinsics.coeffs[1] * r2 * r2 + instrinsics.coeffs[4] * r2 * r2 * r2
                ux = point_x * f + 2 * instrinsics.coeffs[2] * point_x * point_y + instrinsics.coeffs[3] * (r2 + 2 * point_x * point_x)
                uy = point_y * f + 2 * instrinsics.coeffs[3] * point_x * point_y + instrinsics.coeffs[2] * (r2 + 2 * point_y * point_y)
                dx = ux - point_x
                dy = uy - point_y
                if dx * dx + dy * dy < 1e-6:
                    break
                point_x = ux
                point_y = uy

        point = np.array([point_x, point_y, point_z])

        return point
    def scan_for_object(self):
        # get realsense frames
        # print("start function")
        color_image, depth_image = self.get_rs_frames()
        # print('frame dims: ', color_image.shape, depth_image.shape)
        # cv2.imwrite('/home/kimlab/catkin_ws/my_images/latest_image.png',color_image);    

        # if self.object_detected:
            # self.pub.publish(self.object_detected)

            # return
        obj_pixel_pos = self.detect_yellow_trash(color_image)
        # print(color_image)

        # if object not detected, return
        if obj_pixel_pos is None :
            self.object_detected = False
            return

        # publish object detected flag
        self.object_detected = True
        self.depth_image = depth_image
        self.obj_pixel_pos = obj_pixel_pos
        # print("about to publish")
        self.pub.publish(self.object_detected)
        print("after publish")
        

    def calc_object_pose(self):
        depth_image = self.depth_image
        obj_pixel_pos = self.obj_pixel_pos
        print(obj_pixel_pos)
        if depth_image is None or obj_pixel_pos is None:
            self.object_detected = False
            return

        depth_bounding_box = depth_image[max(0,int(obj_pixel_pos[1]) - 10): min(int(obj_pixel_pos[1]) + 10, depth_image.shape[0]),max(0,int(obj_pixel_pos[0]) - 10): min(int(obj_pixel_pos[0]) + 10, depth_image.shape[1])] 
        depth = cv2.mean(depth_bounding_box) * self.depth_scale
        print("depth at (x, y) in meters", depth)

        predicted_3D_coord_c = self.rs2_deproject_pixel_to_point(self.intrinsics, obj_pixel_pos, depth)
        print("predicted 3D coords in camera frame:\n", predicted_3D_coord_c)

        # get camera extrinsics
        R, t = self.get_current_camera_pose()
        print("R: ", R)
        print("t: ", t)

        predicted_3D_coord_w = np.matmul(R, predicted_3D_coord_c) + t
        print("predicted 3D coords in world frame:\n", predicted_3D_coord_w)
        
        # if object found
        self.object_detected = True
        self.object_pose = Pose()
        self.object_pose.position.x = predicted_3D_coord_w[0]
        self.object_pose.position.y = predicted_3D_coord_w[1]
        self.object_pose.position.z = predicted_3D_coord_w[2]

    def get_current_camera_pose(self):
        '''
        Gets current camera pose used for camera extrinsics
        Returns: R (3x3 rotation matrix), t (3x1 translation vector)
        '''
        # if self.use_tf:
        # get camera pose from tf
        trans = self.tfBuffer.lookup_transform("env", "robot1/camera_link", rospy.Time())
        R = quaternion_matrix([trans.transform.rotation.x, 
                                        trans.transform.rotation.y,
                                        trans.transform.rotation.z,
                                        trans.transform.rotation.w])[:3,:3]
        t = np.array([trans.transform.translation.x, 
                        trans.transform.translation.y,
                        trans.transform.translation.z])
        # else:
        #     R = np.array([  [0.0279773,  0.7066555, -0.7070044],
        #                 [0.9882467,  0.0867825,  0.1258461],
        #                 [0.1502855, -0.7022157, -0.6959220]])
        #     t = np.array([0.9624604 , 0.291157812, 0.6327067])

        return R, t

    def run(self):
        while not rospy.is_shutdown():
            self.scan_for_object()
            rospy.sleep(0.5)

if __name__ == '__main__':
    object_detector = ObjectDetector()
    object_detector.run()
