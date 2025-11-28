import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
from geometry_msgs.msg import Quaternion, Pose, Point
from tf.transformations import quaternion_from_euler

class ARCodeDetection:
    def __init__(self):
        rospy.init_node('ar_code_detection_node', anonymous=True)
        self.bridge = CvBridge()
        self.move_base = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.move_base.wait_for_server()
        self.ar_detected = False
        self.depth_image_subscriber = None

    def start_depth_image_processing(self):
        self.depth_image_subscriber = rospy.Subscriber('/camera/depth/image_raw', Image, self.depth_image_callback)

    def stop_depth_image_processing(self):
        if self.depth_image_subscriber is not None:
            self.depth_image_subscriber.unregister()

    def depth_image_callback(self, depth_msg):
        cv_depth_image = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding="passthrough")
        ar_detected, ar_poses = self.detect_ar_codes(cv_depth_image)

        if ar_detected:
            self.move_base.cancel_goal()
            self.correct_robot_pose(ar_poses)
            self.stop_depth_image_processing()

    def detect_ar_codes(self, depth_image):
        # �����ͼ��ת��Ϊ�Ҷ�ͼ����Ϊ����ֻ��Ҫ�Ҷ�ֵ�����AR��
        gray_image = cv2.cvtColor(depth_image, cv2.COLOR_BGR2GRAY)

        # ��������ʹ��OpenCV��ArUco�������AR��
        # ���ȣ�������Ҫ����Ԥ������ֵ䣬��������ʹ��5x5���ֵ�
        aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_5X5_100)
        aruco_params = cv2.aruco.DetectorParameters_create()

        # ʹ��detectMarkers���������ͼ���еı��
        corners, ids, rejectedImgPoints = cv2.aruco.detectMarkers(gray_image, aruco_dict, parameters=aruco_params)

        ar_detected = False
        ar_poses = []

        # �����⵽����һ��AR��
        if ids is not None:
            ar_detected = True

            # �������м�⵽�ı��
            for i in range(len(ids)):
                # ����ÿ����ǵ�����
                c = corners[i][0]
                center = tuple(np.mean(c, axis=0).astype(int))
                ar_poses.append(center)

            
            ar_poses.sort(key=lambda x: x[0])  
            center_ar_pose = ar_poses[len(ar_poses) // 2]  

            
            ar_poses = [center_ar_pose]

        return ar_detected, ar_poses

    def correct_robot_pose(self, ar_poses):
        
        center_x = 320
        center_y = 240

       
        angle_per_pixel = 0.05  # ����ֵ

        
        errors_x = [ar_pose[0] - center_x for ar_pose in ar_poses]
        errors_y = [ar_pose[1] - center_y for ar_pose in ar_poses]

     
        avg_error_x = sum(errors_x) / len(errors_x)
        avg_error_y = sum(errors_y) / len(errors_y)

       
        rotate_angle_x = avg_error_x * angle_per_pixel
        rotate_angle_y = avg_error_y * angle_per_pixel

      
        print("��Ҫ�� X ����ת�ĽǶ�: {:.2f} ��".format(rotate_angle_x))
        print("��Ҫ�� Y ����ת�ĽǶ�: {:.2f} ��".format(rotate_angle_y))

     
        self.robot.rotate_robot(rotate_angle_x, rotate_angle_y)
if __name__ == '__main__':
    ar_code_detector = ARCodeDetection()
    ar_code_detector.start_depth_image_processing()
    rospy.spin()
