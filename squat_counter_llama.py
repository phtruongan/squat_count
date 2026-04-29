#!/usr/bin/env python3
# This version calculate based on the slope angle of the thigh
import time
from ultralytics import YOLO
import cv2
import numpy as np
from collections import deque

# robotic library
import rospy

# image library
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from human_tracking import rotate_head

# import library for qt speech
from qt_robot_interface.srv import *

import threading

# Threshold value for the knee angle (angle between hip-knee and knee-angle line. 0 degrees 
angle_threshold = 90

# Threshold value for the slope angle (the slope angle of the hip-knee line. 0 degree mean vertical. > 90 degress means that the hip joint is lower than the knee joint)
slope_threshold = 70

# instruction for deeper based on the difference betweeen current angle and next angle
prev_angle = 0
delta_angle_threshold = 20
squat_depth_ratio = 0.5

model = YOLO('/home/qtrobot/catkin_ws/src/squat_count/yolo11n-pose.pt')  # load YOLOv11 pose estimation model 
#cap = cv2.VideoCapture(0)

#head yaw
current_yaw = 0 

#Draw the joints on the frame. In this case only left side is working
def draw_joints(frame, joints):
    global angle
    
    if joints is None:
        return frame

    for joint in joints:                
        if not (joint[0] == 0 and joint[1] == 0):
            cv2.circle(frame, joint, 3, (0, 0, 0), 3)

    # Draw line between hip and knee
        shoulder = joints[0]
        hip = joints[1]
        knee = joints[2]
        ankle = joints[3]

    if not (hip[0] == 0 and hip[1] == 0) and not (knee[0] == 0 and knee[1] == 0):
        cv2.line(frame, hip, knee, (255, 0, 0), 2)  # Blue line

    if not (knee[0] == 0 and knee[1] == 0) and not (ankle[0] == 0 and ankle[1] == 0):
        cv2.line(frame, knee, ankle, (255, 0, 0), 2)  # Blue line

    if not (hip[0] == 0 and hip[1] == 0) and not (shoulder[0] == 0 and shoulder[1] == 0):
        cv2.line(frame, hip, shoulder, (255, 0, 0), 2)  # Blue line

    if not (knee[0] == 0 and knee[1] == 0) and not (ankle[0] == 0 and ankle[1] == 0):
        cv2.line(frame, knee, ankle, (255, 0, 0), 2)  # Blue line

    return frame

# AN code: Adding to find the location of human in frame
def extract_human_bbox(poses):
    global current_yaw
    rotate_head(0,-10)

    #for pose in poses:
    #    xy = pose.keypoints.xy.int().numpy()
    #    tl = np.min(xy[0], axis=0) # calculate topleft
    #    br = np.max(xy[0], axis=0) # calcualte bottomright
    #    center_x = (br[0] + tl[0]) / 2.0
    #    center_y = (br[1] + tl[1]) / 2.0
        # propotion of center to center
        # 640 x 480
    
    #    delta_x = center_x - 640/2.0
    #    yaw = delta_x / (640/2.0) * 90
    #    if yaw > 0:
    #        print(yaw)
    #    if abs(current_yaw - yaw) > 10: 
    #        rotate_head(-(yaw-10), 0)
    #        current_yaw = -(yaw-10)
    #    elif abs(current_yaw - yaw) > 5:
    #        rotate_head(-(yaw-10), 0)
    #        current_yaw = -(yaw-5)

def get_left_joints(frame):
    # Predict with the model
    results = model(frame)  # predict on an image

    # TODO: Task 3 qt robot follows the human box.
    extract_human_bbox(results)
    
    for r in results:
        keypoints = r.keypoints.xy.int().numpy()

        # Check if any keypoints were detected BEFORE accessing keypoints[0]
        if keypoints.shape[0] == 0:
            return None

        joints = keypoints[0][5:17]

        left_joints = [
            joints[0],  #left shoulder
            joints[6],  #left hip
            joints[8],  #left knee
            joints[10]  #left ankle (vrist)
        ]

        return left_joints
    return None

#def calculate_thigh_angle(hip, knee):
#    hip = np.array(hip)
#    knee = np.array(knee)
#
#    dx = abs(knee[0] - hip[0])  
#    dy = knee[1] - hip[1]  
#    
#    if dy >= 0:
#        angle_rad = np.arctan2(dx, dy)
#        angle_deg = np.degrees(angle_rad)
#    else:
#        angle_rad = np.arctan2(dx, -dy)
#        angle_deg = 180.0 - np.degrees(angle_rad)
#    
#    return angle_deg

# Instead of calculating the thigh angle, we calculate the knee angel
def calculate_knee_angle(hip, knee, ankle):
    hip = np.array(hip)
    knee = np.array(knee)

    ankle = np.array(ankle)

    # Vector from knee to hip
    vector1 = hip - knee
    # Vector from knee to ankle
    vector2 = ankle - knee

    # Calculate angle between vectors using dot product
    dot_product = np.dot(vector1, vector2)
    magnitude1 = np.linalg.norm(vector1)
    magnitude2 = np.linalg.norm(vector2)

    # Avoid division by zero
    if magnitude1 == 0 or magnitude2 == 0:
        return 0

    cos_angle = dot_product / (magnitude1 * magnitude2)

    # Clamp to [-1,1] to avoid onumerical erros
    cos_angle = np.clip(cos_angle, -1.0, 1.0)
    angle_rad = np.arccos(cos_angle)
    angle_deg = np.degrees(angle_rad)

    # Convert so that straight leg = 0 degrees
    knee_angle = 180.0 - angle_deg
    return knee_angle

def check_hip_depth(hip, knee):
    """
    Check if hip is lower than or equal to knee (proper squat depth)
    Returns True if hip y-coordinate >= knee y-coordinate (lower in image coordinates)
    """

    hip = np.array(hip)
    knee = np.array(knee)

    # In image coordinates, y increases downward
    # So hip_y >= knee_y meahns hip is lower than knee
    return hip[1] >= knee[1]

def calculate_slope_angle(hip, knee):
    """
    Calculate the slope angle of the line between hip and knee
    Vertical line = 0 degrees, Horizontal line = 90 degrees
    """

    hip = np.array(hip)
    knee = np.array(knee)

    # Calculate the differences
    dx = knee[0] - hip[0]
    dy = knee[1] - hip[1]

    # Avoid division by zero
    if dx == 0 and dy == 0:
        return 0

    # Caluclate angle from vertical (0 degree = vertical, 90 degrees = horizontal)
    # atan2 gives angle from horizontal, so we convert
    angle_from_horizontal = np.degrees(np.arctan2(abs(dy),abs(dx)))

    # Convert to angle from vertical
    slope_angle = 90.0 - angle_from_horizontal

    return abs(slope_angle)

#def squat_counter(thigh_angle, angle_buffer, squat_counts, in_squat):
#def squat_counter(knee_angle, angle_buffer, squat_counts, in_squat):
def squat_counter(knee_angle, slope_angle, hip_below_knee, angle_buffer, squat_counts, in_squat):
    global delta_angle_threshold, prev_angle
    angle_buffer.append(knee_angle)
    
    # Only check if we have enough frames
    if len(angle_buffer) == 1:
        mean_angle = np.mean(angle_buffer)
        
        ## Detect squat position (thigh horizontal or beyond)
        #if mean_angle >= angle_threshold and not in_squat:
        # Detect proper squat position: knee angle >= 90 AND (hip below knee OR slope angle >= 90)
        proper_depth = mean_angle >= angle_threshold and (hip_below_knee or slope_angle >= slope_threshold)
        
        if proper_depth and not in_squat:
            in_squat = True
            # Notify the user that she/he touched the point
            instruction = 'Good'
            #feedbackToSpeech(instruction)

        # Reset when standing up (leg almost straight)
        elif mean_angle < 20 and in_squat:
            squat_counts += 1
            # QT counting
            feedbackToSpeech(str(squat_counts))
            in_squat = False

        # Instruction for a deeper squat
        elif mean_angle >= angle_threshold * squat_depth_ratio and not in_squat:
            if delta_angle_threshold < abs(prev_angle - mean_angle):
                # inform the user that she/he needs to go deeper 
                instruction = 'Deeper please!'
                #feedbackToSpeech(instruction)
                prev_angle = mean_angle
    
    return squat_counts, in_squat

def main_processing(frame):
    #global thigh_angles, angle_buffer, squat_counts, in_squat
    global knee_angles, angle_buffer, squat_counts, in_squat

    joints = get_left_joints(frame)

    if joints is None:
        cv2.putText(frame, 'No person detected', (10,100), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,0,255), 2)
        frame = cv2.resize(frame, None, fx=1.2, fy=1.2, interpolation = cv2.INTER_NEAREST)
        cv2.imshow('YoloV11-based Joints', frame)
        key = cv2.waitKey(1)

        # Exit the program
        if key==27:
            #TODO 
            rospy.signal_shutdown('Exiting')
            sys.exit()

    draw_joints(frame, joints)
    #thigh_angle = calculate_thigh_angle(joints[1], joints[2])
    #thigh_angles.append(thigh_angle)
    knee_angle = calculate_knee_angle(joints[1], joints[2], joints[3])
    knee_angles.append(knee_angle)

    #squat_counts, in_squat = squat_counter(knee_angle, angle_buffer, squat_counts, in_squat)
    
    # Calculate slope angle and check hip depth
    slope_angle = calculate_slope_angle(joints[1], joints[2])
    hip_below_knee = check_hip_depth(joints[1],joints[2])

    squat_counts, in_squat = squat_counter(knee_angle, slope_angle, hip_below_knee, angle_buffer, squat_counts, in_squat)

    # Determine if current position is proper depth
    proper_depth = knee_angle >= angle_threshold and (hip_below_knee or slope_angle >= slope_threshold)
    depth_color = (0, 255, 0) if proper_depth else (0,0,255)
    depth_text = "PROPER DEPTH" if proper_depth else "NOT DEEP ENOUGH"

    # Display squat count on frame
    cv2.putText(frame, f"Squats: {squat_counts}", (10, 30), 
                cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
    #cv2.putText(frame, f"Knee Angle: {knee_angle:.1f}", (10, 60), 
    #            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
    
    cv2.putText(frame, f"Knee Angle: {knee_angle:.1f} | Slope: {slope_angle:.1f}", (10, 60), 
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)

    cv2.putText(frame, f"Hip Below Knee: {hip_below_knee}", (10, 90), 
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)

    cv2.putText(frame, depth_text, (10, 120), 
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, depth_color, 2)
    
    frame = cv2.resize(frame, None, fx=1.2, fy=1.2, interpolation = cv2.INTER_NEAREST)
    cv2.imshow('YoloV11-based Joints', frame)
        
    key = cv2.waitKey(1)

    # Exit the program
    if key == 27:
        #TODO
        rospy.signal_shutdown('Exiting')
        sys.exit()

    # Reset the counting by pressing "r"
    if key == 114:
        squat_counts = 0 



def joint_estimate(msg):
    #print('Get teh frame')
    bridge = CvBridge()

    cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
    main_processing(cv_image)
    

# define a ros service
speechConfig = rospy.ServiceProxy('/qt_robot/speech/config', speech_config)

# define a ros service
speechSay = rospy.ServiceProxy('/qt_robot/speech/say', speech_say)
# Text to speech
def feedbackToSpeech(feedback):


    #threading.Thread(target=speechSay, args=(feedback,)).start()
    threading.Thread(target=speechSay, args=(feedback,),daemon=True).start()

    # block/wait for ros service, this will pause the main program until the qt talk. It is not recommended
    #rospy.wait_for_service('/qt_robot/speech/say')

    # block/wait for ros service
    #rospy.wait_for_service('/qt_robot/speech/config')

    #try:
    #    status = speechConfig("en-US",0,0)
    #    if status:
    #        speechSay(feedback)
    #        status = False

    #except KeyboardInterrupt:
    #    pass

    #rospy.loginfo("finsihed!")



#MAIN loop
#thigh_angles = []  # For saving the joint angle time series
knee_angles = []  # For saving the joint angle time series
angle_buffer = deque(maxlen=1)  # Buffer for last 5 frames
squat_counts = 0 #Total number of squats
in_squat = False  # Track if currently in squat position
start_time = time.time()

rospy.init_node('my_node')
rospy.loginfo('My node started!!')

rospy.Subscriber('/camera/color/image_raw', Image, joint_estimate)
rospy.spin()
'''
while cap.isOpened():
    ret, frame = cap.read()
    
    if not ret:
        break
    
    joints = get_left_joints(frame)
    
    if joints is None:
        cv2.putText(frame, "No person detected", (10, 100), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
        frame = cv2.resize(frame, None, fx=1.2, fy=1.2, interpolation = cv2.INTER_NEAREST)
        cv2.imshow('YoloV11-based Joints', frame)
        key = cv2.waitKey(1)
        if key == 27:
            break
        continue
    
    draw_joints(frame, joints)
    
    thigh_angle = calculate_thigh_angle(joints[1], joints[2])
    thigh_angles.append(thigh_angle)
    
    squat_counts, in_squat = squat_counter(thigh_angle, angle_buffer, squat_counts, in_squat)
    
    # Display squat count on frame
    cv2.putText(frame, f"Squats: {squat_counts}", (10, 30), 
                cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
    cv2.putText(frame, f"Thigh Angle: {thigh_angle:.1f}", (10, 60), 
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)

    frame = cv2.resize(frame, None, fx=1.2, fy=1.2, interpolation = cv2.INTER_NEAREST)
    cv2.imshow('YoloV11-based Joints', frame)
        
    key = cv2.waitKey(1)
    
    if key == 27:
        break

cap.release()
cv2.destroyAllWindows()
'''
