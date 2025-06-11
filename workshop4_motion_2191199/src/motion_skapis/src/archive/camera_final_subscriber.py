#!/usr/bin/env python2
import rospy
from sensor_msgs.msg import Image
from jetson_camera.msg import dualImage
from cv_bridge import CvBridge
import cv2
import time

bridge = CvBridge()
recording = False
start_time = None
video_writer1 = None
video_writer2 = None
video_writer_combined = None
finished_recording = False
fps = 7
frame_times = []

def callback(msg):
    global recording, start_time, video_writer1, video_writer2, fps, finished_recording, video_writer_combined, frame_times 
    
    try:
        # now = time.time()
        # frame_times.append(now)
        # if len(frame_times) > 30:
        #     frame_times.pop(0)
        #     fps = len(frame_times) / (frame_times[-1] - frame_times[0])
        img1 = bridge.imgmsg_to_cv2(msg.image1, desired_encoding='bgr8')
        img2 = bridge.imgmsg_to_cv2(msg.image2, desired_encoding='bgr8')
        rospy.loginfo("Received image frame with: {} FPS".format(fps))
        
        # Display both images in separate windows
        cv2.imshow("Image 1", img1)
        cv2.imshow("Image 2", img2)
        cv2.waitKey(1)
        # record(img1, img2)
        if not recording and not finished_recording:
            height, width,  _ = img2.shape
            fourcc = cv2.VideoWriter_fourcc(*'XVID')
            current_time = time.strftime("%Y-%m-%d_%H-%M-%S", time.gmtime())
            video_writer1 = cv2.VideoWriter('/home/jetbot/EVC/workshops/workshop2_2191199/src/jetson_camera/saved_videos/jetson_video_distorted_{}.avi'.format(current_time), fourcc, fps, (width, height))
            video_writer2 = cv2.VideoWriter('/home/jetbot/EVC/workshops/workshop2_2191199/src/jetson_camera/saved_videos/jetson_video_undistorted_{}.avi'.format(current_time), fourcc, fps, (width, height))
            combined_width = width * 2
            combined_height = height
            video_writer_combined = cv2.VideoWriter(
                                '/home/jetbot/EVC/workshops/workshop2_2191199/src/jetson_camera/saved_videos/jetson_video_combined_{}.avi'.format(current_time), 
                                fourcc, fps, (combined_width, combined_height))
            
            start_time = time.time()
            recording = True
            rospy.loginfo("Started video recording...")

        if recording:
            # Add labels
            img1_labeled = img1.copy()
            img2_labeled = img2.copy()
            cv2.putText(img1_labeled, 'Distorted', (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,255,0), 2)
            # cv2.putText(img2_labeled, 'Undistorted', (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,255,0), 2)

            # Combine images
            combined = cv2.hconcat([img1_labeled, img2_labeled])
            video_writer1.write(img1)
            video_writer2.write(img2)
            video_writer_combined.write(combined)

        if recording and (time.time() - start_time >= 70):
            video_writer1.release()
            video_writer2.release()
            video_writer_combined.release()
            recording = False
            finished_recording = True
            rospy.loginfo("Finished recording!")

    except Exception as e:
        rospy.logerr("Failed to convert images: {}".format(e))

def record(img1, img2):
    if not recording and not finished_recording:
        height, width,  _ = img2.shape
        fourcc = cv2.VideoWriter_fourcc(*'XVID')
        current_time = time.strftime("%Y-%m-%d_%H-%M-%S", time.gmtime())
        # video_writer1 = cv2.VideoWriter('/home/jetbot/EVC/workshops/workshop2_1659383/src/jetson_camera/saved_videos/jetson_video_distorted_{}.avi'.format(current_time), fourcc, fps, (width, height))
        # video_writer2 = cv2.VideoWriter('/home/jetbot/EVC/workshops/workshop2_1659383/src/jetson_camera/saved_videos/jetson_video_undistorted_{}.avi'.format(current_time), fourcc, fps, (width, height))
        combined_width = width * 2
        combined_height = height
        # video_writer_combined = cv2.VideoWriter(
        #                     '/home/jetbot/EVC/workshops/workshop2_1659383/src/jetson_camera/saved_videos/jetson_video_combined_{}.avi'.format(current_time), 
        #                     fourcc, fps, (combined_width, combined_height))
        
        start_time = time.time()
        recording = True
        rospy.loginfo("Started video recording...")

    if recording:
        # Add labels
        img1_labeled = img1.copy()
        img2_labeled = img2.copy()
        cv2.putText(img1_labeled, 'Distorted', (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,255,0), 2)
        cv2.putText(img2_labeled, 'Undistorted', (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,255,0), 2)

        # Combine images
        combined = cv2.hconcat([img1_labeled, img2_labeled])
        video_writer1.write(img1)
        video_writer2.write(img2)
        video_writer_combined.write(combined)

    if recording and (time.time() - start_time >= 60):
        video_writer1.release()
        video_writer2.release()
        video_writer_combined.release()
        recording = False
        finished_recording = True
        video_writer1 = cv2.VideoWriter('/home/jetbot/EVC/workshops/workshop2_1659383/src/jetson_camera/saved_videos/jetson_video_distorted_{}.avi'.format(current_time), fourcc, fps, (width, height))
        video_writer2 = cv2.VideoWriter('/home/jetbot/EVC/workshops/workshop2_1659383/src/jetson_camera/saved_videos/jetson_video_undistorted_{}.avi'.format(current_time), fourcc, fps, (width, height))
        video_writer_combined = cv2.VideoWriter(
                            '/home/jetbot/EVC/workshops/workshop2_1659383/src/jetson_camera/saved_videos/jetson_video_combined_{}.avi'.format(current_time), 
                            fourcc, fps, (combined_width, combined_height))
        rospy.loginfo("Finished recording!")

def listener():
    rospy.init_node('camera_subscriber_node_Skapis')
    rospy.Subscriber('/camera/processed_image_Skapis', dualImage, callback)
    rospy.spin()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    listener()