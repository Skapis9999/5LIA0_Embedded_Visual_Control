;; Auto-generated. Do not edit!


(when (boundp 'motion_skapis::dualImage)
  (if (not (find-package "MOTION_SKAPIS"))
    (make-package "MOTION_SKAPIS"))
  (shadow 'dualImage (find-package "MOTION_SKAPIS")))
(unless (find-package "MOTION_SKAPIS::DUALIMAGE")
  (make-package "MOTION_SKAPIS::DUALIMAGE"))

(in-package "ROS")
;;//! \htmlinclude dualImage.msg.html
(if (not (find-package "SENSOR_MSGS"))
  (ros::roseus-add-msgs "sensor_msgs"))


(defclass motion_skapis::dualImage
  :super ros::object
  :slots (_image1 _image2 ))

(defmethod motion_skapis::dualImage
  (:init
   (&key
    ((:image1 __image1) (instance sensor_msgs::Image :init))
    ((:image2 __image2) (instance sensor_msgs::Image :init))
    )
   (send-super :init)
   (setq _image1 __image1)
   (setq _image2 __image2)
   self)
  (:image1
   (&rest __image1)
   (if (keywordp (car __image1))
       (send* _image1 __image1)
     (progn
       (if __image1 (setq _image1 (car __image1)))
       _image1)))
  (:image2
   (&rest __image2)
   (if (keywordp (car __image2))
       (send* _image2 __image2)
     (progn
       (if __image2 (setq _image2 (car __image2)))
       _image2)))
  (:serialization-length
   ()
   (+
    ;; sensor_msgs/Image _image1
    (send _image1 :serialization-length)
    ;; sensor_msgs/Image _image2
    (send _image2 :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; sensor_msgs/Image _image1
       (send _image1 :serialize s)
     ;; sensor_msgs/Image _image2
       (send _image2 :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; sensor_msgs/Image _image1
     (send _image1 :deserialize buf ptr-) (incf ptr- (send _image1 :serialization-length))
   ;; sensor_msgs/Image _image2
     (send _image2 :deserialize buf ptr-) (incf ptr- (send _image2 :serialization-length))
   ;;
   self)
  )

(setf (get motion_skapis::dualImage :md5sum-) "0f5355187faf5ab03929c28f8e5fee54")
(setf (get motion_skapis::dualImage :datatype-) "motion_skapis/dualImage")
(setf (get motion_skapis::dualImage :definition-)
      "sensor_msgs/Image image1
sensor_msgs/Image image2
================================================================================
MSG: sensor_msgs/Image
# This message contains an uncompressed image
# (0, 0) is at top-left corner of image
#

Header header        # Header timestamp should be acquisition time of image
                     # Header frame_id should be optical frame of camera
                     # origin of frame should be optical center of camera
                     # +x should point to the right in the image
                     # +y should point down in the image
                     # +z should point into to plane of the image
                     # If the frame_id here and the frame_id of the CameraInfo
                     # message associated with the image conflict
                     # the behavior is undefined

uint32 height         # image height, that is, number of rows
uint32 width          # image width, that is, number of columns

# The legal values for encoding are in file src/image_encodings.cpp
# If you want to standardize a new string format, join
# ros-users@lists.sourceforge.net and send an email proposing a new encoding.

string encoding       # Encoding of pixels -- channel meaning, ordering, size
                      # taken from the list of strings in include/sensor_msgs/image_encodings.h

uint8 is_bigendian    # is this data bigendian?
uint32 step           # Full row length in bytes
uint8[] data          # actual matrix data, size is (step * rows)

================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id

")



(provide :motion_skapis/dualImage "0f5355187faf5ab03929c28f8e5fee54")


