; Auto-generated. Do not edit!


(cl:in-package motion_skapis-msg)


;//! \htmlinclude dualImage.msg.html

(cl:defclass <dualImage> (roslisp-msg-protocol:ros-message)
  ((image1
    :reader image1
    :initarg :image1
    :type sensor_msgs-msg:Image
    :initform (cl:make-instance 'sensor_msgs-msg:Image))
   (image2
    :reader image2
    :initarg :image2
    :type sensor_msgs-msg:Image
    :initform (cl:make-instance 'sensor_msgs-msg:Image)))
)

(cl:defclass dualImage (<dualImage>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <dualImage>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'dualImage)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name motion_skapis-msg:<dualImage> is deprecated: use motion_skapis-msg:dualImage instead.")))

(cl:ensure-generic-function 'image1-val :lambda-list '(m))
(cl:defmethod image1-val ((m <dualImage>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader motion_skapis-msg:image1-val is deprecated.  Use motion_skapis-msg:image1 instead.")
  (image1 m))

(cl:ensure-generic-function 'image2-val :lambda-list '(m))
(cl:defmethod image2-val ((m <dualImage>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader motion_skapis-msg:image2-val is deprecated.  Use motion_skapis-msg:image2 instead.")
  (image2 m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <dualImage>) ostream)
  "Serializes a message object of type '<dualImage>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'image1) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'image2) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <dualImage>) istream)
  "Deserializes a message object of type '<dualImage>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'image1) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'image2) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<dualImage>)))
  "Returns string type for a message object of type '<dualImage>"
  "motion_skapis/dualImage")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'dualImage)))
  "Returns string type for a message object of type 'dualImage"
  "motion_skapis/dualImage")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<dualImage>)))
  "Returns md5sum for a message object of type '<dualImage>"
  "0f5355187faf5ab03929c28f8e5fee54")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'dualImage)))
  "Returns md5sum for a message object of type 'dualImage"
  "0f5355187faf5ab03929c28f8e5fee54")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<dualImage>)))
  "Returns full string definition for message of type '<dualImage>"
  (cl:format cl:nil "sensor_msgs/Image image1~%sensor_msgs/Image image2~%================================================================================~%MSG: sensor_msgs/Image~%# This message contains an uncompressed image~%# (0, 0) is at top-left corner of image~%#~%~%Header header        # Header timestamp should be acquisition time of image~%                     # Header frame_id should be optical frame of camera~%                     # origin of frame should be optical center of camera~%                     # +x should point to the right in the image~%                     # +y should point down in the image~%                     # +z should point into to plane of the image~%                     # If the frame_id here and the frame_id of the CameraInfo~%                     # message associated with the image conflict~%                     # the behavior is undefined~%~%uint32 height         # image height, that is, number of rows~%uint32 width          # image width, that is, number of columns~%~%# The legal values for encoding are in file src/image_encodings.cpp~%# If you want to standardize a new string format, join~%# ros-users@lists.sourceforge.net and send an email proposing a new encoding.~%~%string encoding       # Encoding of pixels -- channel meaning, ordering, size~%                      # taken from the list of strings in include/sensor_msgs/image_encodings.h~%~%uint8 is_bigendian    # is this data bigendian?~%uint32 step           # Full row length in bytes~%uint8[] data          # actual matrix data, size is (step * rows)~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'dualImage)))
  "Returns full string definition for message of type 'dualImage"
  (cl:format cl:nil "sensor_msgs/Image image1~%sensor_msgs/Image image2~%================================================================================~%MSG: sensor_msgs/Image~%# This message contains an uncompressed image~%# (0, 0) is at top-left corner of image~%#~%~%Header header        # Header timestamp should be acquisition time of image~%                     # Header frame_id should be optical frame of camera~%                     # origin of frame should be optical center of camera~%                     # +x should point to the right in the image~%                     # +y should point down in the image~%                     # +z should point into to plane of the image~%                     # If the frame_id here and the frame_id of the CameraInfo~%                     # message associated with the image conflict~%                     # the behavior is undefined~%~%uint32 height         # image height, that is, number of rows~%uint32 width          # image width, that is, number of columns~%~%# The legal values for encoding are in file src/image_encodings.cpp~%# If you want to standardize a new string format, join~%# ros-users@lists.sourceforge.net and send an email proposing a new encoding.~%~%string encoding       # Encoding of pixels -- channel meaning, ordering, size~%                      # taken from the list of strings in include/sensor_msgs/image_encodings.h~%~%uint8 is_bigendian    # is this data bigendian?~%uint32 step           # Full row length in bytes~%uint8[] data          # actual matrix data, size is (step * rows)~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <dualImage>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'image1))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'image2))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <dualImage>))
  "Converts a ROS message object to a list"
  (cl:list 'dualImage
    (cl:cons ':image1 (image1 msg))
    (cl:cons ':image2 (image2 msg))
))
