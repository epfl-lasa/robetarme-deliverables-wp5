; Auto-generated. Do not edit!


(cl:in-package boustrophedon_msgs-msg)


;//! \htmlinclude StripingPlan.msg.html

(cl:defclass <StripingPlan> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (points
    :reader points
    :initarg :points
    :type (cl:vector boustrophedon_msgs-msg:StripingPoint)
   :initform (cl:make-array 0 :element-type 'boustrophedon_msgs-msg:StripingPoint :initial-element (cl:make-instance 'boustrophedon_msgs-msg:StripingPoint))))
)

(cl:defclass StripingPlan (<StripingPlan>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <StripingPlan>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'StripingPlan)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name boustrophedon_msgs-msg:<StripingPlan> is deprecated: use boustrophedon_msgs-msg:StripingPlan instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <StripingPlan>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader boustrophedon_msgs-msg:header-val is deprecated.  Use boustrophedon_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'points-val :lambda-list '(m))
(cl:defmethod points-val ((m <StripingPlan>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader boustrophedon_msgs-msg:points-val is deprecated.  Use boustrophedon_msgs-msg:points instead.")
  (points m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <StripingPlan>) ostream)
  "Serializes a message object of type '<StripingPlan>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'points))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'points))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <StripingPlan>) istream)
  "Deserializes a message object of type '<StripingPlan>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'points) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'points)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'boustrophedon_msgs-msg:StripingPoint))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<StripingPlan>)))
  "Returns string type for a message object of type '<StripingPlan>"
  "boustrophedon_msgs/StripingPlan")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'StripingPlan)))
  "Returns string type for a message object of type 'StripingPlan"
  "boustrophedon_msgs/StripingPlan")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<StripingPlan>)))
  "Returns md5sum for a message object of type '<StripingPlan>"
  "b2838ae4f42498e5013db14343fd340a")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'StripingPlan)))
  "Returns md5sum for a message object of type 'StripingPlan"
  "b2838ae4f42498e5013db14343fd340a")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<StripingPlan>)))
  "Returns full string definition for message of type '<StripingPlan>"
  (cl:format cl:nil "Header header~%StripingPoint[] points~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: boustrophedon_msgs/StripingPoint~%geometry_msgs/Point point~%uint8 type~%~%uint8 OUTLINE = 0~%uint8 STRIPE_START = 1~%uint8 STRIPE_END = 2~%uint8 STRIPE_INTERMEDIATE = 3~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'StripingPlan)))
  "Returns full string definition for message of type 'StripingPlan"
  (cl:format cl:nil "Header header~%StripingPoint[] points~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: boustrophedon_msgs/StripingPoint~%geometry_msgs/Point point~%uint8 type~%~%uint8 OUTLINE = 0~%uint8 STRIPE_START = 1~%uint8 STRIPE_END = 2~%uint8 STRIPE_INTERMEDIATE = 3~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <StripingPlan>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'points) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <StripingPlan>))
  "Converts a ROS message object to a list"
  (cl:list 'StripingPlan
    (cl:cons ':header (header msg))
    (cl:cons ':points (points msg))
))
