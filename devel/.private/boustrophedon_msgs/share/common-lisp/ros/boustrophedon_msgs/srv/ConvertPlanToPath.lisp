; Auto-generated. Do not edit!


(cl:in-package boustrophedon_msgs-srv)


;//! \htmlinclude ConvertPlanToPath-request.msg.html

(cl:defclass <ConvertPlanToPath-request> (roslisp-msg-protocol:ros-message)
  ((plan
    :reader plan
    :initarg :plan
    :type boustrophedon_msgs-msg:StripingPlan
    :initform (cl:make-instance 'boustrophedon_msgs-msg:StripingPlan)))
)

(cl:defclass ConvertPlanToPath-request (<ConvertPlanToPath-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ConvertPlanToPath-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ConvertPlanToPath-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name boustrophedon_msgs-srv:<ConvertPlanToPath-request> is deprecated: use boustrophedon_msgs-srv:ConvertPlanToPath-request instead.")))

(cl:ensure-generic-function 'plan-val :lambda-list '(m))
(cl:defmethod plan-val ((m <ConvertPlanToPath-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader boustrophedon_msgs-srv:plan-val is deprecated.  Use boustrophedon_msgs-srv:plan instead.")
  (plan m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ConvertPlanToPath-request>) ostream)
  "Serializes a message object of type '<ConvertPlanToPath-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'plan) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ConvertPlanToPath-request>) istream)
  "Deserializes a message object of type '<ConvertPlanToPath-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'plan) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ConvertPlanToPath-request>)))
  "Returns string type for a service object of type '<ConvertPlanToPath-request>"
  "boustrophedon_msgs/ConvertPlanToPathRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ConvertPlanToPath-request)))
  "Returns string type for a service object of type 'ConvertPlanToPath-request"
  "boustrophedon_msgs/ConvertPlanToPathRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ConvertPlanToPath-request>)))
  "Returns md5sum for a message object of type '<ConvertPlanToPath-request>"
  "3a9fa9a5fc158705104b513251035497")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ConvertPlanToPath-request)))
  "Returns md5sum for a message object of type 'ConvertPlanToPath-request"
  "3a9fa9a5fc158705104b513251035497")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ConvertPlanToPath-request>)))
  "Returns full string definition for message of type '<ConvertPlanToPath-request>"
  (cl:format cl:nil "boustrophedon_msgs/StripingPlan plan~%~%================================================================================~%MSG: boustrophedon_msgs/StripingPlan~%Header header~%StripingPoint[] points~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: boustrophedon_msgs/StripingPoint~%geometry_msgs/Point point~%uint8 type~%~%uint8 OUTLINE = 0~%uint8 STRIPE_START = 1~%uint8 STRIPE_END = 2~%uint8 STRIPE_INTERMEDIATE = 3~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ConvertPlanToPath-request)))
  "Returns full string definition for message of type 'ConvertPlanToPath-request"
  (cl:format cl:nil "boustrophedon_msgs/StripingPlan plan~%~%================================================================================~%MSG: boustrophedon_msgs/StripingPlan~%Header header~%StripingPoint[] points~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: boustrophedon_msgs/StripingPoint~%geometry_msgs/Point point~%uint8 type~%~%uint8 OUTLINE = 0~%uint8 STRIPE_START = 1~%uint8 STRIPE_END = 2~%uint8 STRIPE_INTERMEDIATE = 3~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ConvertPlanToPath-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'plan))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ConvertPlanToPath-request>))
  "Converts a ROS message object to a list"
  (cl:list 'ConvertPlanToPath-request
    (cl:cons ':plan (plan msg))
))
;//! \htmlinclude ConvertPlanToPath-response.msg.html

(cl:defclass <ConvertPlanToPath-response> (roslisp-msg-protocol:ros-message)
  ((path
    :reader path
    :initarg :path
    :type nav_msgs-msg:Path
    :initform (cl:make-instance 'nav_msgs-msg:Path)))
)

(cl:defclass ConvertPlanToPath-response (<ConvertPlanToPath-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ConvertPlanToPath-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ConvertPlanToPath-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name boustrophedon_msgs-srv:<ConvertPlanToPath-response> is deprecated: use boustrophedon_msgs-srv:ConvertPlanToPath-response instead.")))

(cl:ensure-generic-function 'path-val :lambda-list '(m))
(cl:defmethod path-val ((m <ConvertPlanToPath-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader boustrophedon_msgs-srv:path-val is deprecated.  Use boustrophedon_msgs-srv:path instead.")
  (path m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ConvertPlanToPath-response>) ostream)
  "Serializes a message object of type '<ConvertPlanToPath-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'path) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ConvertPlanToPath-response>) istream)
  "Deserializes a message object of type '<ConvertPlanToPath-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'path) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ConvertPlanToPath-response>)))
  "Returns string type for a service object of type '<ConvertPlanToPath-response>"
  "boustrophedon_msgs/ConvertPlanToPathResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ConvertPlanToPath-response)))
  "Returns string type for a service object of type 'ConvertPlanToPath-response"
  "boustrophedon_msgs/ConvertPlanToPathResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ConvertPlanToPath-response>)))
  "Returns md5sum for a message object of type '<ConvertPlanToPath-response>"
  "3a9fa9a5fc158705104b513251035497")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ConvertPlanToPath-response)))
  "Returns md5sum for a message object of type 'ConvertPlanToPath-response"
  "3a9fa9a5fc158705104b513251035497")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ConvertPlanToPath-response>)))
  "Returns full string definition for message of type '<ConvertPlanToPath-response>"
  (cl:format cl:nil "nav_msgs/Path path~%~%~%================================================================================~%MSG: nav_msgs/Path~%#An array of poses that represents a Path for a robot to follow~%Header header~%geometry_msgs/PoseStamped[] poses~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/PoseStamped~%# A Pose with reference coordinate frame and timestamp~%Header header~%Pose pose~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ConvertPlanToPath-response)))
  "Returns full string definition for message of type 'ConvertPlanToPath-response"
  (cl:format cl:nil "nav_msgs/Path path~%~%~%================================================================================~%MSG: nav_msgs/Path~%#An array of poses that represents a Path for a robot to follow~%Header header~%geometry_msgs/PoseStamped[] poses~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/PoseStamped~%# A Pose with reference coordinate frame and timestamp~%Header header~%Pose pose~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ConvertPlanToPath-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'path))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ConvertPlanToPath-response>))
  "Converts a ROS message object to a list"
  (cl:list 'ConvertPlanToPath-response
    (cl:cons ':path (path msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'ConvertPlanToPath)))
  'ConvertPlanToPath-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'ConvertPlanToPath)))
  'ConvertPlanToPath-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ConvertPlanToPath)))
  "Returns string type for a service object of type '<ConvertPlanToPath>"
  "boustrophedon_msgs/ConvertPlanToPath")