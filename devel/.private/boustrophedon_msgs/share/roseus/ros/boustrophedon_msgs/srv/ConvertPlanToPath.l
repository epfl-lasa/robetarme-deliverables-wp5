;; Auto-generated. Do not edit!


(when (boundp 'boustrophedon_msgs::ConvertPlanToPath)
  (if (not (find-package "BOUSTROPHEDON_MSGS"))
    (make-package "BOUSTROPHEDON_MSGS"))
  (shadow 'ConvertPlanToPath (find-package "BOUSTROPHEDON_MSGS")))
(unless (find-package "BOUSTROPHEDON_MSGS::CONVERTPLANTOPATH")
  (make-package "BOUSTROPHEDON_MSGS::CONVERTPLANTOPATH"))
(unless (find-package "BOUSTROPHEDON_MSGS::CONVERTPLANTOPATHREQUEST")
  (make-package "BOUSTROPHEDON_MSGS::CONVERTPLANTOPATHREQUEST"))
(unless (find-package "BOUSTROPHEDON_MSGS::CONVERTPLANTOPATHRESPONSE")
  (make-package "BOUSTROPHEDON_MSGS::CONVERTPLANTOPATHRESPONSE"))

(in-package "ROS")



(if (not (find-package "NAV_MSGS"))
  (ros::roseus-add-msgs "nav_msgs"))


(defclass boustrophedon_msgs::ConvertPlanToPathRequest
  :super ros::object
  :slots (_plan ))

(defmethod boustrophedon_msgs::ConvertPlanToPathRequest
  (:init
   (&key
    ((:plan __plan) (instance boustrophedon_msgs::StripingPlan :init))
    )
   (send-super :init)
   (setq _plan __plan)
   self)
  (:plan
   (&rest __plan)
   (if (keywordp (car __plan))
       (send* _plan __plan)
     (progn
       (if __plan (setq _plan (car __plan)))
       _plan)))
  (:serialization-length
   ()
   (+
    ;; boustrophedon_msgs/StripingPlan _plan
    (send _plan :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; boustrophedon_msgs/StripingPlan _plan
       (send _plan :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; boustrophedon_msgs/StripingPlan _plan
     (send _plan :deserialize buf ptr-) (incf ptr- (send _plan :serialization-length))
   ;;
   self)
  )

(defclass boustrophedon_msgs::ConvertPlanToPathResponse
  :super ros::object
  :slots (_path ))

(defmethod boustrophedon_msgs::ConvertPlanToPathResponse
  (:init
   (&key
    ((:path __path) (instance nav_msgs::Path :init))
    )
   (send-super :init)
   (setq _path __path)
   self)
  (:path
   (&rest __path)
   (if (keywordp (car __path))
       (send* _path __path)
     (progn
       (if __path (setq _path (car __path)))
       _path)))
  (:serialization-length
   ()
   (+
    ;; nav_msgs/Path _path
    (send _path :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; nav_msgs/Path _path
       (send _path :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; nav_msgs/Path _path
     (send _path :deserialize buf ptr-) (incf ptr- (send _path :serialization-length))
   ;;
   self)
  )

(defclass boustrophedon_msgs::ConvertPlanToPath
  :super ros::object
  :slots ())

(setf (get boustrophedon_msgs::ConvertPlanToPath :md5sum-) "3a9fa9a5fc158705104b513251035497")
(setf (get boustrophedon_msgs::ConvertPlanToPath :datatype-) "boustrophedon_msgs/ConvertPlanToPath")
(setf (get boustrophedon_msgs::ConvertPlanToPath :request) boustrophedon_msgs::ConvertPlanToPathRequest)
(setf (get boustrophedon_msgs::ConvertPlanToPath :response) boustrophedon_msgs::ConvertPlanToPathResponse)

(defmethod boustrophedon_msgs::ConvertPlanToPathRequest
  (:response () (instance boustrophedon_msgs::ConvertPlanToPathResponse :init)))

(setf (get boustrophedon_msgs::ConvertPlanToPathRequest :md5sum-) "3a9fa9a5fc158705104b513251035497")
(setf (get boustrophedon_msgs::ConvertPlanToPathRequest :datatype-) "boustrophedon_msgs/ConvertPlanToPathRequest")
(setf (get boustrophedon_msgs::ConvertPlanToPathRequest :definition-)
      "boustrophedon_msgs/StripingPlan plan

================================================================================
MSG: boustrophedon_msgs/StripingPlan
Header header
StripingPoint[] points

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

================================================================================
MSG: boustrophedon_msgs/StripingPoint
geometry_msgs/Point point
uint8 type

uint8 OUTLINE = 0
uint8 STRIPE_START = 1
uint8 STRIPE_END = 2
uint8 STRIPE_INTERMEDIATE = 3

================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z
---
nav_msgs/Path path


================================================================================
MSG: nav_msgs/Path
#An array of poses that represents a Path for a robot to follow
Header header
geometry_msgs/PoseStamped[] poses

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

================================================================================
MSG: geometry_msgs/PoseStamped
# A Pose with reference coordinate frame and timestamp
Header header
Pose pose

================================================================================
MSG: geometry_msgs/Pose
# A representation of pose in free space, composed of position and orientation. 
Point position
Quaternion orientation

================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z

================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x
float64 y
float64 z
float64 w
")

(setf (get boustrophedon_msgs::ConvertPlanToPathResponse :md5sum-) "3a9fa9a5fc158705104b513251035497")
(setf (get boustrophedon_msgs::ConvertPlanToPathResponse :datatype-) "boustrophedon_msgs/ConvertPlanToPathResponse")
(setf (get boustrophedon_msgs::ConvertPlanToPathResponse :definition-)
      "boustrophedon_msgs/StripingPlan plan

================================================================================
MSG: boustrophedon_msgs/StripingPlan
Header header
StripingPoint[] points

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

================================================================================
MSG: boustrophedon_msgs/StripingPoint
geometry_msgs/Point point
uint8 type

uint8 OUTLINE = 0
uint8 STRIPE_START = 1
uint8 STRIPE_END = 2
uint8 STRIPE_INTERMEDIATE = 3

================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z
---
nav_msgs/Path path


================================================================================
MSG: nav_msgs/Path
#An array of poses that represents a Path for a robot to follow
Header header
geometry_msgs/PoseStamped[] poses

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

================================================================================
MSG: geometry_msgs/PoseStamped
# A Pose with reference coordinate frame and timestamp
Header header
Pose pose

================================================================================
MSG: geometry_msgs/Pose
# A representation of pose in free space, composed of position and orientation. 
Point position
Quaternion orientation

================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z

================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x
float64 y
float64 z
float64 w
")



(provide :boustrophedon_msgs/ConvertPlanToPath "3a9fa9a5fc158705104b513251035497")


