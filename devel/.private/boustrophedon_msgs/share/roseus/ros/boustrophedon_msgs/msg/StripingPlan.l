;; Auto-generated. Do not edit!


(when (boundp 'boustrophedon_msgs::StripingPlan)
  (if (not (find-package "BOUSTROPHEDON_MSGS"))
    (make-package "BOUSTROPHEDON_MSGS"))
  (shadow 'StripingPlan (find-package "BOUSTROPHEDON_MSGS")))
(unless (find-package "BOUSTROPHEDON_MSGS::STRIPINGPLAN")
  (make-package "BOUSTROPHEDON_MSGS::STRIPINGPLAN"))

(in-package "ROS")
;;//! \htmlinclude StripingPlan.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass boustrophedon_msgs::StripingPlan
  :super ros::object
  :slots (_header _points ))

(defmethod boustrophedon_msgs::StripingPlan
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:points __points) ())
    )
   (send-super :init)
   (setq _header __header)
   (setq _points __points)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:points
   (&rest __points)
   (if (keywordp (car __points))
       (send* _points __points)
     (progn
       (if __points (setq _points (car __points)))
       _points)))
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; boustrophedon_msgs/StripingPoint[] _points
    (apply #'+ (send-all _points :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; boustrophedon_msgs/StripingPoint[] _points
     (write-long (length _points) s)
     (dolist (elem _points)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; boustrophedon_msgs/StripingPoint[] _points
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _points (let (r) (dotimes (i n) (push (instance boustrophedon_msgs::StripingPoint :init) r)) r))
     (dolist (elem- _points)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get boustrophedon_msgs::StripingPlan :md5sum-) "b2838ae4f42498e5013db14343fd340a")
(setf (get boustrophedon_msgs::StripingPlan :datatype-) "boustrophedon_msgs/StripingPlan")
(setf (get boustrophedon_msgs::StripingPlan :definition-)
      "Header header
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

")



(provide :boustrophedon_msgs/StripingPlan "b2838ae4f42498e5013db14343fd340a")


