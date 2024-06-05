; Auto-generated. Do not edit!


(cl:in-package boustrophedon_msgs-msg)


;//! \htmlinclude StripingPoint.msg.html

(cl:defclass <StripingPoint> (roslisp-msg-protocol:ros-message)
  ((point
    :reader point
    :initarg :point
    :type geometry_msgs-msg:Point
    :initform (cl:make-instance 'geometry_msgs-msg:Point))
   (type
    :reader type
    :initarg :type
    :type cl:fixnum
    :initform 0))
)

(cl:defclass StripingPoint (<StripingPoint>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <StripingPoint>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'StripingPoint)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name boustrophedon_msgs-msg:<StripingPoint> is deprecated: use boustrophedon_msgs-msg:StripingPoint instead.")))

(cl:ensure-generic-function 'point-val :lambda-list '(m))
(cl:defmethod point-val ((m <StripingPoint>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader boustrophedon_msgs-msg:point-val is deprecated.  Use boustrophedon_msgs-msg:point instead.")
  (point m))

(cl:ensure-generic-function 'type-val :lambda-list '(m))
(cl:defmethod type-val ((m <StripingPoint>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader boustrophedon_msgs-msg:type-val is deprecated.  Use boustrophedon_msgs-msg:type instead.")
  (type m))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<StripingPoint>)))
    "Constants for message type '<StripingPoint>"
  '((:OUTLINE . 0)
    (:STRIPE_START . 1)
    (:STRIPE_END . 2)
    (:STRIPE_INTERMEDIATE . 3))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'StripingPoint)))
    "Constants for message type 'StripingPoint"
  '((:OUTLINE . 0)
    (:STRIPE_START . 1)
    (:STRIPE_END . 2)
    (:STRIPE_INTERMEDIATE . 3))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <StripingPoint>) ostream)
  "Serializes a message object of type '<StripingPoint>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'point) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'type)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <StripingPoint>) istream)
  "Deserializes a message object of type '<StripingPoint>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'point) istream)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'type)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<StripingPoint>)))
  "Returns string type for a message object of type '<StripingPoint>"
  "boustrophedon_msgs/StripingPoint")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'StripingPoint)))
  "Returns string type for a message object of type 'StripingPoint"
  "boustrophedon_msgs/StripingPoint")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<StripingPoint>)))
  "Returns md5sum for a message object of type '<StripingPoint>"
  "e38799cac5b64d59df88339f62f9d457")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'StripingPoint)))
  "Returns md5sum for a message object of type 'StripingPoint"
  "e38799cac5b64d59df88339f62f9d457")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<StripingPoint>)))
  "Returns full string definition for message of type '<StripingPoint>"
  (cl:format cl:nil "geometry_msgs/Point point~%uint8 type~%~%uint8 OUTLINE = 0~%uint8 STRIPE_START = 1~%uint8 STRIPE_END = 2~%uint8 STRIPE_INTERMEDIATE = 3~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'StripingPoint)))
  "Returns full string definition for message of type 'StripingPoint"
  (cl:format cl:nil "geometry_msgs/Point point~%uint8 type~%~%uint8 OUTLINE = 0~%uint8 STRIPE_START = 1~%uint8 STRIPE_END = 2~%uint8 STRIPE_INTERMEDIATE = 3~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <StripingPoint>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'point))
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <StripingPoint>))
  "Converts a ROS message object to a list"
  (cl:list 'StripingPoint
    (cl:cons ':point (point msg))
    (cl:cons ':type (type msg))
))
