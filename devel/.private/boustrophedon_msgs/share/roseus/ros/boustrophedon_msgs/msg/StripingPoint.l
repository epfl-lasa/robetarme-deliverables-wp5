;; Auto-generated. Do not edit!


(when (boundp 'boustrophedon_msgs::StripingPoint)
  (if (not (find-package "BOUSTROPHEDON_MSGS"))
    (make-package "BOUSTROPHEDON_MSGS"))
  (shadow 'StripingPoint (find-package "BOUSTROPHEDON_MSGS")))
(unless (find-package "BOUSTROPHEDON_MSGS::STRIPINGPOINT")
  (make-package "BOUSTROPHEDON_MSGS::STRIPINGPOINT"))

(in-package "ROS")
;;//! \htmlinclude StripingPoint.msg.html
(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))


(intern "*OUTLINE*" (find-package "BOUSTROPHEDON_MSGS::STRIPINGPOINT"))
(shadow '*OUTLINE* (find-package "BOUSTROPHEDON_MSGS::STRIPINGPOINT"))
(defconstant boustrophedon_msgs::StripingPoint::*OUTLINE* 0)
(intern "*STRIPE_START*" (find-package "BOUSTROPHEDON_MSGS::STRIPINGPOINT"))
(shadow '*STRIPE_START* (find-package "BOUSTROPHEDON_MSGS::STRIPINGPOINT"))
(defconstant boustrophedon_msgs::StripingPoint::*STRIPE_START* 1)
(intern "*STRIPE_END*" (find-package "BOUSTROPHEDON_MSGS::STRIPINGPOINT"))
(shadow '*STRIPE_END* (find-package "BOUSTROPHEDON_MSGS::STRIPINGPOINT"))
(defconstant boustrophedon_msgs::StripingPoint::*STRIPE_END* 2)
(intern "*STRIPE_INTERMEDIATE*" (find-package "BOUSTROPHEDON_MSGS::STRIPINGPOINT"))
(shadow '*STRIPE_INTERMEDIATE* (find-package "BOUSTROPHEDON_MSGS::STRIPINGPOINT"))
(defconstant boustrophedon_msgs::StripingPoint::*STRIPE_INTERMEDIATE* 3)

(defun boustrophedon_msgs::StripingPoint-to-symbol (const)
  (cond
        ((= const 0) 'boustrophedon_msgs::StripingPoint::*OUTLINE*)
        ((= const 1) 'boustrophedon_msgs::StripingPoint::*STRIPE_START*)
        ((= const 2) 'boustrophedon_msgs::StripingPoint::*STRIPE_END*)
        ((= const 3) 'boustrophedon_msgs::StripingPoint::*STRIPE_INTERMEDIATE*)
        (t nil)))

(defclass boustrophedon_msgs::StripingPoint
  :super ros::object
  :slots (_point _type ))

(defmethod boustrophedon_msgs::StripingPoint
  (:init
   (&key
    ((:point __point) (instance geometry_msgs::Point :init))
    ((:type __type) 0)
    )
   (send-super :init)
   (setq _point __point)
   (setq _type (round __type))
   self)
  (:point
   (&rest __point)
   (if (keywordp (car __point))
       (send* _point __point)
     (progn
       (if __point (setq _point (car __point)))
       _point)))
  (:type
   (&optional __type)
   (if __type (setq _type __type)) _type)
  (:serialization-length
   ()
   (+
    ;; geometry_msgs/Point _point
    (send _point :serialization-length)
    ;; uint8 _type
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; geometry_msgs/Point _point
       (send _point :serialize s)
     ;; uint8 _type
       (write-byte _type s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; geometry_msgs/Point _point
     (send _point :deserialize buf ptr-) (incf ptr- (send _point :serialization-length))
   ;; uint8 _type
     (setq _type (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;;
   self)
  )

(setf (get boustrophedon_msgs::StripingPoint :md5sum-) "e38799cac5b64d59df88339f62f9d457")
(setf (get boustrophedon_msgs::StripingPoint :datatype-) "boustrophedon_msgs/StripingPoint")
(setf (get boustrophedon_msgs::StripingPoint :definition-)
      "geometry_msgs/Point point
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



(provide :boustrophedon_msgs/StripingPoint "e38799cac5b64d59df88339f62f9d457")


