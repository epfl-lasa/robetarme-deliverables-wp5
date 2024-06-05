
(cl:in-package :asdf)

(defsystem "boustrophedon_msgs-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :actionlib_msgs-msg
               :geometry_msgs-msg
               :std_msgs-msg
)
  :components ((:file "_package")
    (:file "PlanMowingPathAction" :depends-on ("_package_PlanMowingPathAction"))
    (:file "_package_PlanMowingPathAction" :depends-on ("_package"))
    (:file "PlanMowingPathActionFeedback" :depends-on ("_package_PlanMowingPathActionFeedback"))
    (:file "_package_PlanMowingPathActionFeedback" :depends-on ("_package"))
    (:file "PlanMowingPathActionGoal" :depends-on ("_package_PlanMowingPathActionGoal"))
    (:file "_package_PlanMowingPathActionGoal" :depends-on ("_package"))
    (:file "PlanMowingPathActionResult" :depends-on ("_package_PlanMowingPathActionResult"))
    (:file "_package_PlanMowingPathActionResult" :depends-on ("_package"))
    (:file "PlanMowingPathFeedback" :depends-on ("_package_PlanMowingPathFeedback"))
    (:file "_package_PlanMowingPathFeedback" :depends-on ("_package"))
    (:file "PlanMowingPathGoal" :depends-on ("_package_PlanMowingPathGoal"))
    (:file "_package_PlanMowingPathGoal" :depends-on ("_package"))
    (:file "PlanMowingPathResult" :depends-on ("_package_PlanMowingPathResult"))
    (:file "_package_PlanMowingPathResult" :depends-on ("_package"))
    (:file "StripingPlan" :depends-on ("_package_StripingPlan"))
    (:file "_package_StripingPlan" :depends-on ("_package"))
    (:file "StripingPoint" :depends-on ("_package_StripingPoint"))
    (:file "_package_StripingPoint" :depends-on ("_package"))
  ))