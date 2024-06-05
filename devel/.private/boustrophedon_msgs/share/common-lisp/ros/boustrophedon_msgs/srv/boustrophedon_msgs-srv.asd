
(cl:in-package :asdf)

(defsystem "boustrophedon_msgs-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :boustrophedon_msgs-msg
               :nav_msgs-msg
)
  :components ((:file "_package")
    (:file "ConvertPlanToPath" :depends-on ("_package_ConvertPlanToPath"))
    (:file "_package_ConvertPlanToPath" :depends-on ("_package"))
  ))