
(cl:in-package :asdf)

(defsystem "tracking_msg-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg
               :std_msgs-msg
               :visualization_msgs-msg
)
  :components ((:file "_package")
    (:file "TrackingObject" :depends-on ("_package_TrackingObject"))
    (:file "_package_TrackingObject" :depends-on ("_package"))
    (:file "TrackingObjectArray" :depends-on ("_package_TrackingObjectArray"))
    (:file "_package_TrackingObjectArray" :depends-on ("_package"))
  ))