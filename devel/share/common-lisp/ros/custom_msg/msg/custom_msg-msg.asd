
(cl:in-package :asdf)

(defsystem "custom_msg-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg
)
  :components ((:file "_package")
    (:file "PointArray_msg" :depends-on ("_package_PointArray_msg"))
    (:file "_package_PointArray_msg" :depends-on ("_package"))
  ))