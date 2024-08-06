
(cl:in-package :asdf)

(defsystem "vehicle_msgs-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :std_msgs-msg
)
  :components ((:file "_package")
    (:file "Command" :depends-on ("_package_Command"))
    (:file "_package_Command" :depends-on ("_package"))
    (:file "Track" :depends-on ("_package_Track"))
    (:file "_package_Track" :depends-on ("_package"))
    (:file "TrackCone" :depends-on ("_package_TrackCone"))
    (:file "_package_TrackCone" :depends-on ("_package"))
    (:file "Waypoint" :depends-on ("_package_Waypoint"))
    (:file "_package_Waypoint" :depends-on ("_package"))
    (:file "WaypointsArray" :depends-on ("_package_WaypointsArray"))
    (:file "_package_WaypointsArray" :depends-on ("_package"))
  ))