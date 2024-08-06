;; Auto-generated. Do not edit!


(when (boundp 'vehicle_msgs::Track)
  (if (not (find-package "VEHICLE_MSGS"))
    (make-package "VEHICLE_MSGS"))
  (shadow 'Track (find-package "VEHICLE_MSGS")))
(unless (find-package "VEHICLE_MSGS::TRACK")
  (make-package "VEHICLE_MSGS::TRACK"))

(in-package "ROS")
;;//! \htmlinclude Track.msg.html


(defclass vehicle_msgs::Track
  :super ros::object
  :slots (_cones ))

(defmethod vehicle_msgs::Track
  (:init
   (&key
    ((:cones __cones) ())
    )
   (send-super :init)
   (setq _cones __cones)
   self)
  (:cones
   (&rest __cones)
   (if (keywordp (car __cones))
       (send* _cones __cones)
     (progn
       (if __cones (setq _cones (car __cones)))
       _cones)))
  (:serialization-length
   ()
   (+
    ;; vehicle_msgs/TrackCone[] _cones
    (apply #'+ (send-all _cones :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; vehicle_msgs/TrackCone[] _cones
     (write-long (length _cones) s)
     (dolist (elem _cones)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; vehicle_msgs/TrackCone[] _cones
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _cones (let (r) (dotimes (i n) (push (instance vehicle_msgs::TrackCone :init) r)) r))
     (dolist (elem- _cones)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get vehicle_msgs::Track :md5sum-) "1bd255a768d640e676ebd0a7279d0839")
(setf (get vehicle_msgs::Track :datatype-) "vehicle_msgs/Track")
(setf (get vehicle_msgs::Track :definition-)
      "vehicle_msgs/TrackCone[] cones
================================================================================
MSG: vehicle_msgs/TrackCone
float64 x
float64 y

string type
")



(provide :vehicle_msgs/Track "1bd255a768d640e676ebd0a7279d0839")


