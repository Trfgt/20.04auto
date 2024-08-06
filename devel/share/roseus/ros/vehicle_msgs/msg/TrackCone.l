;; Auto-generated. Do not edit!


(when (boundp 'vehicle_msgs::TrackCone)
  (if (not (find-package "VEHICLE_MSGS"))
    (make-package "VEHICLE_MSGS"))
  (shadow 'TrackCone (find-package "VEHICLE_MSGS")))
(unless (find-package "VEHICLE_MSGS::TRACKCONE")
  (make-package "VEHICLE_MSGS::TRACKCONE"))

(in-package "ROS")
;;//! \htmlinclude TrackCone.msg.html


(defclass vehicle_msgs::TrackCone
  :super ros::object
  :slots (_x _y _type ))

(defmethod vehicle_msgs::TrackCone
  (:init
   (&key
    ((:x __x) 0.0)
    ((:y __y) 0.0)
    ((:type __type) "")
    )
   (send-super :init)
   (setq _x (float __x))
   (setq _y (float __y))
   (setq _type (string __type))
   self)
  (:x
   (&optional __x)
   (if __x (setq _x __x)) _x)
  (:y
   (&optional __y)
   (if __y (setq _y __y)) _y)
  (:type
   (&optional __type)
   (if __type (setq _type __type)) _type)
  (:serialization-length
   ()
   (+
    ;; float64 _x
    8
    ;; float64 _y
    8
    ;; string _type
    4 (length _type)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64 _x
       (sys::poke _x (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _y
       (sys::poke _y (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; string _type
       (write-long (length _type) s) (princ _type s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64 _x
     (setq _x (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _y
     (setq _y (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; string _type
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _type (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(setf (get vehicle_msgs::TrackCone :md5sum-) "e51ab74afad278235eb99632f6b28f18")
(setf (get vehicle_msgs::TrackCone :datatype-) "vehicle_msgs/TrackCone")
(setf (get vehicle_msgs::TrackCone :definition-)
      "float64 x
float64 y

string type
")



(provide :vehicle_msgs/TrackCone "e51ab74afad278235eb99632f6b28f18")


