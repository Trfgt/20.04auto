;; Auto-generated. Do not edit!


(when (boundp 'custom_msg::PointArray_msg)
  (if (not (find-package "CUSTOM_MSG"))
    (make-package "CUSTOM_MSG"))
  (shadow 'PointArray_msg (find-package "CUSTOM_MSG")))
(unless (find-package "CUSTOM_MSG::POINTARRAY_MSG")
  (make-package "CUSTOM_MSG::POINTARRAY_MSG"))

(in-package "ROS")
;;//! \htmlinclude PointArray_msg.msg.html
(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))


(defclass custom_msg::PointArray_msg
  :super ros::object
  :slots (_array ))

(defmethod custom_msg::PointArray_msg
  (:init
   (&key
    ((:array __array) ())
    )
   (send-super :init)
   (setq _array __array)
   self)
  (:array
   (&rest __array)
   (if (keywordp (car __array))
       (send* _array __array)
     (progn
       (if __array (setq _array (car __array)))
       _array)))
  (:serialization-length
   ()
   (+
    ;; geometry_msgs/Point[] _array
    (apply #'+ (send-all _array :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; geometry_msgs/Point[] _array
     (write-long (length _array) s)
     (dolist (elem _array)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; geometry_msgs/Point[] _array
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _array (let (r) (dotimes (i n) (push (instance geometry_msgs::Point :init) r)) r))
     (dolist (elem- _array)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get custom_msg::PointArray_msg :md5sum-) "11842a6e8a5f0ee08a3522f9fd5f1fd5")
(setf (get custom_msg::PointArray_msg :datatype-) "custom_msg/PointArray_msg")
(setf (get custom_msg::PointArray_msg :definition-)
      "geometry_msgs/Point[] array 

================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z

")



(provide :custom_msg/PointArray_msg "11842a6e8a5f0ee08a3522f9fd5f1fd5")


