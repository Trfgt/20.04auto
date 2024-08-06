;; Auto-generated. Do not edit!


(when (boundp 'vehicle_msgs::Command)
  (if (not (find-package "VEHICLE_MSGS"))
    (make-package "VEHICLE_MSGS"))
  (shadow 'Command (find-package "VEHICLE_MSGS")))
(unless (find-package "VEHICLE_MSGS::COMMAND")
  (make-package "VEHICLE_MSGS::COMMAND"))

(in-package "ROS")
;;//! \htmlinclude Command.msg.html


(defclass vehicle_msgs::Command
  :super ros::object
  :slots (_theta_l _theta_r _torque_fl _torque_fr _torque_rl _torque_rr ))

(defmethod vehicle_msgs::Command
  (:init
   (&key
    ((:theta_l __theta_l) 0.0)
    ((:theta_r __theta_r) 0.0)
    ((:torque_fl __torque_fl) 0.0)
    ((:torque_fr __torque_fr) 0.0)
    ((:torque_rl __torque_rl) 0.0)
    ((:torque_rr __torque_rr) 0.0)
    )
   (send-super :init)
   (setq _theta_l (float __theta_l))
   (setq _theta_r (float __theta_r))
   (setq _torque_fl (float __torque_fl))
   (setq _torque_fr (float __torque_fr))
   (setq _torque_rl (float __torque_rl))
   (setq _torque_rr (float __torque_rr))
   self)
  (:theta_l
   (&optional __theta_l)
   (if __theta_l (setq _theta_l __theta_l)) _theta_l)
  (:theta_r
   (&optional __theta_r)
   (if __theta_r (setq _theta_r __theta_r)) _theta_r)
  (:torque_fl
   (&optional __torque_fl)
   (if __torque_fl (setq _torque_fl __torque_fl)) _torque_fl)
  (:torque_fr
   (&optional __torque_fr)
   (if __torque_fr (setq _torque_fr __torque_fr)) _torque_fr)
  (:torque_rl
   (&optional __torque_rl)
   (if __torque_rl (setq _torque_rl __torque_rl)) _torque_rl)
  (:torque_rr
   (&optional __torque_rr)
   (if __torque_rr (setq _torque_rr __torque_rr)) _torque_rr)
  (:serialization-length
   ()
   (+
    ;; float64 _theta_l
    8
    ;; float64 _theta_r
    8
    ;; float64 _torque_fl
    8
    ;; float64 _torque_fr
    8
    ;; float64 _torque_rl
    8
    ;; float64 _torque_rr
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64 _theta_l
       (sys::poke _theta_l (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _theta_r
       (sys::poke _theta_r (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _torque_fl
       (sys::poke _torque_fl (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _torque_fr
       (sys::poke _torque_fr (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _torque_rl
       (sys::poke _torque_rl (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _torque_rr
       (sys::poke _torque_rr (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64 _theta_l
     (setq _theta_l (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _theta_r
     (setq _theta_r (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _torque_fl
     (setq _torque_fl (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _torque_fr
     (setq _torque_fr (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _torque_rl
     (setq _torque_rl (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _torque_rr
     (setq _torque_rr (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(setf (get vehicle_msgs::Command :md5sum-) "5f166b646303bdf36237e2c962a86181")
(setf (get vehicle_msgs::Command :datatype-) "vehicle_msgs/Command")
(setf (get vehicle_msgs::Command :definition-)
      "float64 theta_l
float64 theta_r
float64 torque_fl
float64 torque_fr
float64 torque_rl
float64 torque_rr
")



(provide :vehicle_msgs/Command "5f166b646303bdf36237e2c962a86181")


