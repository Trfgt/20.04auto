; Auto-generated. Do not edit!


(cl:in-package vehicle_msgs-msg)


;//! \htmlinclude WaypointsArray.msg.html

(cl:defclass <WaypointsArray> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (waypoints
    :reader waypoints
    :initarg :waypoints
    :type (cl:vector vehicle_msgs-msg:Waypoint)
   :initform (cl:make-array 0 :element-type 'vehicle_msgs-msg:Waypoint :initial-element (cl:make-instance 'vehicle_msgs-msg:Waypoint)))
   (preliminaryLoopClosure
    :reader preliminaryLoopClosure
    :initarg :preliminaryLoopClosure
    :type cl:boolean
    :initform cl:nil)
   (loopClosure
    :reader loopClosure
    :initarg :loopClosure
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass WaypointsArray (<WaypointsArray>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <WaypointsArray>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'WaypointsArray)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name vehicle_msgs-msg:<WaypointsArray> is deprecated: use vehicle_msgs-msg:WaypointsArray instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <WaypointsArray>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader vehicle_msgs-msg:header-val is deprecated.  Use vehicle_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'waypoints-val :lambda-list '(m))
(cl:defmethod waypoints-val ((m <WaypointsArray>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader vehicle_msgs-msg:waypoints-val is deprecated.  Use vehicle_msgs-msg:waypoints instead.")
  (waypoints m))

(cl:ensure-generic-function 'preliminaryLoopClosure-val :lambda-list '(m))
(cl:defmethod preliminaryLoopClosure-val ((m <WaypointsArray>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader vehicle_msgs-msg:preliminaryLoopClosure-val is deprecated.  Use vehicle_msgs-msg:preliminaryLoopClosure instead.")
  (preliminaryLoopClosure m))

(cl:ensure-generic-function 'loopClosure-val :lambda-list '(m))
(cl:defmethod loopClosure-val ((m <WaypointsArray>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader vehicle_msgs-msg:loopClosure-val is deprecated.  Use vehicle_msgs-msg:loopClosure instead.")
  (loopClosure m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <WaypointsArray>) ostream)
  "Serializes a message object of type '<WaypointsArray>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'waypoints))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'waypoints))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'preliminaryLoopClosure) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'loopClosure) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <WaypointsArray>) istream)
  "Deserializes a message object of type '<WaypointsArray>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'waypoints) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'waypoints)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'vehicle_msgs-msg:Waypoint))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
    (cl:setf (cl:slot-value msg 'preliminaryLoopClosure) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'loopClosure) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<WaypointsArray>)))
  "Returns string type for a message object of type '<WaypointsArray>"
  "vehicle_msgs/WaypointsArray")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'WaypointsArray)))
  "Returns string type for a message object of type 'WaypointsArray"
  "vehicle_msgs/WaypointsArray")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<WaypointsArray>)))
  "Returns md5sum for a message object of type '<WaypointsArray>"
  "0473318081887a3740b7424ce44e5b16")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'WaypointsArray)))
  "Returns md5sum for a message object of type 'WaypointsArray"
  "0473318081887a3740b7424ce44e5b16")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<WaypointsArray>)))
  "Returns full string definition for message of type '<WaypointsArray>"
  (cl:format cl:nil "Header header~%~%vehicle_msgs/Waypoint[] waypoints~%~%bool preliminaryLoopClosure~%bool loopClosure~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: vehicle_msgs/Waypoint~%float64 x~%float64 y~%~%float64 id~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'WaypointsArray)))
  "Returns full string definition for message of type 'WaypointsArray"
  (cl:format cl:nil "Header header~%~%vehicle_msgs/Waypoint[] waypoints~%~%bool preliminaryLoopClosure~%bool loopClosure~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: vehicle_msgs/Waypoint~%float64 x~%float64 y~%~%float64 id~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <WaypointsArray>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'waypoints) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
     1
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <WaypointsArray>))
  "Converts a ROS message object to a list"
  (cl:list 'WaypointsArray
    (cl:cons ':header (header msg))
    (cl:cons ':waypoints (waypoints msg))
    (cl:cons ':preliminaryLoopClosure (preliminaryLoopClosure msg))
    (cl:cons ':loopClosure (loopClosure msg))
))
