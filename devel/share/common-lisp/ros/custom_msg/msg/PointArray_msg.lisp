; Auto-generated. Do not edit!


(cl:in-package custom_msg-msg)


;//! \htmlinclude PointArray_msg.msg.html

(cl:defclass <PointArray_msg> (roslisp-msg-protocol:ros-message)
  ((array
    :reader array
    :initarg :array
    :type (cl:vector geometry_msgs-msg:Point)
   :initform (cl:make-array 0 :element-type 'geometry_msgs-msg:Point :initial-element (cl:make-instance 'geometry_msgs-msg:Point))))
)

(cl:defclass PointArray_msg (<PointArray_msg>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <PointArray_msg>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'PointArray_msg)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name custom_msg-msg:<PointArray_msg> is deprecated: use custom_msg-msg:PointArray_msg instead.")))

(cl:ensure-generic-function 'array-val :lambda-list '(m))
(cl:defmethod array-val ((m <PointArray_msg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader custom_msg-msg:array-val is deprecated.  Use custom_msg-msg:array instead.")
  (array m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <PointArray_msg>) ostream)
  "Serializes a message object of type '<PointArray_msg>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'array))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'array))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <PointArray_msg>) istream)
  "Deserializes a message object of type '<PointArray_msg>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'array) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'array)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'geometry_msgs-msg:Point))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<PointArray_msg>)))
  "Returns string type for a message object of type '<PointArray_msg>"
  "custom_msg/PointArray_msg")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'PointArray_msg)))
  "Returns string type for a message object of type 'PointArray_msg"
  "custom_msg/PointArray_msg")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<PointArray_msg>)))
  "Returns md5sum for a message object of type '<PointArray_msg>"
  "11842a6e8a5f0ee08a3522f9fd5f1fd5")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'PointArray_msg)))
  "Returns md5sum for a message object of type 'PointArray_msg"
  "11842a6e8a5f0ee08a3522f9fd5f1fd5")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<PointArray_msg>)))
  "Returns full string definition for message of type '<PointArray_msg>"
  (cl:format cl:nil "geometry_msgs/Point[] array ~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'PointArray_msg)))
  "Returns full string definition for message of type 'PointArray_msg"
  (cl:format cl:nil "geometry_msgs/Point[] array ~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <PointArray_msg>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'array) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <PointArray_msg>))
  "Converts a ROS message object to a list"
  (cl:list 'PointArray_msg
    (cl:cons ':array (array msg))
))
