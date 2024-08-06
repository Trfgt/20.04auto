;; Auto-generated. Do not edit!


(when (boundp 'tracking_msg::TrackingObject)
  (if (not (find-package "TRACKING_MSG"))
    (make-package "TRACKING_MSG"))
  (shadow 'TrackingObject (find-package "TRACKING_MSG")))
(unless (find-package "TRACKING_MSG::TRACKINGOBJECT")
  (make-package "TRACKING_MSG::TRACKINGOBJECT"))

(in-package "ROS")
;;//! \htmlinclude TrackingObject.msg.html
(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))
(if (not (find-package "VISUALIZATION_MSGS"))
  (ros::roseus-add-msgs "visualization_msgs"))


(defclass tracking_msg::TrackingObject
  :super ros::object
  :slots (_header _id _name _bbox2d _bev _bbox _point _velocity _state _type_id _ttc ))

(defmethod tracking_msg::TrackingObject
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:id __id) 0)
    ((:name __name) (instance std_msgs::String :init))
    ((:bbox2d __bbox2d) (instance std_msgs::Int32MultiArray :init))
    ((:bev __bev) (instance std_msgs::Float32MultiArray :init))
    ((:bbox __bbox) (instance visualization_msgs::Marker :init))
    ((:point __point) (instance geometry_msgs::Point :init))
    ((:velocity __velocity) (instance geometry_msgs::Point :init))
    ((:state __state) 0)
    ((:type_id __type_id) 0)
    ((:ttc __ttc) 0.0)
    )
   (send-super :init)
   (setq _header __header)
   (setq _id (round __id))
   (setq _name __name)
   (setq _bbox2d __bbox2d)
   (setq _bev __bev)
   (setq _bbox __bbox)
   (setq _point __point)
   (setq _velocity __velocity)
   (setq _state (round __state))
   (setq _type_id (round __type_id))
   (setq _ttc (float __ttc))
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:id
   (&optional __id)
   (if __id (setq _id __id)) _id)
  (:name
   (&rest __name)
   (if (keywordp (car __name))
       (send* _name __name)
     (progn
       (if __name (setq _name (car __name)))
       _name)))
  (:bbox2d
   (&rest __bbox2d)
   (if (keywordp (car __bbox2d))
       (send* _bbox2d __bbox2d)
     (progn
       (if __bbox2d (setq _bbox2d (car __bbox2d)))
       _bbox2d)))
  (:bev
   (&rest __bev)
   (if (keywordp (car __bev))
       (send* _bev __bev)
     (progn
       (if __bev (setq _bev (car __bev)))
       _bev)))
  (:bbox
   (&rest __bbox)
   (if (keywordp (car __bbox))
       (send* _bbox __bbox)
     (progn
       (if __bbox (setq _bbox (car __bbox)))
       _bbox)))
  (:point
   (&rest __point)
   (if (keywordp (car __point))
       (send* _point __point)
     (progn
       (if __point (setq _point (car __point)))
       _point)))
  (:velocity
   (&rest __velocity)
   (if (keywordp (car __velocity))
       (send* _velocity __velocity)
     (progn
       (if __velocity (setq _velocity (car __velocity)))
       _velocity)))
  (:state
   (&optional __state)
   (if __state (setq _state __state)) _state)
  (:type_id
   (&optional __type_id)
   (if __type_id (setq _type_id __type_id)) _type_id)
  (:ttc
   (&optional __ttc)
   (if __ttc (setq _ttc __ttc)) _ttc)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; uint32 _id
    4
    ;; std_msgs/String _name
    (send _name :serialization-length)
    ;; std_msgs/Int32MultiArray _bbox2d
    (send _bbox2d :serialization-length)
    ;; std_msgs/Float32MultiArray _bev
    (send _bev :serialization-length)
    ;; visualization_msgs/Marker _bbox
    (send _bbox :serialization-length)
    ;; geometry_msgs/Point _point
    (send _point :serialization-length)
    ;; geometry_msgs/Point _velocity
    (send _velocity :serialization-length)
    ;; uint32 _state
    4
    ;; uint32 _type_id
    4
    ;; float32 _ttc
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; uint32 _id
       (write-long _id s)
     ;; std_msgs/String _name
       (send _name :serialize s)
     ;; std_msgs/Int32MultiArray _bbox2d
       (send _bbox2d :serialize s)
     ;; std_msgs/Float32MultiArray _bev
       (send _bev :serialize s)
     ;; visualization_msgs/Marker _bbox
       (send _bbox :serialize s)
     ;; geometry_msgs/Point _point
       (send _point :serialize s)
     ;; geometry_msgs/Point _velocity
       (send _velocity :serialize s)
     ;; uint32 _state
       (write-long _state s)
     ;; uint32 _type_id
       (write-long _type_id s)
     ;; float32 _ttc
       (sys::poke _ttc (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; uint32 _id
     (setq _id (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; std_msgs/String _name
     (send _name :deserialize buf ptr-) (incf ptr- (send _name :serialization-length))
   ;; std_msgs/Int32MultiArray _bbox2d
     (send _bbox2d :deserialize buf ptr-) (incf ptr- (send _bbox2d :serialization-length))
   ;; std_msgs/Float32MultiArray _bev
     (send _bev :deserialize buf ptr-) (incf ptr- (send _bev :serialization-length))
   ;; visualization_msgs/Marker _bbox
     (send _bbox :deserialize buf ptr-) (incf ptr- (send _bbox :serialization-length))
   ;; geometry_msgs/Point _point
     (send _point :deserialize buf ptr-) (incf ptr- (send _point :serialization-length))
   ;; geometry_msgs/Point _velocity
     (send _velocity :deserialize buf ptr-) (incf ptr- (send _velocity :serialization-length))
   ;; uint32 _state
     (setq _state (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; uint32 _type_id
     (setq _type_id (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; float32 _ttc
     (setq _ttc (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get tracking_msg::TrackingObject :md5sum-) "dbecf0bde23016151623d97fb0e58ed7")
(setf (get tracking_msg::TrackingObject :datatype-) "tracking_msg/TrackingObject")
(setf (get tracking_msg::TrackingObject :definition-)
      "Header header

uint32 id
std_msgs/String name # name id
std_msgs/Int32MultiArray bbox2d # bounding box [x,y,x,y]
std_msgs/Float32MultiArray bev # bird eye view , [x,y,x,y,x,y,x,y]
visualization_msgs/Marker bbox # bounding box
geometry_msgs/Point point # object's closest pt (m)
geometry_msgs/Point velocity #x,y,z velocity (m/s)
uint32 state # stationary(0) or moving(1)
uint32 type_id # what is type? with the document
float32 ttc # time to collison
#uint32 inpath # not in my path(0) or in my path(1)

================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id

================================================================================
MSG: std_msgs/String
string data

================================================================================
MSG: std_msgs/Int32MultiArray
# Please look at the MultiArrayLayout message definition for
# documentation on all multiarrays.

MultiArrayLayout  layout        # specification of data layout
int32[]           data          # array of data


================================================================================
MSG: std_msgs/MultiArrayLayout
# The multiarray declares a generic multi-dimensional array of a
# particular data type.  Dimensions are ordered from outer most
# to inner most.

MultiArrayDimension[] dim # Array of dimension properties
uint32 data_offset        # padding elements at front of data

# Accessors should ALWAYS be written in terms of dimension stride
# and specified outer-most dimension first.
# 
# multiarray(i,j,k) = data[data_offset + dim_stride[1]*i + dim_stride[2]*j + k]
#
# A standard, 3-channel 640x480 image with interleaved color channels
# would be specified as:
#
# dim[0].label  = \"height\"
# dim[0].size   = 480
# dim[0].stride = 3*640*480 = 921600  (note dim[0] stride is just size of image)
# dim[1].label  = \"width\"
# dim[1].size   = 640
# dim[1].stride = 3*640 = 1920
# dim[2].label  = \"channel\"
# dim[2].size   = 3
# dim[2].stride = 3
#
# multiarray(i,j,k) refers to the ith row, jth column, and kth channel.

================================================================================
MSG: std_msgs/MultiArrayDimension
string label   # label of given dimension
uint32 size    # size of given dimension (in type units)
uint32 stride  # stride of given dimension
================================================================================
MSG: std_msgs/Float32MultiArray
# Please look at the MultiArrayLayout message definition for
# documentation on all multiarrays.

MultiArrayLayout  layout        # specification of data layout
float32[]         data          # array of data


================================================================================
MSG: visualization_msgs/Marker
# See http://www.ros.org/wiki/rviz/DisplayTypes/Marker and http://www.ros.org/wiki/rviz/Tutorials/Markers%3A%20Basic%20Shapes for more information on using this message with rviz

uint8 ARROW=0
uint8 CUBE=1
uint8 SPHERE=2
uint8 CYLINDER=3
uint8 LINE_STRIP=4
uint8 LINE_LIST=5
uint8 CUBE_LIST=6
uint8 SPHERE_LIST=7
uint8 POINTS=8
uint8 TEXT_VIEW_FACING=9
uint8 MESH_RESOURCE=10
uint8 TRIANGLE_LIST=11

uint8 ADD=0
uint8 MODIFY=0
uint8 DELETE=2
uint8 DELETEALL=3

Header header                        # header for time/frame information
string ns                            # Namespace to place this object in... used in conjunction with id to create a unique name for the object
int32 id 		                         # object ID useful in conjunction with the namespace for manipulating and deleting the object later
int32 type 		                       # Type of object
int32 action 	                       # 0 add/modify an object, 1 (deprecated), 2 deletes an object, 3 deletes all objects
geometry_msgs/Pose pose                 # Pose of the object
geometry_msgs/Vector3 scale             # Scale of the object 1,1,1 means default (usually 1 meter square)
std_msgs/ColorRGBA color             # Color [0.0-1.0]
duration lifetime                    # How long the object should last before being automatically deleted.  0 means forever
bool frame_locked                    # If this marker should be frame-locked, i.e. retransformed into its frame every timestep

#Only used if the type specified has some use for them (eg. POINTS, LINE_STRIP, ...)
geometry_msgs/Point[] points
#Only used if the type specified has some use for them (eg. POINTS, LINE_STRIP, ...)
#number of colors must either be 0 or equal to the number of points
#NOTE: alpha is not yet used
std_msgs/ColorRGBA[] colors

# NOTE: only used for text markers
string text

# NOTE: only used for MESH_RESOURCE markers
string mesh_resource
bool mesh_use_embedded_materials

================================================================================
MSG: geometry_msgs/Pose
# A representation of pose in free space, composed of position and orientation. 
Point position
Quaternion orientation

================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z

================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x
float64 y
float64 z
float64 w

================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space. 
# It is only meant to represent a direction. Therefore, it does not
# make sense to apply a translation to it (e.g., when applying a 
# generic rigid transformation to a Vector3, tf2 will only apply the
# rotation). If you want your data to be translatable too, use the
# geometry_msgs/Point message instead.

float64 x
float64 y
float64 z
================================================================================
MSG: std_msgs/ColorRGBA
float32 r
float32 g
float32 b
float32 a

")



(provide :tracking_msg/TrackingObject "dbecf0bde23016151623d97fb0e58ed7")


