;; Auto-generated. Do not edit!


(when (boundp 'msg_check::BoxMsg)
  (if (not (find-package "MSG_CHECK"))
    (make-package "MSG_CHECK"))
  (shadow 'BoxMsg (find-package "MSG_CHECK")))
(unless (find-package "MSG_CHECK::BOXMSG")
  (make-package "MSG_CHECK::BOXMSG"))

(in-package "ROS")
;;//! \htmlinclude BoxMsg.msg.html
(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass msg_check::BoxMsg
  :super ros::object
  :slots (_header _dimension _mass ))

(defmethod msg_check::BoxMsg
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:dimension __dimension) (instance geometry_msgs::Vector3 :init))
    ((:mass __mass) 0.0)
    )
   (send-super :init)
   (setq _header __header)
   (setq _dimension __dimension)
   (setq _mass (float __mass))
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:dimension
   (&rest __dimension)
   (if (keywordp (car __dimension))
       (send* _dimension __dimension)
     (progn
       (if __dimension (setq _dimension (car __dimension)))
       _dimension)))
  (:mass
   (&optional __mass)
   (if __mass (setq _mass __mass)) _mass)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; geometry_msgs/Vector3 _dimension
    (send _dimension :serialization-length)
    ;; float64 _mass
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; geometry_msgs/Vector3 _dimension
       (send _dimension :serialize s)
     ;; float64 _mass
       (sys::poke _mass (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; geometry_msgs/Vector3 _dimension
     (send _dimension :deserialize buf ptr-) (incf ptr- (send _dimension :serialization-length))
   ;; float64 _mass
     (setq _mass (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(setf (get msg_check::BoxMsg :md5sum-) "b50fa110ec2c02c572817edb20c0a4c9")
(setf (get msg_check::BoxMsg :datatype-) "msg_check/BoxMsg")
(setf (get msg_check::BoxMsg :definition-)
      "Header header
geometry_msgs/Vector3 dimension
float64 mass
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
# 0: no frame
# 1: global frame
string frame_id

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
")



(provide :msg_check/BoxMsg "b50fa110ec2c02c572817edb20c0a4c9")


