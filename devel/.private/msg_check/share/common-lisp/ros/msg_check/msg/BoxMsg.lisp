; Auto-generated. Do not edit!


(cl:in-package msg_check-msg)


;//! \htmlinclude BoxMsg.msg.html

(cl:defclass <BoxMsg> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (dimension
    :reader dimension
    :initarg :dimension
    :type geometry_msgs-msg:Vector3
    :initform (cl:make-instance 'geometry_msgs-msg:Vector3))
   (mass
    :reader mass
    :initarg :mass
    :type cl:float
    :initform 0.0))
)

(cl:defclass BoxMsg (<BoxMsg>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <BoxMsg>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'BoxMsg)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name msg_check-msg:<BoxMsg> is deprecated: use msg_check-msg:BoxMsg instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <BoxMsg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader msg_check-msg:header-val is deprecated.  Use msg_check-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'dimension-val :lambda-list '(m))
(cl:defmethod dimension-val ((m <BoxMsg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader msg_check-msg:dimension-val is deprecated.  Use msg_check-msg:dimension instead.")
  (dimension m))

(cl:ensure-generic-function 'mass-val :lambda-list '(m))
(cl:defmethod mass-val ((m <BoxMsg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader msg_check-msg:mass-val is deprecated.  Use msg_check-msg:mass instead.")
  (mass m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <BoxMsg>) ostream)
  "Serializes a message object of type '<BoxMsg>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'dimension) ostream)
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'mass))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <BoxMsg>) istream)
  "Deserializes a message object of type '<BoxMsg>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'dimension) istream)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'mass) (roslisp-utils:decode-double-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<BoxMsg>)))
  "Returns string type for a message object of type '<BoxMsg>"
  "msg_check/BoxMsg")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'BoxMsg)))
  "Returns string type for a message object of type 'BoxMsg"
  "msg_check/BoxMsg")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<BoxMsg>)))
  "Returns md5sum for a message object of type '<BoxMsg>"
  "b50fa110ec2c02c572817edb20c0a4c9")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'BoxMsg)))
  "Returns md5sum for a message object of type 'BoxMsg"
  "b50fa110ec2c02c572817edb20c0a4c9")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<BoxMsg>)))
  "Returns full string definition for message of type '<BoxMsg>"
  (cl:format cl:nil "Header header~%geometry_msgs/Vector3 dimension~%float64 mass~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'BoxMsg)))
  "Returns full string definition for message of type 'BoxMsg"
  (cl:format cl:nil "Header header~%geometry_msgs/Vector3 dimension~%float64 mass~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <BoxMsg>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'dimension))
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <BoxMsg>))
  "Converts a ROS message object to a list"
  (cl:list 'BoxMsg
    (cl:cons ':header (header msg))
    (cl:cons ':dimension (dimension msg))
    (cl:cons ':mass (mass msg))
))
