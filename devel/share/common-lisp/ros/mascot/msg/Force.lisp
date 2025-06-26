; Auto-generated. Do not edit!


(cl:in-package mascot-msg)


;//! \htmlinclude Force.msg.html

(cl:defclass <Force> (roslisp-msg-protocol:ros-message)
  ((force
    :reader force
    :initarg :force
    :type geometry_msgs-msg:Vector3
    :initform (cl:make-instance 'geometry_msgs-msg:Vector3)))
)

(cl:defclass Force (<Force>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Force>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Force)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name mascot-msg:<Force> is deprecated: use mascot-msg:Force instead.")))

(cl:ensure-generic-function 'force-val :lambda-list '(m))
(cl:defmethod force-val ((m <Force>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mascot-msg:force-val is deprecated.  Use mascot-msg:force instead.")
  (force m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Force>) ostream)
  "Serializes a message object of type '<Force>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'force) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Force>) istream)
  "Deserializes a message object of type '<Force>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'force) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Force>)))
  "Returns string type for a message object of type '<Force>"
  "mascot/Force")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Force)))
  "Returns string type for a message object of type 'Force"
  "mascot/Force")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Force>)))
  "Returns md5sum for a message object of type '<Force>"
  "d2c6d945b64e1a5c23ef82706f0df5c7")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Force)))
  "Returns md5sum for a message object of type 'Force"
  "d2c6d945b64e1a5c23ef82706f0df5c7")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Force>)))
  "Returns full string definition for message of type '<Force>"
  (cl:format cl:nil "geometry_msgs/Vector3  force~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Force)))
  "Returns full string definition for message of type 'Force"
  (cl:format cl:nil "geometry_msgs/Vector3  force~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Force>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'force))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Force>))
  "Converts a ROS message object to a list"
  (cl:list 'Force
    (cl:cons ':force (force msg))
))
