; Auto-generated. Do not edit!


(cl:in-package mascot-msg)


;//! \htmlinclude Gain.msg.html

(cl:defclass <Gain> (roslisp-msg-protocol:ros-message)
  ((k_p
    :reader k_p
    :initarg :k_p
    :type cl:float
    :initform 0.0)
   (k_d
    :reader k_d
    :initarg :k_d
    :type cl:float
    :initform 0.0)
   (k_i
    :reader k_i
    :initarg :k_i
    :type cl:float
    :initform 0.0))
)

(cl:defclass Gain (<Gain>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Gain>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Gain)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name mascot-msg:<Gain> is deprecated: use mascot-msg:Gain instead.")))

(cl:ensure-generic-function 'k_p-val :lambda-list '(m))
(cl:defmethod k_p-val ((m <Gain>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mascot-msg:k_p-val is deprecated.  Use mascot-msg:k_p instead.")
  (k_p m))

(cl:ensure-generic-function 'k_d-val :lambda-list '(m))
(cl:defmethod k_d-val ((m <Gain>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mascot-msg:k_d-val is deprecated.  Use mascot-msg:k_d instead.")
  (k_d m))

(cl:ensure-generic-function 'k_i-val :lambda-list '(m))
(cl:defmethod k_i-val ((m <Gain>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mascot-msg:k_i-val is deprecated.  Use mascot-msg:k_i instead.")
  (k_i m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Gain>) ostream)
  "Serializes a message object of type '<Gain>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'k_p))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'k_d))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'k_i))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Gain>) istream)
  "Deserializes a message object of type '<Gain>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'k_p) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'k_d) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'k_i) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Gain>)))
  "Returns string type for a message object of type '<Gain>"
  "mascot/Gain")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Gain)))
  "Returns string type for a message object of type 'Gain"
  "mascot/Gain")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Gain>)))
  "Returns md5sum for a message object of type '<Gain>"
  "42218fccfcf87f05cc02314f682ea6ae")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Gain)))
  "Returns md5sum for a message object of type 'Gain"
  "42218fccfcf87f05cc02314f682ea6ae")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Gain>)))
  "Returns full string definition for message of type '<Gain>"
  (cl:format cl:nil "float32 k_p~%float32 k_d~%float32 k_i~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Gain)))
  "Returns full string definition for message of type 'Gain"
  (cl:format cl:nil "float32 k_p~%float32 k_d~%float32 k_i~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Gain>))
  (cl:+ 0
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Gain>))
  "Converts a ROS message object to a list"
  (cl:list 'Gain
    (cl:cons ':k_p (k_p msg))
    (cl:cons ':k_d (k_d msg))
    (cl:cons ':k_i (k_i msg))
))
