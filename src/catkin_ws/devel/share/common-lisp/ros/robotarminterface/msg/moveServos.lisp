; Auto-generated. Do not edit!


(cl:in-package robotarminterface-msg)


;//! \htmlinclude moveServos.msg.html

(cl:defclass <moveServos> (roslisp-msg-protocol:ros-message)
  ((servos
    :reader servos
    :initarg :servos
    :type (cl:vector robotarminterface-msg:servoPosition)
   :initform (cl:make-array 0 :element-type 'robotarminterface-msg:servoPosition :initial-element (cl:make-instance 'robotarminterface-msg:servoPosition)))
   (time
    :reader time
    :initarg :time
    :type cl:integer
    :initform 0))
)

(cl:defclass moveServos (<moveServos>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <moveServos>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'moveServos)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name robotarminterface-msg:<moveServos> is deprecated: use robotarminterface-msg:moveServos instead.")))

(cl:ensure-generic-function 'servos-val :lambda-list '(m))
(cl:defmethod servos-val ((m <moveServos>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader robotarminterface-msg:servos-val is deprecated.  Use robotarminterface-msg:servos instead.")
  (servos m))

(cl:ensure-generic-function 'time-val :lambda-list '(m))
(cl:defmethod time-val ((m <moveServos>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader robotarminterface-msg:time-val is deprecated.  Use robotarminterface-msg:time instead.")
  (time m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <moveServos>) ostream)
  "Serializes a message object of type '<moveServos>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'servos))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'servos))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'time)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'time)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'time)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'time)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <moveServos>) istream)
  "Deserializes a message object of type '<moveServos>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'servos) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'servos)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'robotarminterface-msg:servoPosition))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'time)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'time)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'time)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'time)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<moveServos>)))
  "Returns string type for a message object of type '<moveServos>"
  "robotarminterface/moveServos")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'moveServos)))
  "Returns string type for a message object of type 'moveServos"
  "robotarminterface/moveServos")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<moveServos>)))
  "Returns md5sum for a message object of type '<moveServos>"
  "4a71efd697b71e458b626f0c557f215a")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'moveServos)))
  "Returns md5sum for a message object of type 'moveServos"
  "4a71efd697b71e458b626f0c557f215a")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<moveServos>)))
  "Returns full string definition for message of type '<moveServos>"
  (cl:format cl:nil "servoPosition[] servos~%uint32 time~%~%================================================================================~%MSG: robotarminterface/servoPosition~%uint32 servoId~%int32 position~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'moveServos)))
  "Returns full string definition for message of type 'moveServos"
  (cl:format cl:nil "servoPosition[] servos~%uint32 time~%~%================================================================================~%MSG: robotarminterface/servoPosition~%uint32 servoId~%int32 position~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <moveServos>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'servos) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <moveServos>))
  "Converts a ROS message object to a list"
  (cl:list 'moveServos
    (cl:cons ':servos (servos msg))
    (cl:cons ':time (time msg))
))
