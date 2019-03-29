; Auto-generated. Do not edit!


(cl:in-package robotarminterface-msg)


;//! \htmlinclude singleServo.msg.html

(cl:defclass <singleServo> (roslisp-msg-protocol:ros-message)
  ((servoId
    :reader servoId
    :initarg :servoId
    :type cl:integer
    :initform 0)
   (position
    :reader position
    :initarg :position
    :type cl:integer
    :initform 0)
   (time
    :reader time
    :initarg :time
    :type cl:integer
    :initform 0))
)

(cl:defclass singleServo (<singleServo>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <singleServo>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'singleServo)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name robotarminterface-msg:<singleServo> is deprecated: use robotarminterface-msg:singleServo instead.")))

(cl:ensure-generic-function 'servoId-val :lambda-list '(m))
(cl:defmethod servoId-val ((m <singleServo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader robotarminterface-msg:servoId-val is deprecated.  Use robotarminterface-msg:servoId instead.")
  (servoId m))

(cl:ensure-generic-function 'position-val :lambda-list '(m))
(cl:defmethod position-val ((m <singleServo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader robotarminterface-msg:position-val is deprecated.  Use robotarminterface-msg:position instead.")
  (position m))

(cl:ensure-generic-function 'time-val :lambda-list '(m))
(cl:defmethod time-val ((m <singleServo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader robotarminterface-msg:time-val is deprecated.  Use robotarminterface-msg:time instead.")
  (time m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <singleServo>) ostream)
  "Serializes a message object of type '<singleServo>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'servoId)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'servoId)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'servoId)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'servoId)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'position)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'position)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'position)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'position)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'time)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'time)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'time)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'time)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <singleServo>) istream)
  "Deserializes a message object of type '<singleServo>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'servoId)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'servoId)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'servoId)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'servoId)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'position)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'position)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'position)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'position)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'time)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'time)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'time)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'time)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<singleServo>)))
  "Returns string type for a message object of type '<singleServo>"
  "robotarminterface/singleServo")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'singleServo)))
  "Returns string type for a message object of type 'singleServo"
  "robotarminterface/singleServo")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<singleServo>)))
  "Returns md5sum for a message object of type '<singleServo>"
  "39faf3391f2b3261e444ffb062992407")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'singleServo)))
  "Returns md5sum for a message object of type 'singleServo"
  "39faf3391f2b3261e444ffb062992407")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<singleServo>)))
  "Returns full string definition for message of type '<singleServo>"
  (cl:format cl:nil "uint32 servoId~%uint32 position~%uint32 time~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'singleServo)))
  "Returns full string definition for message of type 'singleServo"
  (cl:format cl:nil "uint32 servoId~%uint32 position~%uint32 time~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <singleServo>))
  (cl:+ 0
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <singleServo>))
  "Converts a ROS message object to a list"
  (cl:list 'singleServo
    (cl:cons ':servoId (servoId msg))
    (cl:cons ':position (position msg))
    (cl:cons ':time (time msg))
))
