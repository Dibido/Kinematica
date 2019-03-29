; Auto-generated. Do not edit!


(cl:in-package robotarminterface-msg)


;//! \htmlinclude stopSingleServo.msg.html

(cl:defclass <stopSingleServo> (roslisp-msg-protocol:ros-message)
  ((servoId
    :reader servoId
    :initarg :servoId
    :type cl:integer
    :initform 0))
)

(cl:defclass stopSingleServo (<stopSingleServo>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <stopSingleServo>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'stopSingleServo)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name robotarminterface-msg:<stopSingleServo> is deprecated: use robotarminterface-msg:stopSingleServo instead.")))

(cl:ensure-generic-function 'servoId-val :lambda-list '(m))
(cl:defmethod servoId-val ((m <stopSingleServo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader robotarminterface-msg:servoId-val is deprecated.  Use robotarminterface-msg:servoId instead.")
  (servoId m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <stopSingleServo>) ostream)
  "Serializes a message object of type '<stopSingleServo>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'servoId)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'servoId)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'servoId)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'servoId)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <stopSingleServo>) istream)
  "Deserializes a message object of type '<stopSingleServo>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'servoId)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'servoId)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'servoId)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'servoId)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<stopSingleServo>)))
  "Returns string type for a message object of type '<stopSingleServo>"
  "robotarminterface/stopSingleServo")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'stopSingleServo)))
  "Returns string type for a message object of type 'stopSingleServo"
  "robotarminterface/stopSingleServo")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<stopSingleServo>)))
  "Returns md5sum for a message object of type '<stopSingleServo>"
  "c229a51eeb50adb8a4245c15dbd40200")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'stopSingleServo)))
  "Returns md5sum for a message object of type 'stopSingleServo"
  "c229a51eeb50adb8a4245c15dbd40200")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<stopSingleServo>)))
  "Returns full string definition for message of type '<stopSingleServo>"
  (cl:format cl:nil "uint32 servoId~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'stopSingleServo)))
  "Returns full string definition for message of type 'stopSingleServo"
  (cl:format cl:nil "uint32 servoId~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <stopSingleServo>))
  (cl:+ 0
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <stopSingleServo>))
  "Converts a ROS message object to a list"
  (cl:list 'stopSingleServo
    (cl:cons ':servoId (servoId msg))
))
