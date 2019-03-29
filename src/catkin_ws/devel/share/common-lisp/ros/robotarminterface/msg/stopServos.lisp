; Auto-generated. Do not edit!


(cl:in-package robotarminterface-msg)


;//! \htmlinclude stopServos.msg.html

(cl:defclass <stopServos> (roslisp-msg-protocol:ros-message)
  ((servoIds
    :reader servoIds
    :initarg :servoIds
    :type (cl:vector cl:integer)
   :initform (cl:make-array 0 :element-type 'cl:integer :initial-element 0)))
)

(cl:defclass stopServos (<stopServos>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <stopServos>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'stopServos)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name robotarminterface-msg:<stopServos> is deprecated: use robotarminterface-msg:stopServos instead.")))

(cl:ensure-generic-function 'servoIds-val :lambda-list '(m))
(cl:defmethod servoIds-val ((m <stopServos>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader robotarminterface-msg:servoIds-val is deprecated.  Use robotarminterface-msg:servoIds instead.")
  (servoIds m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <stopServos>) ostream)
  "Serializes a message object of type '<stopServos>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'servoIds))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:write-byte (cl:ldb (cl:byte 8 0) ele) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) ele) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) ele) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) ele) ostream))
   (cl:slot-value msg 'servoIds))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <stopServos>) istream)
  "Deserializes a message object of type '<stopServos>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'servoIds) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'servoIds)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:aref vals i)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:aref vals i)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:aref vals i)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:aref vals i)) (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<stopServos>)))
  "Returns string type for a message object of type '<stopServos>"
  "robotarminterface/stopServos")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'stopServos)))
  "Returns string type for a message object of type 'stopServos"
  "robotarminterface/stopServos")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<stopServos>)))
  "Returns md5sum for a message object of type '<stopServos>"
  "58c2b1e06ba6865d4582bb6764fd753f")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'stopServos)))
  "Returns md5sum for a message object of type 'stopServos"
  "58c2b1e06ba6865d4582bb6764fd753f")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<stopServos>)))
  "Returns full string definition for message of type '<stopServos>"
  (cl:format cl:nil "uint32[] servoIds~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'stopServos)))
  "Returns full string definition for message of type 'stopServos"
  (cl:format cl:nil "uint32[] servoIds~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <stopServos>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'servoIds) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <stopServos>))
  "Converts a ROS message object to a list"
  (cl:list 'stopServos
    (cl:cons ':servoIds (servoIds msg))
))
