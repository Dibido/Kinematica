; Auto-generated. Do not edit!


(cl:in-package robotarminterface-msg)


;//! \htmlinclude armInstruction.msg.html

(cl:defclass <armInstruction> (roslisp-msg-protocol:ros-message)
  ((instruction
    :reader instruction
    :initarg :instruction
    :type cl:string
    :initform "")
   (time
    :reader time
    :initarg :time
    :type cl:integer
    :initform 0))
)

(cl:defclass armInstruction (<armInstruction>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <armInstruction>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'armInstruction)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name robotarminterface-msg:<armInstruction> is deprecated: use robotarminterface-msg:armInstruction instead.")))

(cl:ensure-generic-function 'instruction-val :lambda-list '(m))
(cl:defmethod instruction-val ((m <armInstruction>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader robotarminterface-msg:instruction-val is deprecated.  Use robotarminterface-msg:instruction instead.")
  (instruction m))

(cl:ensure-generic-function 'time-val :lambda-list '(m))
(cl:defmethod time-val ((m <armInstruction>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader robotarminterface-msg:time-val is deprecated.  Use robotarminterface-msg:time instead.")
  (time m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <armInstruction>) ostream)
  "Serializes a message object of type '<armInstruction>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'instruction))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'instruction))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'time)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'time)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'time)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'time)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <armInstruction>) istream)
  "Deserializes a message object of type '<armInstruction>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'instruction) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'instruction) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'time)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'time)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'time)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'time)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<armInstruction>)))
  "Returns string type for a message object of type '<armInstruction>"
  "robotarminterface/armInstruction")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'armInstruction)))
  "Returns string type for a message object of type 'armInstruction"
  "robotarminterface/armInstruction")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<armInstruction>)))
  "Returns md5sum for a message object of type '<armInstruction>"
  "dde04e15fd9ee87f96fc4cbc41f78428")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'armInstruction)))
  "Returns md5sum for a message object of type 'armInstruction"
  "dde04e15fd9ee87f96fc4cbc41f78428")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<armInstruction>)))
  "Returns full string definition for message of type '<armInstruction>"
  (cl:format cl:nil "string instruction~%uint32 time~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'armInstruction)))
  "Returns full string definition for message of type 'armInstruction"
  (cl:format cl:nil "string instruction~%uint32 time~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <armInstruction>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'instruction))
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <armInstruction>))
  "Converts a ROS message object to a list"
  (cl:list 'armInstruction
    (cl:cons ':instruction (instruction msg))
    (cl:cons ':time (time msg))
))
