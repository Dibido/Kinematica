;; Auto-generated. Do not edit!


(when (boundp 'robotarminterface::singleServo)
  (if (not (find-package "ROBOTARMINTERFACE"))
    (make-package "ROBOTARMINTERFACE"))
  (shadow 'singleServo (find-package "ROBOTARMINTERFACE")))
(unless (find-package "ROBOTARMINTERFACE::SINGLESERVO")
  (make-package "ROBOTARMINTERFACE::SINGLESERVO"))

(in-package "ROS")
;;//! \htmlinclude singleServo.msg.html


(defclass robotarminterface::singleServo
  :super ros::object
  :slots (_servoId _position _time ))

(defmethod robotarminterface::singleServo
  (:init
   (&key
    ((:servoId __servoId) 0)
    ((:position __position) 0)
    ((:time __time) 0)
    )
   (send-super :init)
   (setq _servoId (round __servoId))
   (setq _position (round __position))
   (setq _time (round __time))
   self)
  (:servoId
   (&optional __servoId)
   (if __servoId (setq _servoId __servoId)) _servoId)
  (:position
   (&optional __position)
   (if __position (setq _position __position)) _position)
  (:time
   (&optional __time)
   (if __time (setq _time __time)) _time)
  (:serialization-length
   ()
   (+
    ;; uint32 _servoId
    4
    ;; uint32 _position
    4
    ;; uint32 _time
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint32 _servoId
       (write-long _servoId s)
     ;; uint32 _position
       (write-long _position s)
     ;; uint32 _time
       (write-long _time s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint32 _servoId
     (setq _servoId (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; uint32 _position
     (setq _position (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; uint32 _time
     (setq _time (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get robotarminterface::singleServo :md5sum-) "39faf3391f2b3261e444ffb062992407")
(setf (get robotarminterface::singleServo :datatype-) "robotarminterface/singleServo")
(setf (get robotarminterface::singleServo :definition-)
      "uint32 servoId
uint32 position
uint32 time
")



(provide :robotarminterface/singleServo "39faf3391f2b3261e444ffb062992407")


