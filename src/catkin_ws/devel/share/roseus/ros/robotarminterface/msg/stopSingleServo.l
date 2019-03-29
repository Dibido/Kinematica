;; Auto-generated. Do not edit!


(when (boundp 'robotarminterface::stopSingleServo)
  (if (not (find-package "ROBOTARMINTERFACE"))
    (make-package "ROBOTARMINTERFACE"))
  (shadow 'stopSingleServo (find-package "ROBOTARMINTERFACE")))
(unless (find-package "ROBOTARMINTERFACE::STOPSINGLESERVO")
  (make-package "ROBOTARMINTERFACE::STOPSINGLESERVO"))

(in-package "ROS")
;;//! \htmlinclude stopSingleServo.msg.html


(defclass robotarminterface::stopSingleServo
  :super ros::object
  :slots (_servoId ))

(defmethod robotarminterface::stopSingleServo
  (:init
   (&key
    ((:servoId __servoId) 0)
    )
   (send-super :init)
   (setq _servoId (round __servoId))
   self)
  (:servoId
   (&optional __servoId)
   (if __servoId (setq _servoId __servoId)) _servoId)
  (:serialization-length
   ()
   (+
    ;; uint32 _servoId
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint32 _servoId
       (write-long _servoId s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint32 _servoId
     (setq _servoId (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get robotarminterface::stopSingleServo :md5sum-) "c229a51eeb50adb8a4245c15dbd40200")
(setf (get robotarminterface::stopSingleServo :datatype-) "robotarminterface/stopSingleServo")
(setf (get robotarminterface::stopSingleServo :definition-)
      "uint32 servoId
")



(provide :robotarminterface/stopSingleServo "c229a51eeb50adb8a4245c15dbd40200")


