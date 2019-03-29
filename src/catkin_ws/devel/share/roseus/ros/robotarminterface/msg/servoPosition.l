;; Auto-generated. Do not edit!


(when (boundp 'robotarminterface::servoPosition)
  (if (not (find-package "ROBOTARMINTERFACE"))
    (make-package "ROBOTARMINTERFACE"))
  (shadow 'servoPosition (find-package "ROBOTARMINTERFACE")))
(unless (find-package "ROBOTARMINTERFACE::SERVOPOSITION")
  (make-package "ROBOTARMINTERFACE::SERVOPOSITION"))

(in-package "ROS")
;;//! \htmlinclude servoPosition.msg.html


(defclass robotarminterface::servoPosition
  :super ros::object
  :slots (_servoId _position ))

(defmethod robotarminterface::servoPosition
  (:init
   (&key
    ((:servoId __servoId) 0)
    ((:position __position) 0)
    )
   (send-super :init)
   (setq _servoId (round __servoId))
   (setq _position (round __position))
   self)
  (:servoId
   (&optional __servoId)
   (if __servoId (setq _servoId __servoId)) _servoId)
  (:position
   (&optional __position)
   (if __position (setq _position __position)) _position)
  (:serialization-length
   ()
   (+
    ;; uint32 _servoId
    4
    ;; int32 _position
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint32 _servoId
       (write-long _servoId s)
     ;; int32 _position
       (write-long _position s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint32 _servoId
     (setq _servoId (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _position
     (setq _position (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get robotarminterface::servoPosition :md5sum-) "21c24bd1e99f0c44d572dd36095ff06f")
(setf (get robotarminterface::servoPosition :datatype-) "robotarminterface/servoPosition")
(setf (get robotarminterface::servoPosition :definition-)
      "uint32 servoId
int32 position
")



(provide :robotarminterface/servoPosition "21c24bd1e99f0c44d572dd36095ff06f")


