;; Auto-generated. Do not edit!


(when (boundp 'robotarminterface::moveServos)
  (if (not (find-package "ROBOTARMINTERFACE"))
    (make-package "ROBOTARMINTERFACE"))
  (shadow 'moveServos (find-package "ROBOTARMINTERFACE")))
(unless (find-package "ROBOTARMINTERFACE::MOVESERVOS")
  (make-package "ROBOTARMINTERFACE::MOVESERVOS"))

(in-package "ROS")
;;//! \htmlinclude moveServos.msg.html


(defclass robotarminterface::moveServos
  :super ros::object
  :slots (_servos _time ))

(defmethod robotarminterface::moveServos
  (:init
   (&key
    ((:servos __servos) (let (r) (dotimes (i 0) (push (instance robotarminterface::servoPosition :init) r)) r))
    ((:time __time) 0)
    )
   (send-super :init)
   (setq _servos __servos)
   (setq _time (round __time))
   self)
  (:servos
   (&rest __servos)
   (if (keywordp (car __servos))
       (send* _servos __servos)
     (progn
       (if __servos (setq _servos (car __servos)))
       _servos)))
  (:time
   (&optional __time)
   (if __time (setq _time __time)) _time)
  (:serialization-length
   ()
   (+
    ;; robotarminterface/servoPosition[] _servos
    (apply #'+ (send-all _servos :serialization-length)) 4
    ;; uint32 _time
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; robotarminterface/servoPosition[] _servos
     (write-long (length _servos) s)
     (dolist (elem _servos)
       (send elem :serialize s)
       )
     ;; uint32 _time
       (write-long _time s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; robotarminterface/servoPosition[] _servos
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _servos (let (r) (dotimes (i n) (push (instance robotarminterface::servoPosition :init) r)) r))
     (dolist (elem- _servos)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;; uint32 _time
     (setq _time (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get robotarminterface::moveServos :md5sum-) "4a71efd697b71e458b626f0c557f215a")
(setf (get robotarminterface::moveServos :datatype-) "robotarminterface/moveServos")
(setf (get robotarminterface::moveServos :definition-)
      "servoPosition[] servos
uint32 time

================================================================================
MSG: robotarminterface/servoPosition
uint32 servoId
int32 position
")



(provide :robotarminterface/moveServos "4a71efd697b71e458b626f0c557f215a")


