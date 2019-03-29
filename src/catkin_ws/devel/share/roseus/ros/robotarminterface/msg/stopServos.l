;; Auto-generated. Do not edit!


(when (boundp 'robotarminterface::stopServos)
  (if (not (find-package "ROBOTARMINTERFACE"))
    (make-package "ROBOTARMINTERFACE"))
  (shadow 'stopServos (find-package "ROBOTARMINTERFACE")))
(unless (find-package "ROBOTARMINTERFACE::STOPSERVOS")
  (make-package "ROBOTARMINTERFACE::STOPSERVOS"))

(in-package "ROS")
;;//! \htmlinclude stopServos.msg.html


(defclass robotarminterface::stopServos
  :super ros::object
  :slots (_servoIds ))

(defmethod robotarminterface::stopServos
  (:init
   (&key
    ((:servoIds __servoIds) (make-array 0 :initial-element 0 :element-type :integer))
    )
   (send-super :init)
   (setq _servoIds __servoIds)
   self)
  (:servoIds
   (&optional __servoIds)
   (if __servoIds (setq _servoIds __servoIds)) _servoIds)
  (:serialization-length
   ()
   (+
    ;; uint32[] _servoIds
    (* 4    (length _servoIds)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint32[] _servoIds
     (write-long (length _servoIds) s)
     (dotimes (i (length _servoIds))
       (write-long (elt _servoIds i) s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint32[] _servoIds
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _servoIds (instantiate integer-vector n))
     (dotimes (i n)
     (setf (elt _servoIds i) (sys::peek buf ptr- :integer)) (incf ptr- 4)
     ))
   ;;
   self)
  )

(setf (get robotarminterface::stopServos :md5sum-) "58c2b1e06ba6865d4582bb6764fd753f")
(setf (get robotarminterface::stopServos :datatype-) "robotarminterface/stopServos")
(setf (get robotarminterface::stopServos :definition-)
      "uint32[] servoIds

")



(provide :robotarminterface/stopServos "58c2b1e06ba6865d4582bb6764fd753f")


