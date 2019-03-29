;; Auto-generated. Do not edit!


(when (boundp 'robotarminterface::armInstruction)
  (if (not (find-package "ROBOTARMINTERFACE"))
    (make-package "ROBOTARMINTERFACE"))
  (shadow 'armInstruction (find-package "ROBOTARMINTERFACE")))
(unless (find-package "ROBOTARMINTERFACE::ARMINSTRUCTION")
  (make-package "ROBOTARMINTERFACE::ARMINSTRUCTION"))

(in-package "ROS")
;;//! \htmlinclude armInstruction.msg.html


(defclass robotarminterface::armInstruction
  :super ros::object
  :slots (_instruction _time ))

(defmethod robotarminterface::armInstruction
  (:init
   (&key
    ((:instruction __instruction) "")
    ((:time __time) 0)
    )
   (send-super :init)
   (setq _instruction (string __instruction))
   (setq _time (round __time))
   self)
  (:instruction
   (&optional __instruction)
   (if __instruction (setq _instruction __instruction)) _instruction)
  (:time
   (&optional __time)
   (if __time (setq _time __time)) _time)
  (:serialization-length
   ()
   (+
    ;; string _instruction
    4 (length _instruction)
    ;; uint32 _time
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _instruction
       (write-long (length _instruction) s) (princ _instruction s)
     ;; uint32 _time
       (write-long _time s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _instruction
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _instruction (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; uint32 _time
     (setq _time (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get robotarminterface::armInstruction :md5sum-) "dde04e15fd9ee87f96fc4cbc41f78428")
(setf (get robotarminterface::armInstruction :datatype-) "robotarminterface/armInstruction")
(setf (get robotarminterface::armInstruction :definition-)
      "string instruction
uint32 time
")



(provide :robotarminterface/armInstruction "dde04e15fd9ee87f96fc4cbc41f78428")


