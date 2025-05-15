;; Auto-generated. Do not edit!


(when (boundp 'dobot::ThrowService)
  (if (not (find-package "DOBOT"))
    (make-package "DOBOT"))
  (shadow 'ThrowService (find-package "DOBOT")))
(unless (find-package "DOBOT::THROWSERVICE")
  (make-package "DOBOT::THROWSERVICE"))
(unless (find-package "DOBOT::THROWSERVICEREQUEST")
  (make-package "DOBOT::THROWSERVICEREQUEST"))
(unless (find-package "DOBOT::THROWSERVICERESPONSE")
  (make-package "DOBOT::THROWSERVICERESPONSE"))

(in-package "ROS")





(defclass dobot::ThrowServiceRequest
  :super ros::object
  :slots (_pos_z _mailbox_pos ))

(defmethod dobot::ThrowServiceRequest
  (:init
   (&key
    ((:pos_z __pos_z) nil)
    ((:mailbox_pos __mailbox_pos) nil)
    )
   (send-super :init)
   (setq _pos_z __pos_z)
   (setq _mailbox_pos __mailbox_pos)
   self)
  (:pos_z
   (&optional __pos_z)
   (if __pos_z (setq _pos_z __pos_z)) _pos_z)
  (:mailbox_pos
   (&optional __mailbox_pos)
   (if __mailbox_pos (setq _mailbox_pos __mailbox_pos)) _mailbox_pos)
  (:serialization-length
   ()
   (+
    ;; bool _pos_z
    1
    ;; bool _mailbox_pos
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _pos_z
       (if _pos_z (write-byte -1 s) (write-byte 0 s))
     ;; bool _mailbox_pos
       (if _mailbox_pos (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _pos_z
     (setq _pos_z (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _mailbox_pos
     (setq _mailbox_pos (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(defclass dobot::ThrowServiceResponse
  :super ros::object
  :slots (_success ))

(defmethod dobot::ThrowServiceResponse
  (:init
   (&key
    ((:success __success) nil)
    )
   (send-super :init)
   (setq _success __success)
   self)
  (:success
   (&optional __success)
   (if __success (setq _success __success)) _success)
  (:serialization-length
   ()
   (+
    ;; bool _success
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _success
       (if _success (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _success
     (setq _success (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(defclass dobot::ThrowService
  :super ros::object
  :slots ())

(setf (get dobot::ThrowService :md5sum-) "b5ca28a4c733f2992478f530e3af3cbb")
(setf (get dobot::ThrowService :datatype-) "dobot/ThrowService")
(setf (get dobot::ThrowService :request) dobot::ThrowServiceRequest)
(setf (get dobot::ThrowService :response) dobot::ThrowServiceResponse)

(defmethod dobot::ThrowServiceRequest
  (:response () (instance dobot::ThrowServiceResponse :init)))

(setf (get dobot::ThrowServiceRequest :md5sum-) "b5ca28a4c733f2992478f530e3af3cbb")
(setf (get dobot::ThrowServiceRequest :datatype-) "dobot/ThrowServiceRequest")
(setf (get dobot::ThrowServiceRequest :definition-)
      "bool pos_z
bool mailbox_pos
---
bool success
")

(setf (get dobot::ThrowServiceResponse :md5sum-) "b5ca28a4c733f2992478f530e3af3cbb")
(setf (get dobot::ThrowServiceResponse :datatype-) "dobot/ThrowServiceResponse")
(setf (get dobot::ThrowServiceResponse :definition-)
      "bool pos_z
bool mailbox_pos
---
bool success
")



(provide :dobot/ThrowService "b5ca28a4c733f2992478f530e3af3cbb")


