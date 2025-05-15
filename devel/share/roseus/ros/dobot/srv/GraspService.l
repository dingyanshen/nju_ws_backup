;; Auto-generated. Do not edit!


(when (boundp 'dobot::GraspService)
  (if (not (find-package "DOBOT"))
    (make-package "DOBOT"))
  (shadow 'GraspService (find-package "DOBOT")))
(unless (find-package "DOBOT::GRASPSERVICE")
  (make-package "DOBOT::GRASPSERVICE"))
(unless (find-package "DOBOT::GRASPSERVICEREQUEST")
  (make-package "DOBOT::GRASPSERVICEREQUEST"))
(unless (find-package "DOBOT::GRASPSERVICERESPONSE")
  (make-package "DOBOT::GRASPSERVICERESPONSE"))

(in-package "ROS")





(defclass dobot::GraspServiceRequest
  :super ros::object
  :slots (_shelf_z _pos_z _error_x _error_y ))

(defmethod dobot::GraspServiceRequest
  (:init
   (&key
    ((:shelf_z __shelf_z) nil)
    ((:pos_z __pos_z) nil)
    ((:error_x __error_x) 0.0)
    ((:error_y __error_y) 0.0)
    )
   (send-super :init)
   (setq _shelf_z __shelf_z)
   (setq _pos_z __pos_z)
   (setq _error_x (float __error_x))
   (setq _error_y (float __error_y))
   self)
  (:shelf_z
   (&optional __shelf_z)
   (if __shelf_z (setq _shelf_z __shelf_z)) _shelf_z)
  (:pos_z
   (&optional __pos_z)
   (if __pos_z (setq _pos_z __pos_z)) _pos_z)
  (:error_x
   (&optional __error_x)
   (if __error_x (setq _error_x __error_x)) _error_x)
  (:error_y
   (&optional __error_y)
   (if __error_y (setq _error_y __error_y)) _error_y)
  (:serialization-length
   ()
   (+
    ;; bool _shelf_z
    1
    ;; bool _pos_z
    1
    ;; float32 _error_x
    4
    ;; float32 _error_y
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _shelf_z
       (if _shelf_z (write-byte -1 s) (write-byte 0 s))
     ;; bool _pos_z
       (if _pos_z (write-byte -1 s) (write-byte 0 s))
     ;; float32 _error_x
       (sys::poke _error_x (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _error_y
       (sys::poke _error_y (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _shelf_z
     (setq _shelf_z (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _pos_z
     (setq _pos_z (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; float32 _error_x
     (setq _error_x (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _error_y
     (setq _error_y (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(defclass dobot::GraspServiceResponse
  :super ros::object
  :slots (_success ))

(defmethod dobot::GraspServiceResponse
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

(defclass dobot::GraspService
  :super ros::object
  :slots ())

(setf (get dobot::GraspService :md5sum-) "1e7660736e31416d5bfaa2f3f2cf2e32")
(setf (get dobot::GraspService :datatype-) "dobot/GraspService")
(setf (get dobot::GraspService :request) dobot::GraspServiceRequest)
(setf (get dobot::GraspService :response) dobot::GraspServiceResponse)

(defmethod dobot::GraspServiceRequest
  (:response () (instance dobot::GraspServiceResponse :init)))

(setf (get dobot::GraspServiceRequest :md5sum-) "1e7660736e31416d5bfaa2f3f2cf2e32")
(setf (get dobot::GraspServiceRequest :datatype-) "dobot/GraspServiceRequest")
(setf (get dobot::GraspServiceRequest :definition-)
      "bool shelf_z
bool pos_z
float32 error_x
float32 error_y
---
bool success
")

(setf (get dobot::GraspServiceResponse :md5sum-) "1e7660736e31416d5bfaa2f3f2cf2e32")
(setf (get dobot::GraspServiceResponse :datatype-) "dobot/GraspServiceResponse")
(setf (get dobot::GraspServiceResponse :definition-)
      "bool shelf_z
bool pos_z
float32 error_x
float32 error_y
---
bool success
")



(provide :dobot/GraspService "1e7660736e31416d5bfaa2f3f2cf2e32")


