;; Auto-generated. Do not edit!


(when (boundp 'camera::PhotoService)
  (if (not (find-package "CAMERA"))
    (make-package "CAMERA"))
  (shadow 'PhotoService (find-package "CAMERA")))
(unless (find-package "CAMERA::PHOTOSERVICE")
  (make-package "CAMERA::PHOTOSERVICE"))
(unless (find-package "CAMERA::PHOTOSERVICEREQUEST")
  (make-package "CAMERA::PHOTOSERVICEREQUEST"))
(unless (find-package "CAMERA::PHOTOSERVICERESPONSE")
  (make-package "CAMERA::PHOTOSERVICERESPONSE"))

(in-package "ROS")





(defclass camera::PhotoServiceRequest
  :super ros::object
  :slots (_type ))

(defmethod camera::PhotoServiceRequest
  (:init
   (&key
    ((:type __type) 0)
    )
   (send-super :init)
   (setq _type (round __type))
   self)
  (:type
   (&optional __type)
   (if __type (setq _type __type)) _type)
  (:serialization-length
   ()
   (+
    ;; int32 _type
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int32 _type
       (write-long _type s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int32 _type
     (setq _type (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(defclass camera::PhotoServiceResponse
  :super ros::object
  :slots (_error_x _error_y ))

(defmethod camera::PhotoServiceResponse
  (:init
   (&key
    ((:error_x __error_x) 0.0)
    ((:error_y __error_y) 0.0)
    )
   (send-super :init)
   (setq _error_x (float __error_x))
   (setq _error_y (float __error_y))
   self)
  (:error_x
   (&optional __error_x)
   (if __error_x (setq _error_x __error_x)) _error_x)
  (:error_y
   (&optional __error_y)
   (if __error_y (setq _error_y __error_y)) _error_y)
  (:serialization-length
   ()
   (+
    ;; float32 _error_x
    4
    ;; float32 _error_y
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float32 _error_x
       (sys::poke _error_x (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _error_y
       (sys::poke _error_y (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float32 _error_x
     (setq _error_x (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _error_y
     (setq _error_y (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(defclass camera::PhotoService
  :super ros::object
  :slots ())

(setf (get camera::PhotoService :md5sum-) "117e6535d65959983e49d94959e9d276")
(setf (get camera::PhotoService :datatype-) "camera/PhotoService")
(setf (get camera::PhotoService :request) camera::PhotoServiceRequest)
(setf (get camera::PhotoService :response) camera::PhotoServiceResponse)

(defmethod camera::PhotoServiceRequest
  (:response () (instance camera::PhotoServiceResponse :init)))

(setf (get camera::PhotoServiceRequest :md5sum-) "117e6535d65959983e49d94959e9d276")
(setf (get camera::PhotoServiceRequest :datatype-) "camera/PhotoServiceRequest")
(setf (get camera::PhotoServiceRequest :definition-)
      "int32 type
---
float32 error_x
float32 error_y
")

(setf (get camera::PhotoServiceResponse :md5sum-) "117e6535d65959983e49d94959e9d276")
(setf (get camera::PhotoServiceResponse :datatype-) "camera/PhotoServiceResponse")
(setf (get camera::PhotoServiceResponse :definition-)
      "int32 type
---
float32 error_x
float32 error_y
")



(provide :camera/PhotoService "117e6535d65959983e49d94959e9d276")


