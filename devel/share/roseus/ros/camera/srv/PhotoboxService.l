;; Auto-generated. Do not edit!


(when (boundp 'camera::PhotoboxService)
  (if (not (find-package "CAMERA"))
    (make-package "CAMERA"))
  (shadow 'PhotoboxService (find-package "CAMERA")))
(unless (find-package "CAMERA::PHOTOBOXSERVICE")
  (make-package "CAMERA::PHOTOBOXSERVICE"))
(unless (find-package "CAMERA::PHOTOBOXSERVICEREQUEST")
  (make-package "CAMERA::PHOTOBOXSERVICEREQUEST"))
(unless (find-package "CAMERA::PHOTOBOXSERVICERESPONSE")
  (make-package "CAMERA::PHOTOBOXSERVICERESPONSE"))

(in-package "ROS")





(defclass camera::PhotoboxServiceRequest
  :super ros::object
  :slots ())

(defmethod camera::PhotoboxServiceRequest
  (:init
   (&key
    )
   (send-super :init)
   self)
  (:serialization-length
   ()
   (+
    0
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;;
   self)
  )

(defclass camera::PhotoboxServiceResponse
  :super ros::object
  :slots (_result ))

(defmethod camera::PhotoboxServiceResponse
  (:init
   (&key
    ((:result __result) 0)
    )
   (send-super :init)
   (setq _result (round __result))
   self)
  (:result
   (&optional __result)
   (if __result (setq _result __result)) _result)
  (:serialization-length
   ()
   (+
    ;; int32 _result
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int32 _result
       (write-long _result s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int32 _result
     (setq _result (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(defclass camera::PhotoboxService
  :super ros::object
  :slots ())

(setf (get camera::PhotoboxService :md5sum-) "034a8e20d6a306665e3a5b340fab3f09")
(setf (get camera::PhotoboxService :datatype-) "camera/PhotoboxService")
(setf (get camera::PhotoboxService :request) camera::PhotoboxServiceRequest)
(setf (get camera::PhotoboxService :response) camera::PhotoboxServiceResponse)

(defmethod camera::PhotoboxServiceRequest
  (:response () (instance camera::PhotoboxServiceResponse :init)))

(setf (get camera::PhotoboxServiceRequest :md5sum-) "034a8e20d6a306665e3a5b340fab3f09")
(setf (get camera::PhotoboxServiceRequest :datatype-) "camera/PhotoboxServiceRequest")
(setf (get camera::PhotoboxServiceRequest :definition-)
      "---
int32 result
")

(setf (get camera::PhotoboxServiceResponse :md5sum-) "034a8e20d6a306665e3a5b340fab3f09")
(setf (get camera::PhotoboxServiceResponse :datatype-) "camera/PhotoboxServiceResponse")
(setf (get camera::PhotoboxServiceResponse :definition-)
      "---
int32 result
")



(provide :camera/PhotoboxService "034a8e20d6a306665e3a5b340fab3f09")


