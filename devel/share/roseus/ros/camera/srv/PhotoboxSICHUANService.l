;; Auto-generated. Do not edit!


(when (boundp 'camera::PhotoboxSICHUANService)
  (if (not (find-package "CAMERA"))
    (make-package "CAMERA"))
  (shadow 'PhotoboxSICHUANService (find-package "CAMERA")))
(unless (find-package "CAMERA::PHOTOBOXSICHUANSERVICE")
  (make-package "CAMERA::PHOTOBOXSICHUANSERVICE"))
(unless (find-package "CAMERA::PHOTOBOXSICHUANSERVICEREQUEST")
  (make-package "CAMERA::PHOTOBOXSICHUANSERVICEREQUEST"))
(unless (find-package "CAMERA::PHOTOBOXSICHUANSERVICERESPONSE")
  (make-package "CAMERA::PHOTOBOXSICHUANSERVICERESPONSE"))

(in-package "ROS")





(defclass camera::PhotoboxSICHUANServiceRequest
  :super ros::object
  :slots ())

(defmethod camera::PhotoboxSICHUANServiceRequest
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

(defclass camera::PhotoboxSICHUANServiceResponse
  :super ros::object
  :slots (_result ))

(defmethod camera::PhotoboxSICHUANServiceResponse
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

(defclass camera::PhotoboxSICHUANService
  :super ros::object
  :slots ())

(setf (get camera::PhotoboxSICHUANService :md5sum-) "034a8e20d6a306665e3a5b340fab3f09")
(setf (get camera::PhotoboxSICHUANService :datatype-) "camera/PhotoboxSICHUANService")
(setf (get camera::PhotoboxSICHUANService :request) camera::PhotoboxSICHUANServiceRequest)
(setf (get camera::PhotoboxSICHUANService :response) camera::PhotoboxSICHUANServiceResponse)

(defmethod camera::PhotoboxSICHUANServiceRequest
  (:response () (instance camera::PhotoboxSICHUANServiceResponse :init)))

(setf (get camera::PhotoboxSICHUANServiceRequest :md5sum-) "034a8e20d6a306665e3a5b340fab3f09")
(setf (get camera::PhotoboxSICHUANServiceRequest :datatype-) "camera/PhotoboxSICHUANServiceRequest")
(setf (get camera::PhotoboxSICHUANServiceRequest :definition-)
      "---
int32 result
")

(setf (get camera::PhotoboxSICHUANServiceResponse :md5sum-) "034a8e20d6a306665e3a5b340fab3f09")
(setf (get camera::PhotoboxSICHUANServiceResponse :datatype-) "camera/PhotoboxSICHUANServiceResponse")
(setf (get camera::PhotoboxSICHUANServiceResponse :definition-)
      "---
int32 result
")



(provide :camera/PhotoboxSICHUANService "034a8e20d6a306665e3a5b340fab3f09")


