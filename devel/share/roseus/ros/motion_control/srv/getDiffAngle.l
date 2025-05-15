;; Auto-generated. Do not edit!


(when (boundp 'motion_control::getDiffAngle)
  (if (not (find-package "MOTION_CONTROL"))
    (make-package "MOTION_CONTROL"))
  (shadow 'getDiffAngle (find-package "MOTION_CONTROL")))
(unless (find-package "MOTION_CONTROL::GETDIFFANGLE")
  (make-package "MOTION_CONTROL::GETDIFFANGLE"))
(unless (find-package "MOTION_CONTROL::GETDIFFANGLEREQUEST")
  (make-package "MOTION_CONTROL::GETDIFFANGLEREQUEST"))
(unless (find-package "MOTION_CONTROL::GETDIFFANGLERESPONSE")
  (make-package "MOTION_CONTROL::GETDIFFANGLERESPONSE"))

(in-package "ROS")





(defclass motion_control::getDiffAngleRequest
  :super ros::object
  :slots ())

(defmethod motion_control::getDiffAngleRequest
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

(defclass motion_control::getDiffAngleResponse
  :super ros::object
  :slots (_diffAngle ))

(defmethod motion_control::getDiffAngleResponse
  (:init
   (&key
    ((:diffAngle __diffAngle) 0.0)
    )
   (send-super :init)
   (setq _diffAngle (float __diffAngle))
   self)
  (:diffAngle
   (&optional __diffAngle)
   (if __diffAngle (setq _diffAngle __diffAngle)) _diffAngle)
  (:serialization-length
   ()
   (+
    ;; float32 _diffAngle
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float32 _diffAngle
       (sys::poke _diffAngle (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float32 _diffAngle
     (setq _diffAngle (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(defclass motion_control::getDiffAngle
  :super ros::object
  :slots ())

(setf (get motion_control::getDiffAngle :md5sum-) "39a7032f716c41fca7c3cc6c27125bc3")
(setf (get motion_control::getDiffAngle :datatype-) "motion_control/getDiffAngle")
(setf (get motion_control::getDiffAngle :request) motion_control::getDiffAngleRequest)
(setf (get motion_control::getDiffAngle :response) motion_control::getDiffAngleResponse)

(defmethod motion_control::getDiffAngleRequest
  (:response () (instance motion_control::getDiffAngleResponse :init)))

(setf (get motion_control::getDiffAngleRequest :md5sum-) "39a7032f716c41fca7c3cc6c27125bc3")
(setf (get motion_control::getDiffAngleRequest :datatype-) "motion_control/getDiffAngleRequest")
(setf (get motion_control::getDiffAngleRequest :definition-)
      "---
float32 diffAngle
")

(setf (get motion_control::getDiffAngleResponse :md5sum-) "39a7032f716c41fca7c3cc6c27125bc3")
(setf (get motion_control::getDiffAngleResponse :datatype-) "motion_control/getDiffAngleResponse")
(setf (get motion_control::getDiffAngleResponse :definition-)
      "---
float32 diffAngle
")



(provide :motion_control/getDiffAngle "39a7032f716c41fca7c3cc6c27125bc3")


