;; Auto-generated. Do not edit!


(when (boundp 'motion_control::getRightDist)
  (if (not (find-package "MOTION_CONTROL"))
    (make-package "MOTION_CONTROL"))
  (shadow 'getRightDist (find-package "MOTION_CONTROL")))
(unless (find-package "MOTION_CONTROL::GETRIGHTDIST")
  (make-package "MOTION_CONTROL::GETRIGHTDIST"))
(unless (find-package "MOTION_CONTROL::GETRIGHTDISTREQUEST")
  (make-package "MOTION_CONTROL::GETRIGHTDISTREQUEST"))
(unless (find-package "MOTION_CONTROL::GETRIGHTDISTRESPONSE")
  (make-package "MOTION_CONTROL::GETRIGHTDISTRESPONSE"))

(in-package "ROS")





(defclass motion_control::getRightDistRequest
  :super ros::object
  :slots ())

(defmethod motion_control::getRightDistRequest
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

(defclass motion_control::getRightDistResponse
  :super ros::object
  :slots (_rightDist ))

(defmethod motion_control::getRightDistResponse
  (:init
   (&key
    ((:rightDist __rightDist) 0.0)
    )
   (send-super :init)
   (setq _rightDist (float __rightDist))
   self)
  (:rightDist
   (&optional __rightDist)
   (if __rightDist (setq _rightDist __rightDist)) _rightDist)
  (:serialization-length
   ()
   (+
    ;; float32 _rightDist
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float32 _rightDist
       (sys::poke _rightDist (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float32 _rightDist
     (setq _rightDist (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(defclass motion_control::getRightDist
  :super ros::object
  :slots ())

(setf (get motion_control::getRightDist :md5sum-) "46d3dd3b27754b2db4e99d35f4cb12c3")
(setf (get motion_control::getRightDist :datatype-) "motion_control/getRightDist")
(setf (get motion_control::getRightDist :request) motion_control::getRightDistRequest)
(setf (get motion_control::getRightDist :response) motion_control::getRightDistResponse)

(defmethod motion_control::getRightDistRequest
  (:response () (instance motion_control::getRightDistResponse :init)))

(setf (get motion_control::getRightDistRequest :md5sum-) "46d3dd3b27754b2db4e99d35f4cb12c3")
(setf (get motion_control::getRightDistRequest :datatype-) "motion_control/getRightDistRequest")
(setf (get motion_control::getRightDistRequest :definition-)
      "---
float32 rightDist
")

(setf (get motion_control::getRightDistResponse :md5sum-) "46d3dd3b27754b2db4e99d35f4cb12c3")
(setf (get motion_control::getRightDistResponse :datatype-) "motion_control/getRightDistResponse")
(setf (get motion_control::getRightDistResponse :definition-)
      "---
float32 rightDist
")



(provide :motion_control/getRightDist "46d3dd3b27754b2db4e99d35f4cb12c3")


