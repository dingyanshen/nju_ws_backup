;; Auto-generated. Do not edit!


(when (boundp 'motion_control::getRightFrontDist)
  (if (not (find-package "MOTION_CONTROL"))
    (make-package "MOTION_CONTROL"))
  (shadow 'getRightFrontDist (find-package "MOTION_CONTROL")))
(unless (find-package "MOTION_CONTROL::GETRIGHTFRONTDIST")
  (make-package "MOTION_CONTROL::GETRIGHTFRONTDIST"))
(unless (find-package "MOTION_CONTROL::GETRIGHTFRONTDISTREQUEST")
  (make-package "MOTION_CONTROL::GETRIGHTFRONTDISTREQUEST"))
(unless (find-package "MOTION_CONTROL::GETRIGHTFRONTDISTRESPONSE")
  (make-package "MOTION_CONTROL::GETRIGHTFRONTDISTRESPONSE"))

(in-package "ROS")





(defclass motion_control::getRightFrontDistRequest
  :super ros::object
  :slots ())

(defmethod motion_control::getRightFrontDistRequest
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

(defclass motion_control::getRightFrontDistResponse
  :super ros::object
  :slots (_rightFrontDist ))

(defmethod motion_control::getRightFrontDistResponse
  (:init
   (&key
    ((:rightFrontDist __rightFrontDist) 0.0)
    )
   (send-super :init)
   (setq _rightFrontDist (float __rightFrontDist))
   self)
  (:rightFrontDist
   (&optional __rightFrontDist)
   (if __rightFrontDist (setq _rightFrontDist __rightFrontDist)) _rightFrontDist)
  (:serialization-length
   ()
   (+
    ;; float32 _rightFrontDist
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float32 _rightFrontDist
       (sys::poke _rightFrontDist (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float32 _rightFrontDist
     (setq _rightFrontDist (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(defclass motion_control::getRightFrontDist
  :super ros::object
  :slots ())

(setf (get motion_control::getRightFrontDist :md5sum-) "b33671b8766d8b0db5bcef50e936b90c")
(setf (get motion_control::getRightFrontDist :datatype-) "motion_control/getRightFrontDist")
(setf (get motion_control::getRightFrontDist :request) motion_control::getRightFrontDistRequest)
(setf (get motion_control::getRightFrontDist :response) motion_control::getRightFrontDistResponse)

(defmethod motion_control::getRightFrontDistRequest
  (:response () (instance motion_control::getRightFrontDistResponse :init)))

(setf (get motion_control::getRightFrontDistRequest :md5sum-) "b33671b8766d8b0db5bcef50e936b90c")
(setf (get motion_control::getRightFrontDistRequest :datatype-) "motion_control/getRightFrontDistRequest")
(setf (get motion_control::getRightFrontDistRequest :definition-)
      "---
float32 rightFrontDist
")

(setf (get motion_control::getRightFrontDistResponse :md5sum-) "b33671b8766d8b0db5bcef50e936b90c")
(setf (get motion_control::getRightFrontDistResponse :datatype-) "motion_control/getRightFrontDistResponse")
(setf (get motion_control::getRightFrontDistResponse :definition-)
      "---
float32 rightFrontDist
")



(provide :motion_control/getRightFrontDist "b33671b8766d8b0db5bcef50e936b90c")


