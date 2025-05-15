;; Auto-generated. Do not edit!


(when (boundp 'motion_control::getLeftDist)
  (if (not (find-package "MOTION_CONTROL"))
    (make-package "MOTION_CONTROL"))
  (shadow 'getLeftDist (find-package "MOTION_CONTROL")))
(unless (find-package "MOTION_CONTROL::GETLEFTDIST")
  (make-package "MOTION_CONTROL::GETLEFTDIST"))
(unless (find-package "MOTION_CONTROL::GETLEFTDISTREQUEST")
  (make-package "MOTION_CONTROL::GETLEFTDISTREQUEST"))
(unless (find-package "MOTION_CONTROL::GETLEFTDISTRESPONSE")
  (make-package "MOTION_CONTROL::GETLEFTDISTRESPONSE"))

(in-package "ROS")





(defclass motion_control::getLeftDistRequest
  :super ros::object
  :slots ())

(defmethod motion_control::getLeftDistRequest
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

(defclass motion_control::getLeftDistResponse
  :super ros::object
  :slots (_leftDist ))

(defmethod motion_control::getLeftDistResponse
  (:init
   (&key
    ((:leftDist __leftDist) 0.0)
    )
   (send-super :init)
   (setq _leftDist (float __leftDist))
   self)
  (:leftDist
   (&optional __leftDist)
   (if __leftDist (setq _leftDist __leftDist)) _leftDist)
  (:serialization-length
   ()
   (+
    ;; float32 _leftDist
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float32 _leftDist
       (sys::poke _leftDist (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float32 _leftDist
     (setq _leftDist (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(defclass motion_control::getLeftDist
  :super ros::object
  :slots ())

(setf (get motion_control::getLeftDist :md5sum-) "4e0d27729a997215618b53389df19443")
(setf (get motion_control::getLeftDist :datatype-) "motion_control/getLeftDist")
(setf (get motion_control::getLeftDist :request) motion_control::getLeftDistRequest)
(setf (get motion_control::getLeftDist :response) motion_control::getLeftDistResponse)

(defmethod motion_control::getLeftDistRequest
  (:response () (instance motion_control::getLeftDistResponse :init)))

(setf (get motion_control::getLeftDistRequest :md5sum-) "4e0d27729a997215618b53389df19443")
(setf (get motion_control::getLeftDistRequest :datatype-) "motion_control/getLeftDistRequest")
(setf (get motion_control::getLeftDistRequest :definition-)
      "---
float32 leftDist
")

(setf (get motion_control::getLeftDistResponse :md5sum-) "4e0d27729a997215618b53389df19443")
(setf (get motion_control::getLeftDistResponse :datatype-) "motion_control/getLeftDistResponse")
(setf (get motion_control::getLeftDistResponse :definition-)
      "---
float32 leftDist
")



(provide :motion_control/getLeftDist "4e0d27729a997215618b53389df19443")


