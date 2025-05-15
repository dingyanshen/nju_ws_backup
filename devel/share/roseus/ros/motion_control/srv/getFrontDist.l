;; Auto-generated. Do not edit!


(when (boundp 'motion_control::getFrontDist)
  (if (not (find-package "MOTION_CONTROL"))
    (make-package "MOTION_CONTROL"))
  (shadow 'getFrontDist (find-package "MOTION_CONTROL")))
(unless (find-package "MOTION_CONTROL::GETFRONTDIST")
  (make-package "MOTION_CONTROL::GETFRONTDIST"))
(unless (find-package "MOTION_CONTROL::GETFRONTDISTREQUEST")
  (make-package "MOTION_CONTROL::GETFRONTDISTREQUEST"))
(unless (find-package "MOTION_CONTROL::GETFRONTDISTRESPONSE")
  (make-package "MOTION_CONTROL::GETFRONTDISTRESPONSE"))

(in-package "ROS")





(defclass motion_control::getFrontDistRequest
  :super ros::object
  :slots ())

(defmethod motion_control::getFrontDistRequest
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

(defclass motion_control::getFrontDistResponse
  :super ros::object
  :slots (_frontDist ))

(defmethod motion_control::getFrontDistResponse
  (:init
   (&key
    ((:frontDist __frontDist) 0.0)
    )
   (send-super :init)
   (setq _frontDist (float __frontDist))
   self)
  (:frontDist
   (&optional __frontDist)
   (if __frontDist (setq _frontDist __frontDist)) _frontDist)
  (:serialization-length
   ()
   (+
    ;; float32 _frontDist
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float32 _frontDist
       (sys::poke _frontDist (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float32 _frontDist
     (setq _frontDist (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(defclass motion_control::getFrontDist
  :super ros::object
  :slots ())

(setf (get motion_control::getFrontDist :md5sum-) "1e75451363e2baac9b8bfb0e12a9fdd0")
(setf (get motion_control::getFrontDist :datatype-) "motion_control/getFrontDist")
(setf (get motion_control::getFrontDist :request) motion_control::getFrontDistRequest)
(setf (get motion_control::getFrontDist :response) motion_control::getFrontDistResponse)

(defmethod motion_control::getFrontDistRequest
  (:response () (instance motion_control::getFrontDistResponse :init)))

(setf (get motion_control::getFrontDistRequest :md5sum-) "1e75451363e2baac9b8bfb0e12a9fdd0")
(setf (get motion_control::getFrontDistRequest :datatype-) "motion_control/getFrontDistRequest")
(setf (get motion_control::getFrontDistRequest :definition-)
      "---
float32 frontDist
")

(setf (get motion_control::getFrontDistResponse :md5sum-) "1e75451363e2baac9b8bfb0e12a9fdd0")
(setf (get motion_control::getFrontDistResponse :datatype-) "motion_control/getFrontDistResponse")
(setf (get motion_control::getFrontDistResponse :definition-)
      "---
float32 frontDist
")



(provide :motion_control/getFrontDist "1e75451363e2baac9b8bfb0e12a9fdd0")


