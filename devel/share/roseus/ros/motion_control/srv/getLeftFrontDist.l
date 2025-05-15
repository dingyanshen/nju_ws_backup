;; Auto-generated. Do not edit!


(when (boundp 'motion_control::getLeftFrontDist)
  (if (not (find-package "MOTION_CONTROL"))
    (make-package "MOTION_CONTROL"))
  (shadow 'getLeftFrontDist (find-package "MOTION_CONTROL")))
(unless (find-package "MOTION_CONTROL::GETLEFTFRONTDIST")
  (make-package "MOTION_CONTROL::GETLEFTFRONTDIST"))
(unless (find-package "MOTION_CONTROL::GETLEFTFRONTDISTREQUEST")
  (make-package "MOTION_CONTROL::GETLEFTFRONTDISTREQUEST"))
(unless (find-package "MOTION_CONTROL::GETLEFTFRONTDISTRESPONSE")
  (make-package "MOTION_CONTROL::GETLEFTFRONTDISTRESPONSE"))

(in-package "ROS")





(defclass motion_control::getLeftFrontDistRequest
  :super ros::object
  :slots ())

(defmethod motion_control::getLeftFrontDistRequest
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

(defclass motion_control::getLeftFrontDistResponse
  :super ros::object
  :slots (_leftFrontDist ))

(defmethod motion_control::getLeftFrontDistResponse
  (:init
   (&key
    ((:leftFrontDist __leftFrontDist) 0.0)
    )
   (send-super :init)
   (setq _leftFrontDist (float __leftFrontDist))
   self)
  (:leftFrontDist
   (&optional __leftFrontDist)
   (if __leftFrontDist (setq _leftFrontDist __leftFrontDist)) _leftFrontDist)
  (:serialization-length
   ()
   (+
    ;; float32 _leftFrontDist
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float32 _leftFrontDist
       (sys::poke _leftFrontDist (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float32 _leftFrontDist
     (setq _leftFrontDist (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(defclass motion_control::getLeftFrontDist
  :super ros::object
  :slots ())

(setf (get motion_control::getLeftFrontDist :md5sum-) "4c23ace1d1a78b192ca63b2f98acf0d3")
(setf (get motion_control::getLeftFrontDist :datatype-) "motion_control/getLeftFrontDist")
(setf (get motion_control::getLeftFrontDist :request) motion_control::getLeftFrontDistRequest)
(setf (get motion_control::getLeftFrontDist :response) motion_control::getLeftFrontDistResponse)

(defmethod motion_control::getLeftFrontDistRequest
  (:response () (instance motion_control::getLeftFrontDistResponse :init)))

(setf (get motion_control::getLeftFrontDistRequest :md5sum-) "4c23ace1d1a78b192ca63b2f98acf0d3")
(setf (get motion_control::getLeftFrontDistRequest :datatype-) "motion_control/getLeftFrontDistRequest")
(setf (get motion_control::getLeftFrontDistRequest :definition-)
      "---
float32 leftFrontDist
")

(setf (get motion_control::getLeftFrontDistResponse :md5sum-) "4c23ace1d1a78b192ca63b2f98acf0d3")
(setf (get motion_control::getLeftFrontDistResponse :datatype-) "motion_control/getLeftFrontDistResponse")
(setf (get motion_control::getLeftFrontDistResponse :definition-)
      "---
float32 leftFrontDist
")



(provide :motion_control/getLeftFrontDist "4c23ace1d1a78b192ca63b2f98acf0d3")


