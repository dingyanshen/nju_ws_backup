;; Auto-generated. Do not edit!


(when (boundp 'camera::PhotoshelfService)
  (if (not (find-package "CAMERA"))
    (make-package "CAMERA"))
  (shadow 'PhotoshelfService (find-package "CAMERA")))
(unless (find-package "CAMERA::PHOTOSHELFSERVICE")
  (make-package "CAMERA::PHOTOSHELFSERVICE"))
(unless (find-package "CAMERA::PHOTOSHELFSERVICEREQUEST")
  (make-package "CAMERA::PHOTOSHELFSERVICEREQUEST"))
(unless (find-package "CAMERA::PHOTOSHELFSERVICERESPONSE")
  (make-package "CAMERA::PHOTOSHELFSERVICERESPONSE"))

(in-package "ROS")





(defclass camera::PhotoshelfServiceRequest
  :super ros::object
  :slots (_type ))

(defmethod camera::PhotoshelfServiceRequest
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

(defclass camera::PhotoshelfServiceResponse
  :super ros::object
  :slots (_results _positions_z _positions_x ))

(defmethod camera::PhotoshelfServiceResponse
  (:init
   (&key
    ((:results __results) (make-array 0 :initial-element 0 :element-type :integer))
    ((:positions_z __positions_z) (make-array 0 :initial-element 0 :element-type :integer))
    ((:positions_x __positions_x) (make-array 0 :initial-element 0 :element-type :integer))
    )
   (send-super :init)
   (setq _results __results)
   (setq _positions_z __positions_z)
   (setq _positions_x __positions_x)
   self)
  (:results
   (&optional __results)
   (if __results (setq _results __results)) _results)
  (:positions_z
   (&optional __positions_z)
   (if __positions_z (setq _positions_z __positions_z)) _positions_z)
  (:positions_x
   (&optional __positions_x)
   (if __positions_x (setq _positions_x __positions_x)) _positions_x)
  (:serialization-length
   ()
   (+
    ;; int32[] _results
    (* 4    (length _results)) 4
    ;; int32[] _positions_z
    (* 4    (length _positions_z)) 4
    ;; int32[] _positions_x
    (* 4    (length _positions_x)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int32[] _results
     (write-long (length _results) s)
     (dotimes (i (length _results))
       (write-long (elt _results i) s)
       )
     ;; int32[] _positions_z
     (write-long (length _positions_z) s)
     (dotimes (i (length _positions_z))
       (write-long (elt _positions_z i) s)
       )
     ;; int32[] _positions_x
     (write-long (length _positions_x) s)
     (dotimes (i (length _positions_x))
       (write-long (elt _positions_x i) s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int32[] _results
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _results (instantiate integer-vector n))
     (dotimes (i n)
     (setf (elt _results i) (sys::peek buf ptr- :integer)) (incf ptr- 4)
     ))
   ;; int32[] _positions_z
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _positions_z (instantiate integer-vector n))
     (dotimes (i n)
     (setf (elt _positions_z i) (sys::peek buf ptr- :integer)) (incf ptr- 4)
     ))
   ;; int32[] _positions_x
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _positions_x (instantiate integer-vector n))
     (dotimes (i n)
     (setf (elt _positions_x i) (sys::peek buf ptr- :integer)) (incf ptr- 4)
     ))
   ;;
   self)
  )

(defclass camera::PhotoshelfService
  :super ros::object
  :slots ())

(setf (get camera::PhotoshelfService :md5sum-) "b956cbec76bf70325b60df673b2d7722")
(setf (get camera::PhotoshelfService :datatype-) "camera/PhotoshelfService")
(setf (get camera::PhotoshelfService :request) camera::PhotoshelfServiceRequest)
(setf (get camera::PhotoshelfService :response) camera::PhotoshelfServiceResponse)

(defmethod camera::PhotoshelfServiceRequest
  (:response () (instance camera::PhotoshelfServiceResponse :init)))

(setf (get camera::PhotoshelfServiceRequest :md5sum-) "b956cbec76bf70325b60df673b2d7722")
(setf (get camera::PhotoshelfServiceRequest :datatype-) "camera/PhotoshelfServiceRequest")
(setf (get camera::PhotoshelfServiceRequest :definition-)
      "int32 type
---
int32[] results
int32[] positions_z
int32[] positions_x
")

(setf (get camera::PhotoshelfServiceResponse :md5sum-) "b956cbec76bf70325b60df673b2d7722")
(setf (get camera::PhotoshelfServiceResponse :datatype-) "camera/PhotoshelfServiceResponse")
(setf (get camera::PhotoshelfServiceResponse :definition-)
      "int32 type
---
int32[] results
int32[] positions_z
int32[] positions_x
")



(provide :camera/PhotoshelfService "b956cbec76bf70325b60df673b2d7722")


