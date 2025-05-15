; Auto-generated. Do not edit!


(cl:in-package camera-srv)


;//! \htmlinclude PhotoshelfService-request.msg.html

(cl:defclass <PhotoshelfService-request> (roslisp-msg-protocol:ros-message)
  ((type
    :reader type
    :initarg :type
    :type cl:integer
    :initform 0))
)

(cl:defclass PhotoshelfService-request (<PhotoshelfService-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <PhotoshelfService-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'PhotoshelfService-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name camera-srv:<PhotoshelfService-request> is deprecated: use camera-srv:PhotoshelfService-request instead.")))

(cl:ensure-generic-function 'type-val :lambda-list '(m))
(cl:defmethod type-val ((m <PhotoshelfService-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader camera-srv:type-val is deprecated.  Use camera-srv:type instead.")
  (type m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <PhotoshelfService-request>) ostream)
  "Serializes a message object of type '<PhotoshelfService-request>"
  (cl:let* ((signed (cl:slot-value msg 'type)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <PhotoshelfService-request>) istream)
  "Deserializes a message object of type '<PhotoshelfService-request>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'type) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<PhotoshelfService-request>)))
  "Returns string type for a service object of type '<PhotoshelfService-request>"
  "camera/PhotoshelfServiceRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'PhotoshelfService-request)))
  "Returns string type for a service object of type 'PhotoshelfService-request"
  "camera/PhotoshelfServiceRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<PhotoshelfService-request>)))
  "Returns md5sum for a message object of type '<PhotoshelfService-request>"
  "b956cbec76bf70325b60df673b2d7722")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'PhotoshelfService-request)))
  "Returns md5sum for a message object of type 'PhotoshelfService-request"
  "b956cbec76bf70325b60df673b2d7722")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<PhotoshelfService-request>)))
  "Returns full string definition for message of type '<PhotoshelfService-request>"
  (cl:format cl:nil "int32 type~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'PhotoshelfService-request)))
  "Returns full string definition for message of type 'PhotoshelfService-request"
  (cl:format cl:nil "int32 type~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <PhotoshelfService-request>))
  (cl:+ 0
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <PhotoshelfService-request>))
  "Converts a ROS message object to a list"
  (cl:list 'PhotoshelfService-request
    (cl:cons ':type (type msg))
))
;//! \htmlinclude PhotoshelfService-response.msg.html

(cl:defclass <PhotoshelfService-response> (roslisp-msg-protocol:ros-message)
  ((results
    :reader results
    :initarg :results
    :type (cl:vector cl:integer)
   :initform (cl:make-array 0 :element-type 'cl:integer :initial-element 0))
   (positions_z
    :reader positions_z
    :initarg :positions_z
    :type (cl:vector cl:integer)
   :initform (cl:make-array 0 :element-type 'cl:integer :initial-element 0))
   (positions_x
    :reader positions_x
    :initarg :positions_x
    :type (cl:vector cl:integer)
   :initform (cl:make-array 0 :element-type 'cl:integer :initial-element 0)))
)

(cl:defclass PhotoshelfService-response (<PhotoshelfService-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <PhotoshelfService-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'PhotoshelfService-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name camera-srv:<PhotoshelfService-response> is deprecated: use camera-srv:PhotoshelfService-response instead.")))

(cl:ensure-generic-function 'results-val :lambda-list '(m))
(cl:defmethod results-val ((m <PhotoshelfService-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader camera-srv:results-val is deprecated.  Use camera-srv:results instead.")
  (results m))

(cl:ensure-generic-function 'positions_z-val :lambda-list '(m))
(cl:defmethod positions_z-val ((m <PhotoshelfService-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader camera-srv:positions_z-val is deprecated.  Use camera-srv:positions_z instead.")
  (positions_z m))

(cl:ensure-generic-function 'positions_x-val :lambda-list '(m))
(cl:defmethod positions_x-val ((m <PhotoshelfService-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader camera-srv:positions_x-val is deprecated.  Use camera-srv:positions_x instead.")
  (positions_x m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <PhotoshelfService-response>) ostream)
  "Serializes a message object of type '<PhotoshelfService-response>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'results))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let* ((signed ele) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    ))
   (cl:slot-value msg 'results))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'positions_z))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let* ((signed ele) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    ))
   (cl:slot-value msg 'positions_z))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'positions_x))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let* ((signed ele) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    ))
   (cl:slot-value msg 'positions_x))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <PhotoshelfService-response>) istream)
  "Deserializes a message object of type '<PhotoshelfService-response>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'results) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'results)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:aref vals i) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296)))))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'positions_z) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'positions_z)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:aref vals i) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296)))))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'positions_x) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'positions_x)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:aref vals i) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296)))))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<PhotoshelfService-response>)))
  "Returns string type for a service object of type '<PhotoshelfService-response>"
  "camera/PhotoshelfServiceResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'PhotoshelfService-response)))
  "Returns string type for a service object of type 'PhotoshelfService-response"
  "camera/PhotoshelfServiceResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<PhotoshelfService-response>)))
  "Returns md5sum for a message object of type '<PhotoshelfService-response>"
  "b956cbec76bf70325b60df673b2d7722")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'PhotoshelfService-response)))
  "Returns md5sum for a message object of type 'PhotoshelfService-response"
  "b956cbec76bf70325b60df673b2d7722")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<PhotoshelfService-response>)))
  "Returns full string definition for message of type '<PhotoshelfService-response>"
  (cl:format cl:nil "int32[] results~%int32[] positions_z~%int32[] positions_x~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'PhotoshelfService-response)))
  "Returns full string definition for message of type 'PhotoshelfService-response"
  (cl:format cl:nil "int32[] results~%int32[] positions_z~%int32[] positions_x~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <PhotoshelfService-response>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'results) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'positions_z) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'positions_x) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <PhotoshelfService-response>))
  "Converts a ROS message object to a list"
  (cl:list 'PhotoshelfService-response
    (cl:cons ':results (results msg))
    (cl:cons ':positions_z (positions_z msg))
    (cl:cons ':positions_x (positions_x msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'PhotoshelfService)))
  'PhotoshelfService-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'PhotoshelfService)))
  'PhotoshelfService-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'PhotoshelfService)))
  "Returns string type for a service object of type '<PhotoshelfService>"
  "camera/PhotoshelfService")