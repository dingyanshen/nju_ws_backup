; Auto-generated. Do not edit!


(cl:in-package camera-srv)


;//! \htmlinclude PhotoService-request.msg.html

(cl:defclass <PhotoService-request> (roslisp-msg-protocol:ros-message)
  ((type
    :reader type
    :initarg :type
    :type cl:integer
    :initform 0))
)

(cl:defclass PhotoService-request (<PhotoService-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <PhotoService-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'PhotoService-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name camera-srv:<PhotoService-request> is deprecated: use camera-srv:PhotoService-request instead.")))

(cl:ensure-generic-function 'type-val :lambda-list '(m))
(cl:defmethod type-val ((m <PhotoService-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader camera-srv:type-val is deprecated.  Use camera-srv:type instead.")
  (type m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <PhotoService-request>) ostream)
  "Serializes a message object of type '<PhotoService-request>"
  (cl:let* ((signed (cl:slot-value msg 'type)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <PhotoService-request>) istream)
  "Deserializes a message object of type '<PhotoService-request>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'type) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<PhotoService-request>)))
  "Returns string type for a service object of type '<PhotoService-request>"
  "camera/PhotoServiceRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'PhotoService-request)))
  "Returns string type for a service object of type 'PhotoService-request"
  "camera/PhotoServiceRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<PhotoService-request>)))
  "Returns md5sum for a message object of type '<PhotoService-request>"
  "117e6535d65959983e49d94959e9d276")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'PhotoService-request)))
  "Returns md5sum for a message object of type 'PhotoService-request"
  "117e6535d65959983e49d94959e9d276")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<PhotoService-request>)))
  "Returns full string definition for message of type '<PhotoService-request>"
  (cl:format cl:nil "int32 type~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'PhotoService-request)))
  "Returns full string definition for message of type 'PhotoService-request"
  (cl:format cl:nil "int32 type~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <PhotoService-request>))
  (cl:+ 0
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <PhotoService-request>))
  "Converts a ROS message object to a list"
  (cl:list 'PhotoService-request
    (cl:cons ':type (type msg))
))
;//! \htmlinclude PhotoService-response.msg.html

(cl:defclass <PhotoService-response> (roslisp-msg-protocol:ros-message)
  ((error_x
    :reader error_x
    :initarg :error_x
    :type cl:float
    :initform 0.0)
   (error_y
    :reader error_y
    :initarg :error_y
    :type cl:float
    :initform 0.0))
)

(cl:defclass PhotoService-response (<PhotoService-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <PhotoService-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'PhotoService-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name camera-srv:<PhotoService-response> is deprecated: use camera-srv:PhotoService-response instead.")))

(cl:ensure-generic-function 'error_x-val :lambda-list '(m))
(cl:defmethod error_x-val ((m <PhotoService-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader camera-srv:error_x-val is deprecated.  Use camera-srv:error_x instead.")
  (error_x m))

(cl:ensure-generic-function 'error_y-val :lambda-list '(m))
(cl:defmethod error_y-val ((m <PhotoService-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader camera-srv:error_y-val is deprecated.  Use camera-srv:error_y instead.")
  (error_y m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <PhotoService-response>) ostream)
  "Serializes a message object of type '<PhotoService-response>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'error_x))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'error_y))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <PhotoService-response>) istream)
  "Deserializes a message object of type '<PhotoService-response>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'error_x) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'error_y) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<PhotoService-response>)))
  "Returns string type for a service object of type '<PhotoService-response>"
  "camera/PhotoServiceResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'PhotoService-response)))
  "Returns string type for a service object of type 'PhotoService-response"
  "camera/PhotoServiceResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<PhotoService-response>)))
  "Returns md5sum for a message object of type '<PhotoService-response>"
  "117e6535d65959983e49d94959e9d276")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'PhotoService-response)))
  "Returns md5sum for a message object of type 'PhotoService-response"
  "117e6535d65959983e49d94959e9d276")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<PhotoService-response>)))
  "Returns full string definition for message of type '<PhotoService-response>"
  (cl:format cl:nil "float32 error_x~%float32 error_y~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'PhotoService-response)))
  "Returns full string definition for message of type 'PhotoService-response"
  (cl:format cl:nil "float32 error_x~%float32 error_y~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <PhotoService-response>))
  (cl:+ 0
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <PhotoService-response>))
  "Converts a ROS message object to a list"
  (cl:list 'PhotoService-response
    (cl:cons ':error_x (error_x msg))
    (cl:cons ':error_y (error_y msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'PhotoService)))
  'PhotoService-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'PhotoService)))
  'PhotoService-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'PhotoService)))
  "Returns string type for a service object of type '<PhotoService>"
  "camera/PhotoService")