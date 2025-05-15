; Auto-generated. Do not edit!


(cl:in-package camera-srv)


;//! \htmlinclude PhotoboxService-request.msg.html

(cl:defclass <PhotoboxService-request> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass PhotoboxService-request (<PhotoboxService-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <PhotoboxService-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'PhotoboxService-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name camera-srv:<PhotoboxService-request> is deprecated: use camera-srv:PhotoboxService-request instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <PhotoboxService-request>) ostream)
  "Serializes a message object of type '<PhotoboxService-request>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <PhotoboxService-request>) istream)
  "Deserializes a message object of type '<PhotoboxService-request>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<PhotoboxService-request>)))
  "Returns string type for a service object of type '<PhotoboxService-request>"
  "camera/PhotoboxServiceRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'PhotoboxService-request)))
  "Returns string type for a service object of type 'PhotoboxService-request"
  "camera/PhotoboxServiceRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<PhotoboxService-request>)))
  "Returns md5sum for a message object of type '<PhotoboxService-request>"
  "034a8e20d6a306665e3a5b340fab3f09")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'PhotoboxService-request)))
  "Returns md5sum for a message object of type 'PhotoboxService-request"
  "034a8e20d6a306665e3a5b340fab3f09")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<PhotoboxService-request>)))
  "Returns full string definition for message of type '<PhotoboxService-request>"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'PhotoboxService-request)))
  "Returns full string definition for message of type 'PhotoboxService-request"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <PhotoboxService-request>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <PhotoboxService-request>))
  "Converts a ROS message object to a list"
  (cl:list 'PhotoboxService-request
))
;//! \htmlinclude PhotoboxService-response.msg.html

(cl:defclass <PhotoboxService-response> (roslisp-msg-protocol:ros-message)
  ((result
    :reader result
    :initarg :result
    :type cl:integer
    :initform 0))
)

(cl:defclass PhotoboxService-response (<PhotoboxService-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <PhotoboxService-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'PhotoboxService-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name camera-srv:<PhotoboxService-response> is deprecated: use camera-srv:PhotoboxService-response instead.")))

(cl:ensure-generic-function 'result-val :lambda-list '(m))
(cl:defmethod result-val ((m <PhotoboxService-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader camera-srv:result-val is deprecated.  Use camera-srv:result instead.")
  (result m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <PhotoboxService-response>) ostream)
  "Serializes a message object of type '<PhotoboxService-response>"
  (cl:let* ((signed (cl:slot-value msg 'result)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <PhotoboxService-response>) istream)
  "Deserializes a message object of type '<PhotoboxService-response>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'result) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<PhotoboxService-response>)))
  "Returns string type for a service object of type '<PhotoboxService-response>"
  "camera/PhotoboxServiceResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'PhotoboxService-response)))
  "Returns string type for a service object of type 'PhotoboxService-response"
  "camera/PhotoboxServiceResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<PhotoboxService-response>)))
  "Returns md5sum for a message object of type '<PhotoboxService-response>"
  "034a8e20d6a306665e3a5b340fab3f09")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'PhotoboxService-response)))
  "Returns md5sum for a message object of type 'PhotoboxService-response"
  "034a8e20d6a306665e3a5b340fab3f09")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<PhotoboxService-response>)))
  "Returns full string definition for message of type '<PhotoboxService-response>"
  (cl:format cl:nil "int32 result~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'PhotoboxService-response)))
  "Returns full string definition for message of type 'PhotoboxService-response"
  (cl:format cl:nil "int32 result~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <PhotoboxService-response>))
  (cl:+ 0
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <PhotoboxService-response>))
  "Converts a ROS message object to a list"
  (cl:list 'PhotoboxService-response
    (cl:cons ':result (result msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'PhotoboxService)))
  'PhotoboxService-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'PhotoboxService)))
  'PhotoboxService-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'PhotoboxService)))
  "Returns string type for a service object of type '<PhotoboxService>"
  "camera/PhotoboxService")