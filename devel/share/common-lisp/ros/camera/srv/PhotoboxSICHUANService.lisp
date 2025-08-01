; Auto-generated. Do not edit!


(cl:in-package camera-srv)


;//! \htmlinclude PhotoboxSICHUANService-request.msg.html

(cl:defclass <PhotoboxSICHUANService-request> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass PhotoboxSICHUANService-request (<PhotoboxSICHUANService-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <PhotoboxSICHUANService-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'PhotoboxSICHUANService-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name camera-srv:<PhotoboxSICHUANService-request> is deprecated: use camera-srv:PhotoboxSICHUANService-request instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <PhotoboxSICHUANService-request>) ostream)
  "Serializes a message object of type '<PhotoboxSICHUANService-request>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <PhotoboxSICHUANService-request>) istream)
  "Deserializes a message object of type '<PhotoboxSICHUANService-request>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<PhotoboxSICHUANService-request>)))
  "Returns string type for a service object of type '<PhotoboxSICHUANService-request>"
  "camera/PhotoboxSICHUANServiceRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'PhotoboxSICHUANService-request)))
  "Returns string type for a service object of type 'PhotoboxSICHUANService-request"
  "camera/PhotoboxSICHUANServiceRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<PhotoboxSICHUANService-request>)))
  "Returns md5sum for a message object of type '<PhotoboxSICHUANService-request>"
  "034a8e20d6a306665e3a5b340fab3f09")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'PhotoboxSICHUANService-request)))
  "Returns md5sum for a message object of type 'PhotoboxSICHUANService-request"
  "034a8e20d6a306665e3a5b340fab3f09")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<PhotoboxSICHUANService-request>)))
  "Returns full string definition for message of type '<PhotoboxSICHUANService-request>"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'PhotoboxSICHUANService-request)))
  "Returns full string definition for message of type 'PhotoboxSICHUANService-request"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <PhotoboxSICHUANService-request>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <PhotoboxSICHUANService-request>))
  "Converts a ROS message object to a list"
  (cl:list 'PhotoboxSICHUANService-request
))
;//! \htmlinclude PhotoboxSICHUANService-response.msg.html

(cl:defclass <PhotoboxSICHUANService-response> (roslisp-msg-protocol:ros-message)
  ((result
    :reader result
    :initarg :result
    :type cl:integer
    :initform 0))
)

(cl:defclass PhotoboxSICHUANService-response (<PhotoboxSICHUANService-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <PhotoboxSICHUANService-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'PhotoboxSICHUANService-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name camera-srv:<PhotoboxSICHUANService-response> is deprecated: use camera-srv:PhotoboxSICHUANService-response instead.")))

(cl:ensure-generic-function 'result-val :lambda-list '(m))
(cl:defmethod result-val ((m <PhotoboxSICHUANService-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader camera-srv:result-val is deprecated.  Use camera-srv:result instead.")
  (result m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <PhotoboxSICHUANService-response>) ostream)
  "Serializes a message object of type '<PhotoboxSICHUANService-response>"
  (cl:let* ((signed (cl:slot-value msg 'result)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <PhotoboxSICHUANService-response>) istream)
  "Deserializes a message object of type '<PhotoboxSICHUANService-response>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'result) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<PhotoboxSICHUANService-response>)))
  "Returns string type for a service object of type '<PhotoboxSICHUANService-response>"
  "camera/PhotoboxSICHUANServiceResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'PhotoboxSICHUANService-response)))
  "Returns string type for a service object of type 'PhotoboxSICHUANService-response"
  "camera/PhotoboxSICHUANServiceResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<PhotoboxSICHUANService-response>)))
  "Returns md5sum for a message object of type '<PhotoboxSICHUANService-response>"
  "034a8e20d6a306665e3a5b340fab3f09")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'PhotoboxSICHUANService-response)))
  "Returns md5sum for a message object of type 'PhotoboxSICHUANService-response"
  "034a8e20d6a306665e3a5b340fab3f09")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<PhotoboxSICHUANService-response>)))
  "Returns full string definition for message of type '<PhotoboxSICHUANService-response>"
  (cl:format cl:nil "int32 result~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'PhotoboxSICHUANService-response)))
  "Returns full string definition for message of type 'PhotoboxSICHUANService-response"
  (cl:format cl:nil "int32 result~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <PhotoboxSICHUANService-response>))
  (cl:+ 0
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <PhotoboxSICHUANService-response>))
  "Converts a ROS message object to a list"
  (cl:list 'PhotoboxSICHUANService-response
    (cl:cons ':result (result msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'PhotoboxSICHUANService)))
  'PhotoboxSICHUANService-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'PhotoboxSICHUANService)))
  'PhotoboxSICHUANService-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'PhotoboxSICHUANService)))
  "Returns string type for a service object of type '<PhotoboxSICHUANService>"
  "camera/PhotoboxSICHUANService")