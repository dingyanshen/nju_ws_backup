; Auto-generated. Do not edit!


(cl:in-package motion_control-srv)


;//! \htmlinclude getDiffAngle-request.msg.html

(cl:defclass <getDiffAngle-request> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass getDiffAngle-request (<getDiffAngle-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <getDiffAngle-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'getDiffAngle-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name motion_control-srv:<getDiffAngle-request> is deprecated: use motion_control-srv:getDiffAngle-request instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <getDiffAngle-request>) ostream)
  "Serializes a message object of type '<getDiffAngle-request>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <getDiffAngle-request>) istream)
  "Deserializes a message object of type '<getDiffAngle-request>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<getDiffAngle-request>)))
  "Returns string type for a service object of type '<getDiffAngle-request>"
  "motion_control/getDiffAngleRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'getDiffAngle-request)))
  "Returns string type for a service object of type 'getDiffAngle-request"
  "motion_control/getDiffAngleRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<getDiffAngle-request>)))
  "Returns md5sum for a message object of type '<getDiffAngle-request>"
  "39a7032f716c41fca7c3cc6c27125bc3")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'getDiffAngle-request)))
  "Returns md5sum for a message object of type 'getDiffAngle-request"
  "39a7032f716c41fca7c3cc6c27125bc3")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<getDiffAngle-request>)))
  "Returns full string definition for message of type '<getDiffAngle-request>"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'getDiffAngle-request)))
  "Returns full string definition for message of type 'getDiffAngle-request"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <getDiffAngle-request>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <getDiffAngle-request>))
  "Converts a ROS message object to a list"
  (cl:list 'getDiffAngle-request
))
;//! \htmlinclude getDiffAngle-response.msg.html

(cl:defclass <getDiffAngle-response> (roslisp-msg-protocol:ros-message)
  ((diffAngle
    :reader diffAngle
    :initarg :diffAngle
    :type cl:float
    :initform 0.0))
)

(cl:defclass getDiffAngle-response (<getDiffAngle-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <getDiffAngle-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'getDiffAngle-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name motion_control-srv:<getDiffAngle-response> is deprecated: use motion_control-srv:getDiffAngle-response instead.")))

(cl:ensure-generic-function 'diffAngle-val :lambda-list '(m))
(cl:defmethod diffAngle-val ((m <getDiffAngle-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader motion_control-srv:diffAngle-val is deprecated.  Use motion_control-srv:diffAngle instead.")
  (diffAngle m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <getDiffAngle-response>) ostream)
  "Serializes a message object of type '<getDiffAngle-response>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'diffAngle))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <getDiffAngle-response>) istream)
  "Deserializes a message object of type '<getDiffAngle-response>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'diffAngle) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<getDiffAngle-response>)))
  "Returns string type for a service object of type '<getDiffAngle-response>"
  "motion_control/getDiffAngleResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'getDiffAngle-response)))
  "Returns string type for a service object of type 'getDiffAngle-response"
  "motion_control/getDiffAngleResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<getDiffAngle-response>)))
  "Returns md5sum for a message object of type '<getDiffAngle-response>"
  "39a7032f716c41fca7c3cc6c27125bc3")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'getDiffAngle-response)))
  "Returns md5sum for a message object of type 'getDiffAngle-response"
  "39a7032f716c41fca7c3cc6c27125bc3")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<getDiffAngle-response>)))
  "Returns full string definition for message of type '<getDiffAngle-response>"
  (cl:format cl:nil "float32 diffAngle~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'getDiffAngle-response)))
  "Returns full string definition for message of type 'getDiffAngle-response"
  (cl:format cl:nil "float32 diffAngle~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <getDiffAngle-response>))
  (cl:+ 0
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <getDiffAngle-response>))
  "Converts a ROS message object to a list"
  (cl:list 'getDiffAngle-response
    (cl:cons ':diffAngle (diffAngle msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'getDiffAngle)))
  'getDiffAngle-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'getDiffAngle)))
  'getDiffAngle-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'getDiffAngle)))
  "Returns string type for a service object of type '<getDiffAngle>"
  "motion_control/getDiffAngle")