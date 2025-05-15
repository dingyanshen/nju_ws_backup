; Auto-generated. Do not edit!


(cl:in-package motion_control-srv)


;//! \htmlinclude getRightDist-request.msg.html

(cl:defclass <getRightDist-request> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass getRightDist-request (<getRightDist-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <getRightDist-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'getRightDist-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name motion_control-srv:<getRightDist-request> is deprecated: use motion_control-srv:getRightDist-request instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <getRightDist-request>) ostream)
  "Serializes a message object of type '<getRightDist-request>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <getRightDist-request>) istream)
  "Deserializes a message object of type '<getRightDist-request>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<getRightDist-request>)))
  "Returns string type for a service object of type '<getRightDist-request>"
  "motion_control/getRightDistRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'getRightDist-request)))
  "Returns string type for a service object of type 'getRightDist-request"
  "motion_control/getRightDistRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<getRightDist-request>)))
  "Returns md5sum for a message object of type '<getRightDist-request>"
  "46d3dd3b27754b2db4e99d35f4cb12c3")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'getRightDist-request)))
  "Returns md5sum for a message object of type 'getRightDist-request"
  "46d3dd3b27754b2db4e99d35f4cb12c3")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<getRightDist-request>)))
  "Returns full string definition for message of type '<getRightDist-request>"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'getRightDist-request)))
  "Returns full string definition for message of type 'getRightDist-request"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <getRightDist-request>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <getRightDist-request>))
  "Converts a ROS message object to a list"
  (cl:list 'getRightDist-request
))
;//! \htmlinclude getRightDist-response.msg.html

(cl:defclass <getRightDist-response> (roslisp-msg-protocol:ros-message)
  ((rightDist
    :reader rightDist
    :initarg :rightDist
    :type cl:float
    :initform 0.0))
)

(cl:defclass getRightDist-response (<getRightDist-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <getRightDist-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'getRightDist-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name motion_control-srv:<getRightDist-response> is deprecated: use motion_control-srv:getRightDist-response instead.")))

(cl:ensure-generic-function 'rightDist-val :lambda-list '(m))
(cl:defmethod rightDist-val ((m <getRightDist-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader motion_control-srv:rightDist-val is deprecated.  Use motion_control-srv:rightDist instead.")
  (rightDist m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <getRightDist-response>) ostream)
  "Serializes a message object of type '<getRightDist-response>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'rightDist))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <getRightDist-response>) istream)
  "Deserializes a message object of type '<getRightDist-response>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'rightDist) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<getRightDist-response>)))
  "Returns string type for a service object of type '<getRightDist-response>"
  "motion_control/getRightDistResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'getRightDist-response)))
  "Returns string type for a service object of type 'getRightDist-response"
  "motion_control/getRightDistResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<getRightDist-response>)))
  "Returns md5sum for a message object of type '<getRightDist-response>"
  "46d3dd3b27754b2db4e99d35f4cb12c3")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'getRightDist-response)))
  "Returns md5sum for a message object of type 'getRightDist-response"
  "46d3dd3b27754b2db4e99d35f4cb12c3")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<getRightDist-response>)))
  "Returns full string definition for message of type '<getRightDist-response>"
  (cl:format cl:nil "float32 rightDist~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'getRightDist-response)))
  "Returns full string definition for message of type 'getRightDist-response"
  (cl:format cl:nil "float32 rightDist~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <getRightDist-response>))
  (cl:+ 0
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <getRightDist-response>))
  "Converts a ROS message object to a list"
  (cl:list 'getRightDist-response
    (cl:cons ':rightDist (rightDist msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'getRightDist)))
  'getRightDist-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'getRightDist)))
  'getRightDist-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'getRightDist)))
  "Returns string type for a service object of type '<getRightDist>"
  "motion_control/getRightDist")