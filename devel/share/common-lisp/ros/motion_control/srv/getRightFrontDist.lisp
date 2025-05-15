; Auto-generated. Do not edit!


(cl:in-package motion_control-srv)


;//! \htmlinclude getRightFrontDist-request.msg.html

(cl:defclass <getRightFrontDist-request> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass getRightFrontDist-request (<getRightFrontDist-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <getRightFrontDist-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'getRightFrontDist-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name motion_control-srv:<getRightFrontDist-request> is deprecated: use motion_control-srv:getRightFrontDist-request instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <getRightFrontDist-request>) ostream)
  "Serializes a message object of type '<getRightFrontDist-request>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <getRightFrontDist-request>) istream)
  "Deserializes a message object of type '<getRightFrontDist-request>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<getRightFrontDist-request>)))
  "Returns string type for a service object of type '<getRightFrontDist-request>"
  "motion_control/getRightFrontDistRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'getRightFrontDist-request)))
  "Returns string type for a service object of type 'getRightFrontDist-request"
  "motion_control/getRightFrontDistRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<getRightFrontDist-request>)))
  "Returns md5sum for a message object of type '<getRightFrontDist-request>"
  "b33671b8766d8b0db5bcef50e936b90c")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'getRightFrontDist-request)))
  "Returns md5sum for a message object of type 'getRightFrontDist-request"
  "b33671b8766d8b0db5bcef50e936b90c")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<getRightFrontDist-request>)))
  "Returns full string definition for message of type '<getRightFrontDist-request>"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'getRightFrontDist-request)))
  "Returns full string definition for message of type 'getRightFrontDist-request"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <getRightFrontDist-request>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <getRightFrontDist-request>))
  "Converts a ROS message object to a list"
  (cl:list 'getRightFrontDist-request
))
;//! \htmlinclude getRightFrontDist-response.msg.html

(cl:defclass <getRightFrontDist-response> (roslisp-msg-protocol:ros-message)
  ((rightFrontDist
    :reader rightFrontDist
    :initarg :rightFrontDist
    :type cl:float
    :initform 0.0))
)

(cl:defclass getRightFrontDist-response (<getRightFrontDist-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <getRightFrontDist-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'getRightFrontDist-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name motion_control-srv:<getRightFrontDist-response> is deprecated: use motion_control-srv:getRightFrontDist-response instead.")))

(cl:ensure-generic-function 'rightFrontDist-val :lambda-list '(m))
(cl:defmethod rightFrontDist-val ((m <getRightFrontDist-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader motion_control-srv:rightFrontDist-val is deprecated.  Use motion_control-srv:rightFrontDist instead.")
  (rightFrontDist m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <getRightFrontDist-response>) ostream)
  "Serializes a message object of type '<getRightFrontDist-response>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'rightFrontDist))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <getRightFrontDist-response>) istream)
  "Deserializes a message object of type '<getRightFrontDist-response>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'rightFrontDist) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<getRightFrontDist-response>)))
  "Returns string type for a service object of type '<getRightFrontDist-response>"
  "motion_control/getRightFrontDistResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'getRightFrontDist-response)))
  "Returns string type for a service object of type 'getRightFrontDist-response"
  "motion_control/getRightFrontDistResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<getRightFrontDist-response>)))
  "Returns md5sum for a message object of type '<getRightFrontDist-response>"
  "b33671b8766d8b0db5bcef50e936b90c")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'getRightFrontDist-response)))
  "Returns md5sum for a message object of type 'getRightFrontDist-response"
  "b33671b8766d8b0db5bcef50e936b90c")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<getRightFrontDist-response>)))
  "Returns full string definition for message of type '<getRightFrontDist-response>"
  (cl:format cl:nil "float32 rightFrontDist~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'getRightFrontDist-response)))
  "Returns full string definition for message of type 'getRightFrontDist-response"
  (cl:format cl:nil "float32 rightFrontDist~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <getRightFrontDist-response>))
  (cl:+ 0
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <getRightFrontDist-response>))
  "Converts a ROS message object to a list"
  (cl:list 'getRightFrontDist-response
    (cl:cons ':rightFrontDist (rightFrontDist msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'getRightFrontDist)))
  'getRightFrontDist-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'getRightFrontDist)))
  'getRightFrontDist-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'getRightFrontDist)))
  "Returns string type for a service object of type '<getRightFrontDist>"
  "motion_control/getRightFrontDist")