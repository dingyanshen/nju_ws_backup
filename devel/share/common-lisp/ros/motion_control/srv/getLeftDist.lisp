; Auto-generated. Do not edit!


(cl:in-package motion_control-srv)


;//! \htmlinclude getLeftDist-request.msg.html

(cl:defclass <getLeftDist-request> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass getLeftDist-request (<getLeftDist-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <getLeftDist-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'getLeftDist-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name motion_control-srv:<getLeftDist-request> is deprecated: use motion_control-srv:getLeftDist-request instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <getLeftDist-request>) ostream)
  "Serializes a message object of type '<getLeftDist-request>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <getLeftDist-request>) istream)
  "Deserializes a message object of type '<getLeftDist-request>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<getLeftDist-request>)))
  "Returns string type for a service object of type '<getLeftDist-request>"
  "motion_control/getLeftDistRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'getLeftDist-request)))
  "Returns string type for a service object of type 'getLeftDist-request"
  "motion_control/getLeftDistRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<getLeftDist-request>)))
  "Returns md5sum for a message object of type '<getLeftDist-request>"
  "4e0d27729a997215618b53389df19443")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'getLeftDist-request)))
  "Returns md5sum for a message object of type 'getLeftDist-request"
  "4e0d27729a997215618b53389df19443")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<getLeftDist-request>)))
  "Returns full string definition for message of type '<getLeftDist-request>"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'getLeftDist-request)))
  "Returns full string definition for message of type 'getLeftDist-request"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <getLeftDist-request>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <getLeftDist-request>))
  "Converts a ROS message object to a list"
  (cl:list 'getLeftDist-request
))
;//! \htmlinclude getLeftDist-response.msg.html

(cl:defclass <getLeftDist-response> (roslisp-msg-protocol:ros-message)
  ((leftDist
    :reader leftDist
    :initarg :leftDist
    :type cl:float
    :initform 0.0))
)

(cl:defclass getLeftDist-response (<getLeftDist-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <getLeftDist-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'getLeftDist-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name motion_control-srv:<getLeftDist-response> is deprecated: use motion_control-srv:getLeftDist-response instead.")))

(cl:ensure-generic-function 'leftDist-val :lambda-list '(m))
(cl:defmethod leftDist-val ((m <getLeftDist-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader motion_control-srv:leftDist-val is deprecated.  Use motion_control-srv:leftDist instead.")
  (leftDist m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <getLeftDist-response>) ostream)
  "Serializes a message object of type '<getLeftDist-response>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'leftDist))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <getLeftDist-response>) istream)
  "Deserializes a message object of type '<getLeftDist-response>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'leftDist) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<getLeftDist-response>)))
  "Returns string type for a service object of type '<getLeftDist-response>"
  "motion_control/getLeftDistResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'getLeftDist-response)))
  "Returns string type for a service object of type 'getLeftDist-response"
  "motion_control/getLeftDistResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<getLeftDist-response>)))
  "Returns md5sum for a message object of type '<getLeftDist-response>"
  "4e0d27729a997215618b53389df19443")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'getLeftDist-response)))
  "Returns md5sum for a message object of type 'getLeftDist-response"
  "4e0d27729a997215618b53389df19443")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<getLeftDist-response>)))
  "Returns full string definition for message of type '<getLeftDist-response>"
  (cl:format cl:nil "float32 leftDist~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'getLeftDist-response)))
  "Returns full string definition for message of type 'getLeftDist-response"
  (cl:format cl:nil "float32 leftDist~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <getLeftDist-response>))
  (cl:+ 0
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <getLeftDist-response>))
  "Converts a ROS message object to a list"
  (cl:list 'getLeftDist-response
    (cl:cons ':leftDist (leftDist msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'getLeftDist)))
  'getLeftDist-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'getLeftDist)))
  'getLeftDist-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'getLeftDist)))
  "Returns string type for a service object of type '<getLeftDist>"
  "motion_control/getLeftDist")