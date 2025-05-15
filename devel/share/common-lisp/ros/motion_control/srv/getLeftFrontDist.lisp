; Auto-generated. Do not edit!


(cl:in-package motion_control-srv)


;//! \htmlinclude getLeftFrontDist-request.msg.html

(cl:defclass <getLeftFrontDist-request> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass getLeftFrontDist-request (<getLeftFrontDist-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <getLeftFrontDist-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'getLeftFrontDist-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name motion_control-srv:<getLeftFrontDist-request> is deprecated: use motion_control-srv:getLeftFrontDist-request instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <getLeftFrontDist-request>) ostream)
  "Serializes a message object of type '<getLeftFrontDist-request>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <getLeftFrontDist-request>) istream)
  "Deserializes a message object of type '<getLeftFrontDist-request>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<getLeftFrontDist-request>)))
  "Returns string type for a service object of type '<getLeftFrontDist-request>"
  "motion_control/getLeftFrontDistRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'getLeftFrontDist-request)))
  "Returns string type for a service object of type 'getLeftFrontDist-request"
  "motion_control/getLeftFrontDistRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<getLeftFrontDist-request>)))
  "Returns md5sum for a message object of type '<getLeftFrontDist-request>"
  "4c23ace1d1a78b192ca63b2f98acf0d3")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'getLeftFrontDist-request)))
  "Returns md5sum for a message object of type 'getLeftFrontDist-request"
  "4c23ace1d1a78b192ca63b2f98acf0d3")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<getLeftFrontDist-request>)))
  "Returns full string definition for message of type '<getLeftFrontDist-request>"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'getLeftFrontDist-request)))
  "Returns full string definition for message of type 'getLeftFrontDist-request"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <getLeftFrontDist-request>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <getLeftFrontDist-request>))
  "Converts a ROS message object to a list"
  (cl:list 'getLeftFrontDist-request
))
;//! \htmlinclude getLeftFrontDist-response.msg.html

(cl:defclass <getLeftFrontDist-response> (roslisp-msg-protocol:ros-message)
  ((leftFrontDist
    :reader leftFrontDist
    :initarg :leftFrontDist
    :type cl:float
    :initform 0.0))
)

(cl:defclass getLeftFrontDist-response (<getLeftFrontDist-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <getLeftFrontDist-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'getLeftFrontDist-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name motion_control-srv:<getLeftFrontDist-response> is deprecated: use motion_control-srv:getLeftFrontDist-response instead.")))

(cl:ensure-generic-function 'leftFrontDist-val :lambda-list '(m))
(cl:defmethod leftFrontDist-val ((m <getLeftFrontDist-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader motion_control-srv:leftFrontDist-val is deprecated.  Use motion_control-srv:leftFrontDist instead.")
  (leftFrontDist m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <getLeftFrontDist-response>) ostream)
  "Serializes a message object of type '<getLeftFrontDist-response>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'leftFrontDist))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <getLeftFrontDist-response>) istream)
  "Deserializes a message object of type '<getLeftFrontDist-response>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'leftFrontDist) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<getLeftFrontDist-response>)))
  "Returns string type for a service object of type '<getLeftFrontDist-response>"
  "motion_control/getLeftFrontDistResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'getLeftFrontDist-response)))
  "Returns string type for a service object of type 'getLeftFrontDist-response"
  "motion_control/getLeftFrontDistResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<getLeftFrontDist-response>)))
  "Returns md5sum for a message object of type '<getLeftFrontDist-response>"
  "4c23ace1d1a78b192ca63b2f98acf0d3")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'getLeftFrontDist-response)))
  "Returns md5sum for a message object of type 'getLeftFrontDist-response"
  "4c23ace1d1a78b192ca63b2f98acf0d3")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<getLeftFrontDist-response>)))
  "Returns full string definition for message of type '<getLeftFrontDist-response>"
  (cl:format cl:nil "float32 leftFrontDist~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'getLeftFrontDist-response)))
  "Returns full string definition for message of type 'getLeftFrontDist-response"
  (cl:format cl:nil "float32 leftFrontDist~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <getLeftFrontDist-response>))
  (cl:+ 0
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <getLeftFrontDist-response>))
  "Converts a ROS message object to a list"
  (cl:list 'getLeftFrontDist-response
    (cl:cons ':leftFrontDist (leftFrontDist msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'getLeftFrontDist)))
  'getLeftFrontDist-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'getLeftFrontDist)))
  'getLeftFrontDist-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'getLeftFrontDist)))
  "Returns string type for a service object of type '<getLeftFrontDist>"
  "motion_control/getLeftFrontDist")