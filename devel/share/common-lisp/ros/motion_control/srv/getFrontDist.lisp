; Auto-generated. Do not edit!


(cl:in-package motion_control-srv)


;//! \htmlinclude getFrontDist-request.msg.html

(cl:defclass <getFrontDist-request> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass getFrontDist-request (<getFrontDist-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <getFrontDist-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'getFrontDist-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name motion_control-srv:<getFrontDist-request> is deprecated: use motion_control-srv:getFrontDist-request instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <getFrontDist-request>) ostream)
  "Serializes a message object of type '<getFrontDist-request>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <getFrontDist-request>) istream)
  "Deserializes a message object of type '<getFrontDist-request>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<getFrontDist-request>)))
  "Returns string type for a service object of type '<getFrontDist-request>"
  "motion_control/getFrontDistRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'getFrontDist-request)))
  "Returns string type for a service object of type 'getFrontDist-request"
  "motion_control/getFrontDistRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<getFrontDist-request>)))
  "Returns md5sum for a message object of type '<getFrontDist-request>"
  "1e75451363e2baac9b8bfb0e12a9fdd0")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'getFrontDist-request)))
  "Returns md5sum for a message object of type 'getFrontDist-request"
  "1e75451363e2baac9b8bfb0e12a9fdd0")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<getFrontDist-request>)))
  "Returns full string definition for message of type '<getFrontDist-request>"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'getFrontDist-request)))
  "Returns full string definition for message of type 'getFrontDist-request"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <getFrontDist-request>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <getFrontDist-request>))
  "Converts a ROS message object to a list"
  (cl:list 'getFrontDist-request
))
;//! \htmlinclude getFrontDist-response.msg.html

(cl:defclass <getFrontDist-response> (roslisp-msg-protocol:ros-message)
  ((frontDist
    :reader frontDist
    :initarg :frontDist
    :type cl:float
    :initform 0.0))
)

(cl:defclass getFrontDist-response (<getFrontDist-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <getFrontDist-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'getFrontDist-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name motion_control-srv:<getFrontDist-response> is deprecated: use motion_control-srv:getFrontDist-response instead.")))

(cl:ensure-generic-function 'frontDist-val :lambda-list '(m))
(cl:defmethod frontDist-val ((m <getFrontDist-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader motion_control-srv:frontDist-val is deprecated.  Use motion_control-srv:frontDist instead.")
  (frontDist m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <getFrontDist-response>) ostream)
  "Serializes a message object of type '<getFrontDist-response>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'frontDist))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <getFrontDist-response>) istream)
  "Deserializes a message object of type '<getFrontDist-response>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'frontDist) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<getFrontDist-response>)))
  "Returns string type for a service object of type '<getFrontDist-response>"
  "motion_control/getFrontDistResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'getFrontDist-response)))
  "Returns string type for a service object of type 'getFrontDist-response"
  "motion_control/getFrontDistResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<getFrontDist-response>)))
  "Returns md5sum for a message object of type '<getFrontDist-response>"
  "1e75451363e2baac9b8bfb0e12a9fdd0")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'getFrontDist-response)))
  "Returns md5sum for a message object of type 'getFrontDist-response"
  "1e75451363e2baac9b8bfb0e12a9fdd0")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<getFrontDist-response>)))
  "Returns full string definition for message of type '<getFrontDist-response>"
  (cl:format cl:nil "float32 frontDist~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'getFrontDist-response)))
  "Returns full string definition for message of type 'getFrontDist-response"
  (cl:format cl:nil "float32 frontDist~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <getFrontDist-response>))
  (cl:+ 0
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <getFrontDist-response>))
  "Converts a ROS message object to a list"
  (cl:list 'getFrontDist-response
    (cl:cons ':frontDist (frontDist msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'getFrontDist)))
  'getFrontDist-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'getFrontDist)))
  'getFrontDist-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'getFrontDist)))
  "Returns string type for a service object of type '<getFrontDist>"
  "motion_control/getFrontDist")