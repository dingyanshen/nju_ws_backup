; Auto-generated. Do not edit!


(cl:in-package dobot-srv)


;//! \htmlinclude GraspService-request.msg.html

(cl:defclass <GraspService-request> (roslisp-msg-protocol:ros-message)
  ((shelf_z
    :reader shelf_z
    :initarg :shelf_z
    :type cl:boolean
    :initform cl:nil)
   (pos_z
    :reader pos_z
    :initarg :pos_z
    :type cl:boolean
    :initform cl:nil)
   (error_x
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

(cl:defclass GraspService-request (<GraspService-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GraspService-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GraspService-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name dobot-srv:<GraspService-request> is deprecated: use dobot-srv:GraspService-request instead.")))

(cl:ensure-generic-function 'shelf_z-val :lambda-list '(m))
(cl:defmethod shelf_z-val ((m <GraspService-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dobot-srv:shelf_z-val is deprecated.  Use dobot-srv:shelf_z instead.")
  (shelf_z m))

(cl:ensure-generic-function 'pos_z-val :lambda-list '(m))
(cl:defmethod pos_z-val ((m <GraspService-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dobot-srv:pos_z-val is deprecated.  Use dobot-srv:pos_z instead.")
  (pos_z m))

(cl:ensure-generic-function 'error_x-val :lambda-list '(m))
(cl:defmethod error_x-val ((m <GraspService-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dobot-srv:error_x-val is deprecated.  Use dobot-srv:error_x instead.")
  (error_x m))

(cl:ensure-generic-function 'error_y-val :lambda-list '(m))
(cl:defmethod error_y-val ((m <GraspService-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dobot-srv:error_y-val is deprecated.  Use dobot-srv:error_y instead.")
  (error_y m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GraspService-request>) ostream)
  "Serializes a message object of type '<GraspService-request>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'shelf_z) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'pos_z) 1 0)) ostream)
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
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GraspService-request>) istream)
  "Deserializes a message object of type '<GraspService-request>"
    (cl:setf (cl:slot-value msg 'shelf_z) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'pos_z) (cl:not (cl:zerop (cl:read-byte istream))))
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GraspService-request>)))
  "Returns string type for a service object of type '<GraspService-request>"
  "dobot/GraspServiceRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GraspService-request)))
  "Returns string type for a service object of type 'GraspService-request"
  "dobot/GraspServiceRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GraspService-request>)))
  "Returns md5sum for a message object of type '<GraspService-request>"
  "1e7660736e31416d5bfaa2f3f2cf2e32")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GraspService-request)))
  "Returns md5sum for a message object of type 'GraspService-request"
  "1e7660736e31416d5bfaa2f3f2cf2e32")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GraspService-request>)))
  "Returns full string definition for message of type '<GraspService-request>"
  (cl:format cl:nil "bool shelf_z~%bool pos_z~%float32 error_x~%float32 error_y~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GraspService-request)))
  "Returns full string definition for message of type 'GraspService-request"
  (cl:format cl:nil "bool shelf_z~%bool pos_z~%float32 error_x~%float32 error_y~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GraspService-request>))
  (cl:+ 0
     1
     1
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GraspService-request>))
  "Converts a ROS message object to a list"
  (cl:list 'GraspService-request
    (cl:cons ':shelf_z (shelf_z msg))
    (cl:cons ':pos_z (pos_z msg))
    (cl:cons ':error_x (error_x msg))
    (cl:cons ':error_y (error_y msg))
))
;//! \htmlinclude GraspService-response.msg.html

(cl:defclass <GraspService-response> (roslisp-msg-protocol:ros-message)
  ((success
    :reader success
    :initarg :success
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass GraspService-response (<GraspService-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GraspService-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GraspService-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name dobot-srv:<GraspService-response> is deprecated: use dobot-srv:GraspService-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <GraspService-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dobot-srv:success-val is deprecated.  Use dobot-srv:success instead.")
  (success m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GraspService-response>) ostream)
  "Serializes a message object of type '<GraspService-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GraspService-response>) istream)
  "Deserializes a message object of type '<GraspService-response>"
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GraspService-response>)))
  "Returns string type for a service object of type '<GraspService-response>"
  "dobot/GraspServiceResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GraspService-response)))
  "Returns string type for a service object of type 'GraspService-response"
  "dobot/GraspServiceResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GraspService-response>)))
  "Returns md5sum for a message object of type '<GraspService-response>"
  "1e7660736e31416d5bfaa2f3f2cf2e32")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GraspService-response)))
  "Returns md5sum for a message object of type 'GraspService-response"
  "1e7660736e31416d5bfaa2f3f2cf2e32")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GraspService-response>)))
  "Returns full string definition for message of type '<GraspService-response>"
  (cl:format cl:nil "bool success~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GraspService-response)))
  "Returns full string definition for message of type 'GraspService-response"
  (cl:format cl:nil "bool success~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GraspService-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GraspService-response>))
  "Converts a ROS message object to a list"
  (cl:list 'GraspService-response
    (cl:cons ':success (success msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'GraspService)))
  'GraspService-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'GraspService)))
  'GraspService-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GraspService)))
  "Returns string type for a service object of type '<GraspService>"
  "dobot/GraspService")