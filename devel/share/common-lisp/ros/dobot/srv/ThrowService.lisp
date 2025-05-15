; Auto-generated. Do not edit!


(cl:in-package dobot-srv)


;//! \htmlinclude ThrowService-request.msg.html

(cl:defclass <ThrowService-request> (roslisp-msg-protocol:ros-message)
  ((pos_z
    :reader pos_z
    :initarg :pos_z
    :type cl:boolean
    :initform cl:nil)
   (mailbox_pos
    :reader mailbox_pos
    :initarg :mailbox_pos
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass ThrowService-request (<ThrowService-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ThrowService-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ThrowService-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name dobot-srv:<ThrowService-request> is deprecated: use dobot-srv:ThrowService-request instead.")))

(cl:ensure-generic-function 'pos_z-val :lambda-list '(m))
(cl:defmethod pos_z-val ((m <ThrowService-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dobot-srv:pos_z-val is deprecated.  Use dobot-srv:pos_z instead.")
  (pos_z m))

(cl:ensure-generic-function 'mailbox_pos-val :lambda-list '(m))
(cl:defmethod mailbox_pos-val ((m <ThrowService-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dobot-srv:mailbox_pos-val is deprecated.  Use dobot-srv:mailbox_pos instead.")
  (mailbox_pos m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ThrowService-request>) ostream)
  "Serializes a message object of type '<ThrowService-request>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'pos_z) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'mailbox_pos) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ThrowService-request>) istream)
  "Deserializes a message object of type '<ThrowService-request>"
    (cl:setf (cl:slot-value msg 'pos_z) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'mailbox_pos) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ThrowService-request>)))
  "Returns string type for a service object of type '<ThrowService-request>"
  "dobot/ThrowServiceRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ThrowService-request)))
  "Returns string type for a service object of type 'ThrowService-request"
  "dobot/ThrowServiceRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ThrowService-request>)))
  "Returns md5sum for a message object of type '<ThrowService-request>"
  "b5ca28a4c733f2992478f530e3af3cbb")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ThrowService-request)))
  "Returns md5sum for a message object of type 'ThrowService-request"
  "b5ca28a4c733f2992478f530e3af3cbb")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ThrowService-request>)))
  "Returns full string definition for message of type '<ThrowService-request>"
  (cl:format cl:nil "bool pos_z~%bool mailbox_pos~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ThrowService-request)))
  "Returns full string definition for message of type 'ThrowService-request"
  (cl:format cl:nil "bool pos_z~%bool mailbox_pos~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ThrowService-request>))
  (cl:+ 0
     1
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ThrowService-request>))
  "Converts a ROS message object to a list"
  (cl:list 'ThrowService-request
    (cl:cons ':pos_z (pos_z msg))
    (cl:cons ':mailbox_pos (mailbox_pos msg))
))
;//! \htmlinclude ThrowService-response.msg.html

(cl:defclass <ThrowService-response> (roslisp-msg-protocol:ros-message)
  ((success
    :reader success
    :initarg :success
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass ThrowService-response (<ThrowService-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ThrowService-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ThrowService-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name dobot-srv:<ThrowService-response> is deprecated: use dobot-srv:ThrowService-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <ThrowService-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader dobot-srv:success-val is deprecated.  Use dobot-srv:success instead.")
  (success m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ThrowService-response>) ostream)
  "Serializes a message object of type '<ThrowService-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ThrowService-response>) istream)
  "Deserializes a message object of type '<ThrowService-response>"
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ThrowService-response>)))
  "Returns string type for a service object of type '<ThrowService-response>"
  "dobot/ThrowServiceResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ThrowService-response)))
  "Returns string type for a service object of type 'ThrowService-response"
  "dobot/ThrowServiceResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ThrowService-response>)))
  "Returns md5sum for a message object of type '<ThrowService-response>"
  "b5ca28a4c733f2992478f530e3af3cbb")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ThrowService-response)))
  "Returns md5sum for a message object of type 'ThrowService-response"
  "b5ca28a4c733f2992478f530e3af3cbb")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ThrowService-response>)))
  "Returns full string definition for message of type '<ThrowService-response>"
  (cl:format cl:nil "bool success~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ThrowService-response)))
  "Returns full string definition for message of type 'ThrowService-response"
  (cl:format cl:nil "bool success~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ThrowService-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ThrowService-response>))
  "Converts a ROS message object to a list"
  (cl:list 'ThrowService-response
    (cl:cons ':success (success msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'ThrowService)))
  'ThrowService-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'ThrowService)))
  'ThrowService-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ThrowService)))
  "Returns string type for a service object of type '<ThrowService>"
  "dobot/ThrowService")