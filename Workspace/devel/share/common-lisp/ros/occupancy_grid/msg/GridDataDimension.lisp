; Auto-generated. Do not edit!


(cl:in-package occupancy_grid-msg)


;//! \htmlinclude GridDataDimension.msg.html

(cl:defclass <GridDataDimension> (roslisp-msg-protocol:ros-message)
  ((label
    :reader label
    :initarg :label
    :type cl:string
    :initform "")
   (size
    :reader size
    :initarg :size
    :type cl:integer
    :initform 0)
   (stride
    :reader stride
    :initarg :stride
    :type cl:integer
    :initform 0))
)

(cl:defclass GridDataDimension (<GridDataDimension>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GridDataDimension>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GridDataDimension)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name occupancy_grid-msg:<GridDataDimension> is deprecated: use occupancy_grid-msg:GridDataDimension instead.")))

(cl:ensure-generic-function 'label-val :lambda-list '(m))
(cl:defmethod label-val ((m <GridDataDimension>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader occupancy_grid-msg:label-val is deprecated.  Use occupancy_grid-msg:label instead.")
  (label m))

(cl:ensure-generic-function 'size-val :lambda-list '(m))
(cl:defmethod size-val ((m <GridDataDimension>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader occupancy_grid-msg:size-val is deprecated.  Use occupancy_grid-msg:size instead.")
  (size m))

(cl:ensure-generic-function 'stride-val :lambda-list '(m))
(cl:defmethod stride-val ((m <GridDataDimension>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader occupancy_grid-msg:stride-val is deprecated.  Use occupancy_grid-msg:stride instead.")
  (stride m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GridDataDimension>) ostream)
  "Serializes a message object of type '<GridDataDimension>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'label))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'label))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'size)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'size)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'size)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'size)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'stride)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'stride)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'stride)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'stride)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GridDataDimension>) istream)
  "Deserializes a message object of type '<GridDataDimension>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'label) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'label) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'size)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'size)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'size)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'size)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'stride)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'stride)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'stride)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'stride)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GridDataDimension>)))
  "Returns string type for a message object of type '<GridDataDimension>"
  "occupancy_grid/GridDataDimension")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GridDataDimension)))
  "Returns string type for a message object of type 'GridDataDimension"
  "occupancy_grid/GridDataDimension")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GridDataDimension>)))
  "Returns md5sum for a message object of type '<GridDataDimension>"
  "4cd0c83a8683deae40ecdac60e53bfa8")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GridDataDimension)))
  "Returns md5sum for a message object of type 'GridDataDimension"
  "4cd0c83a8683deae40ecdac60e53bfa8")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GridDataDimension>)))
  "Returns full string definition for message of type '<GridDataDimension>"
  (cl:format cl:nil "string label~%uint32 size~%uint32 stride~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GridDataDimension)))
  "Returns full string definition for message of type 'GridDataDimension"
  (cl:format cl:nil "string label~%uint32 size~%uint32 stride~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GridDataDimension>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'label))
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GridDataDimension>))
  "Converts a ROS message object to a list"
  (cl:list 'GridDataDimension
    (cl:cons ':label (label msg))
    (cl:cons ':size (size msg))
    (cl:cons ':stride (stride msg))
))
