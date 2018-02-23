; Auto-generated. Do not edit!


(cl:in-package occupancy_grid-msg)


;//! \htmlinclude OccupancyGridHeader.msg.html

(cl:defclass <OccupancyGridHeader> (roslisp-msg-protocol:ros-message)
  ((cameraZMax
    :reader cameraZMax
    :initarg :cameraZMax
    :type cl:integer
    :initform 0)
   (cameraXMax
    :reader cameraXMax
    :initarg :cameraXMax
    :type cl:integer
    :initform 0)
   (cameraYOffset
    :reader cameraYOffset
    :initarg :cameraYOffset
    :type cl:float
    :initform 0.0)
   (gridResolution
    :reader gridResolution
    :initarg :gridResolution
    :type cl:float
    :initform 0.0)
   (gridCameraZ
    :reader gridCameraZ
    :initarg :gridCameraZ
    :type cl:integer
    :initform 0)
   (gridCameraX
    :reader gridCameraX
    :initarg :gridCameraX
    :type cl:integer
    :initform 0))
)

(cl:defclass OccupancyGridHeader (<OccupancyGridHeader>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <OccupancyGridHeader>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'OccupancyGridHeader)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name occupancy_grid-msg:<OccupancyGridHeader> is deprecated: use occupancy_grid-msg:OccupancyGridHeader instead.")))

(cl:ensure-generic-function 'cameraZMax-val :lambda-list '(m))
(cl:defmethod cameraZMax-val ((m <OccupancyGridHeader>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader occupancy_grid-msg:cameraZMax-val is deprecated.  Use occupancy_grid-msg:cameraZMax instead.")
  (cameraZMax m))

(cl:ensure-generic-function 'cameraXMax-val :lambda-list '(m))
(cl:defmethod cameraXMax-val ((m <OccupancyGridHeader>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader occupancy_grid-msg:cameraXMax-val is deprecated.  Use occupancy_grid-msg:cameraXMax instead.")
  (cameraXMax m))

(cl:ensure-generic-function 'cameraYOffset-val :lambda-list '(m))
(cl:defmethod cameraYOffset-val ((m <OccupancyGridHeader>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader occupancy_grid-msg:cameraYOffset-val is deprecated.  Use occupancy_grid-msg:cameraYOffset instead.")
  (cameraYOffset m))

(cl:ensure-generic-function 'gridResolution-val :lambda-list '(m))
(cl:defmethod gridResolution-val ((m <OccupancyGridHeader>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader occupancy_grid-msg:gridResolution-val is deprecated.  Use occupancy_grid-msg:gridResolution instead.")
  (gridResolution m))

(cl:ensure-generic-function 'gridCameraZ-val :lambda-list '(m))
(cl:defmethod gridCameraZ-val ((m <OccupancyGridHeader>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader occupancy_grid-msg:gridCameraZ-val is deprecated.  Use occupancy_grid-msg:gridCameraZ instead.")
  (gridCameraZ m))

(cl:ensure-generic-function 'gridCameraX-val :lambda-list '(m))
(cl:defmethod gridCameraX-val ((m <OccupancyGridHeader>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader occupancy_grid-msg:gridCameraX-val is deprecated.  Use occupancy_grid-msg:gridCameraX instead.")
  (gridCameraX m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <OccupancyGridHeader>) ostream)
  "Serializes a message object of type '<OccupancyGridHeader>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'cameraZMax)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'cameraZMax)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'cameraZMax)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'cameraZMax)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'cameraXMax)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'cameraXMax)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'cameraXMax)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'cameraXMax)) ostream)
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'cameraYOffset))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'gridResolution))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'gridCameraZ)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'gridCameraZ)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'gridCameraZ)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'gridCameraZ)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'gridCameraX)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'gridCameraX)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'gridCameraX)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'gridCameraX)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <OccupancyGridHeader>) istream)
  "Deserializes a message object of type '<OccupancyGridHeader>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'cameraZMax)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'cameraZMax)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'cameraZMax)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'cameraZMax)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'cameraXMax)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'cameraXMax)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'cameraXMax)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'cameraXMax)) (cl:read-byte istream))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'cameraYOffset) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'gridResolution) (roslisp-utils:decode-single-float-bits bits)))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'gridCameraZ)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'gridCameraZ)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'gridCameraZ)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'gridCameraZ)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'gridCameraX)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'gridCameraX)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'gridCameraX)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'gridCameraX)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<OccupancyGridHeader>)))
  "Returns string type for a message object of type '<OccupancyGridHeader>"
  "occupancy_grid/OccupancyGridHeader")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'OccupancyGridHeader)))
  "Returns string type for a message object of type 'OccupancyGridHeader"
  "occupancy_grid/OccupancyGridHeader")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<OccupancyGridHeader>)))
  "Returns md5sum for a message object of type '<OccupancyGridHeader>"
  "9f8b35825f257a4716d746612ebebd87")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'OccupancyGridHeader)))
  "Returns md5sum for a message object of type 'OccupancyGridHeader"
  "9f8b35825f257a4716d746612ebebd87")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<OccupancyGridHeader>)))
  "Returns full string definition for message of type '<OccupancyGridHeader>"
  (cl:format cl:nil "#Camera Info~%uint32 cameraZMax~%uint32 cameraXMax~%float32 cameraYOffset~%~%#Occupancy Grid Info~%float32 gridResolution~%uint32 gridCameraZ~%uint32 gridCameraX~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'OccupancyGridHeader)))
  "Returns full string definition for message of type 'OccupancyGridHeader"
  (cl:format cl:nil "#Camera Info~%uint32 cameraZMax~%uint32 cameraXMax~%float32 cameraYOffset~%~%#Occupancy Grid Info~%float32 gridResolution~%uint32 gridCameraZ~%uint32 gridCameraX~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <OccupancyGridHeader>))
  (cl:+ 0
     4
     4
     4
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <OccupancyGridHeader>))
  "Converts a ROS message object to a list"
  (cl:list 'OccupancyGridHeader
    (cl:cons ':cameraZMax (cameraZMax msg))
    (cl:cons ':cameraXMax (cameraXMax msg))
    (cl:cons ':cameraYOffset (cameraYOffset msg))
    (cl:cons ':gridResolution (gridResolution msg))
    (cl:cons ':gridCameraZ (gridCameraZ msg))
    (cl:cons ':gridCameraX (gridCameraX msg))
))
