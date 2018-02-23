; Auto-generated. Do not edit!


(cl:in-package occupancy_grid-msg)


;//! \htmlinclude OccupancyGrid.msg.html

(cl:defclass <OccupancyGrid> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type occupancy_grid-msg:OccupancyGridHeader
    :initform (cl:make-instance 'occupancy_grid-msg:OccupancyGridHeader))
   (dataDimension
    :reader dataDimension
    :initarg :dataDimension
    :type (cl:vector occupancy_grid-msg:GridDataDimension)
   :initform (cl:make-array 0 :element-type 'occupancy_grid-msg:GridDataDimension :initial-element (cl:make-instance 'occupancy_grid-msg:GridDataDimension)))
   (data
    :reader data
    :initarg :data
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0)))
)

(cl:defclass OccupancyGrid (<OccupancyGrid>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <OccupancyGrid>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'OccupancyGrid)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name occupancy_grid-msg:<OccupancyGrid> is deprecated: use occupancy_grid-msg:OccupancyGrid instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <OccupancyGrid>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader occupancy_grid-msg:header-val is deprecated.  Use occupancy_grid-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'dataDimension-val :lambda-list '(m))
(cl:defmethod dataDimension-val ((m <OccupancyGrid>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader occupancy_grid-msg:dataDimension-val is deprecated.  Use occupancy_grid-msg:dataDimension instead.")
  (dataDimension m))

(cl:ensure-generic-function 'data-val :lambda-list '(m))
(cl:defmethod data-val ((m <OccupancyGrid>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader occupancy_grid-msg:data-val is deprecated.  Use occupancy_grid-msg:data instead.")
  (data m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <OccupancyGrid>) ostream)
  "Serializes a message object of type '<OccupancyGrid>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'dataDimension))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'dataDimension))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'data))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)))
   (cl:slot-value msg 'data))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <OccupancyGrid>) istream)
  "Deserializes a message object of type '<OccupancyGrid>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'dataDimension) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'dataDimension)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'occupancy_grid-msg:GridDataDimension))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'data) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'data)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-single-float-bits bits))))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<OccupancyGrid>)))
  "Returns string type for a message object of type '<OccupancyGrid>"
  "occupancy_grid/OccupancyGrid")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'OccupancyGrid)))
  "Returns string type for a message object of type 'OccupancyGrid"
  "occupancy_grid/OccupancyGrid")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<OccupancyGrid>)))
  "Returns md5sum for a message object of type '<OccupancyGrid>"
  "ce7444edcba848b954358db865f7fab7")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'OccupancyGrid)))
  "Returns md5sum for a message object of type 'OccupancyGrid"
  "ce7444edcba848b954358db865f7fab7")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<OccupancyGrid>)))
  "Returns full string definition for message of type '<OccupancyGrid>"
  (cl:format cl:nil "OccupancyGridHeader header~%~%GridDataDimension[] dataDimension~%float32[] data~%~%================================================================================~%MSG: occupancy_grid/OccupancyGridHeader~%#Camera Info~%uint32 cameraZMax~%uint32 cameraXMax~%float32 cameraYOffset~%~%#Occupancy Grid Info~%float32 gridResolution~%uint32 gridCameraZ~%uint32 gridCameraX~%~%~%================================================================================~%MSG: occupancy_grid/GridDataDimension~%string label~%uint32 size~%uint32 stride~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'OccupancyGrid)))
  "Returns full string definition for message of type 'OccupancyGrid"
  (cl:format cl:nil "OccupancyGridHeader header~%~%GridDataDimension[] dataDimension~%float32[] data~%~%================================================================================~%MSG: occupancy_grid/OccupancyGridHeader~%#Camera Info~%uint32 cameraZMax~%uint32 cameraXMax~%float32 cameraYOffset~%~%#Occupancy Grid Info~%float32 gridResolution~%uint32 gridCameraZ~%uint32 gridCameraX~%~%~%================================================================================~%MSG: occupancy_grid/GridDataDimension~%string label~%uint32 size~%uint32 stride~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <OccupancyGrid>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'dataDimension) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'data) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <OccupancyGrid>))
  "Converts a ROS message object to a list"
  (cl:list 'OccupancyGrid
    (cl:cons ':header (header msg))
    (cl:cons ':dataDimension (dataDimension msg))
    (cl:cons ':data (data msg))
))
