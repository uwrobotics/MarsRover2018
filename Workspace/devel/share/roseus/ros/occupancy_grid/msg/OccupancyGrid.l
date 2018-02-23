;; Auto-generated. Do not edit!


(when (boundp 'occupancy_grid::OccupancyGrid)
  (if (not (find-package "OCCUPANCY_GRID"))
    (make-package "OCCUPANCY_GRID"))
  (shadow 'OccupancyGrid (find-package "OCCUPANCY_GRID")))
(unless (find-package "OCCUPANCY_GRID::OCCUPANCYGRID")
  (make-package "OCCUPANCY_GRID::OCCUPANCYGRID"))

(in-package "ROS")
;;//! \htmlinclude OccupancyGrid.msg.html


(defclass occupancy_grid::OccupancyGrid
  :super ros::object
  :slots (_header _dataDimension _data ))

(defmethod occupancy_grid::OccupancyGrid
  (:init
   (&key
    ((:header __header) (instance occupancy_grid::OccupancyGridHeader :init))
    ((:dataDimension __dataDimension) (let (r) (dotimes (i 0) (push (instance occupancy_grid::GridDataDimension :init) r)) r))
    ((:data __data) (make-array 0 :initial-element 0.0 :element-type :float))
    )
   (send-super :init)
   (setq _header __header)
   (setq _dataDimension __dataDimension)
   (setq _data __data)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:dataDimension
   (&rest __dataDimension)
   (if (keywordp (car __dataDimension))
       (send* _dataDimension __dataDimension)
     (progn
       (if __dataDimension (setq _dataDimension (car __dataDimension)))
       _dataDimension)))
  (:data
   (&optional __data)
   (if __data (setq _data __data)) _data)
  (:serialization-length
   ()
   (+
    ;; occupancy_grid/OccupancyGridHeader _header
    (send _header :serialization-length)
    ;; occupancy_grid/GridDataDimension[] _dataDimension
    (apply #'+ (send-all _dataDimension :serialization-length)) 4
    ;; float32[] _data
    (* 4    (length _data)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; occupancy_grid/OccupancyGridHeader _header
       (send _header :serialize s)
     ;; occupancy_grid/GridDataDimension[] _dataDimension
     (write-long (length _dataDimension) s)
     (dolist (elem _dataDimension)
       (send elem :serialize s)
       )
     ;; float32[] _data
     (write-long (length _data) s)
     (dotimes (i (length _data))
       (sys::poke (elt _data i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; occupancy_grid/OccupancyGridHeader _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; occupancy_grid/GridDataDimension[] _dataDimension
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _dataDimension (let (r) (dotimes (i n) (push (instance occupancy_grid::GridDataDimension :init) r)) r))
     (dolist (elem- _dataDimension)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;; float32[] _data
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _data (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _data i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     ))
   ;;
   self)
  )

(setf (get occupancy_grid::OccupancyGrid :md5sum-) "ce7444edcba848b954358db865f7fab7")
(setf (get occupancy_grid::OccupancyGrid :datatype-) "occupancy_grid/OccupancyGrid")
(setf (get occupancy_grid::OccupancyGrid :definition-)
      "OccupancyGridHeader header

GridDataDimension[] dataDimension
float32[] data

================================================================================
MSG: occupancy_grid/OccupancyGridHeader
#Camera Info
uint32 cameraZMax
uint32 cameraXMax
float32 cameraYOffset

#Occupancy Grid Info
float32 gridResolution
uint32 gridCameraZ
uint32 gridCameraX


================================================================================
MSG: occupancy_grid/GridDataDimension
string label
uint32 size
uint32 stride

")



(provide :occupancy_grid/OccupancyGrid "ce7444edcba848b954358db865f7fab7")


