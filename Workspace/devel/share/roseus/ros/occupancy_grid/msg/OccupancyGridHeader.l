;; Auto-generated. Do not edit!


(when (boundp 'occupancy_grid::OccupancyGridHeader)
  (if (not (find-package "OCCUPANCY_GRID"))
    (make-package "OCCUPANCY_GRID"))
  (shadow 'OccupancyGridHeader (find-package "OCCUPANCY_GRID")))
(unless (find-package "OCCUPANCY_GRID::OCCUPANCYGRIDHEADER")
  (make-package "OCCUPANCY_GRID::OCCUPANCYGRIDHEADER"))

(in-package "ROS")
;;//! \htmlinclude OccupancyGridHeader.msg.html


(defclass occupancy_grid::OccupancyGridHeader
  :super ros::object
  :slots (_cameraZMax _cameraXMax _cameraYOffset _gridResolution _gridCameraZ _gridCameraX ))

(defmethod occupancy_grid::OccupancyGridHeader
  (:init
   (&key
    ((:cameraZMax __cameraZMax) 0)
    ((:cameraXMax __cameraXMax) 0)
    ((:cameraYOffset __cameraYOffset) 0.0)
    ((:gridResolution __gridResolution) 0.0)
    ((:gridCameraZ __gridCameraZ) 0)
    ((:gridCameraX __gridCameraX) 0)
    )
   (send-super :init)
   (setq _cameraZMax (round __cameraZMax))
   (setq _cameraXMax (round __cameraXMax))
   (setq _cameraYOffset (float __cameraYOffset))
   (setq _gridResolution (float __gridResolution))
   (setq _gridCameraZ (round __gridCameraZ))
   (setq _gridCameraX (round __gridCameraX))
   self)
  (:cameraZMax
   (&optional __cameraZMax)
   (if __cameraZMax (setq _cameraZMax __cameraZMax)) _cameraZMax)
  (:cameraXMax
   (&optional __cameraXMax)
   (if __cameraXMax (setq _cameraXMax __cameraXMax)) _cameraXMax)
  (:cameraYOffset
   (&optional __cameraYOffset)
   (if __cameraYOffset (setq _cameraYOffset __cameraYOffset)) _cameraYOffset)
  (:gridResolution
   (&optional __gridResolution)
   (if __gridResolution (setq _gridResolution __gridResolution)) _gridResolution)
  (:gridCameraZ
   (&optional __gridCameraZ)
   (if __gridCameraZ (setq _gridCameraZ __gridCameraZ)) _gridCameraZ)
  (:gridCameraX
   (&optional __gridCameraX)
   (if __gridCameraX (setq _gridCameraX __gridCameraX)) _gridCameraX)
  (:serialization-length
   ()
   (+
    ;; uint32 _cameraZMax
    4
    ;; uint32 _cameraXMax
    4
    ;; float32 _cameraYOffset
    4
    ;; float32 _gridResolution
    4
    ;; uint32 _gridCameraZ
    4
    ;; uint32 _gridCameraX
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint32 _cameraZMax
       (write-long _cameraZMax s)
     ;; uint32 _cameraXMax
       (write-long _cameraXMax s)
     ;; float32 _cameraYOffset
       (sys::poke _cameraYOffset (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _gridResolution
       (sys::poke _gridResolution (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; uint32 _gridCameraZ
       (write-long _gridCameraZ s)
     ;; uint32 _gridCameraX
       (write-long _gridCameraX s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint32 _cameraZMax
     (setq _cameraZMax (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; uint32 _cameraXMax
     (setq _cameraXMax (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; float32 _cameraYOffset
     (setq _cameraYOffset (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _gridResolution
     (setq _gridResolution (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; uint32 _gridCameraZ
     (setq _gridCameraZ (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; uint32 _gridCameraX
     (setq _gridCameraX (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get occupancy_grid::OccupancyGridHeader :md5sum-) "9f8b35825f257a4716d746612ebebd87")
(setf (get occupancy_grid::OccupancyGridHeader :datatype-) "occupancy_grid/OccupancyGridHeader")
(setf (get occupancy_grid::OccupancyGridHeader :definition-)
      "#Camera Info
uint32 cameraZMax
uint32 cameraXMax
float32 cameraYOffset

#Occupancy Grid Info
float32 gridResolution
uint32 gridCameraZ
uint32 gridCameraX


")



(provide :occupancy_grid/OccupancyGridHeader "9f8b35825f257a4716d746612ebebd87")


