;; Auto-generated. Do not edit!


(when (boundp 'fallen_analyser::cords)
  (if (not (find-package "FALLEN_ANALYSER"))
    (make-package "FALLEN_ANALYSER"))
  (shadow 'cords (find-package "FALLEN_ANALYSER")))
(unless (find-package "FALLEN_ANALYSER::CORDS")
  (make-package "FALLEN_ANALYSER::CORDS"))

(in-package "ROS")
;;//! \htmlinclude cords.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass fallen_analyser::cords
  :super ros::object
  :slots (_xCord _yCord ))

(defmethod fallen_analyser::cords
  (:init
   (&key
    ((:xCord __xCord) (instance std_msgs::Float32 :init))
    ((:yCord __yCord) (instance std_msgs::Float32 :init))
    )
   (send-super :init)
   (setq _xCord __xCord)
   (setq _yCord __yCord)
   self)
  (:xCord
   (&rest __xCord)
   (if (keywordp (car __xCord))
       (send* _xCord __xCord)
     (progn
       (if __xCord (setq _xCord (car __xCord)))
       _xCord)))
  (:yCord
   (&rest __yCord)
   (if (keywordp (car __yCord))
       (send* _yCord __yCord)
     (progn
       (if __yCord (setq _yCord (car __yCord)))
       _yCord)))
  (:serialization-length
   ()
   (+
    ;; std_msgs/Float32 _xCord
    (send _xCord :serialization-length)
    ;; std_msgs/Float32 _yCord
    (send _yCord :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Float32 _xCord
       (send _xCord :serialize s)
     ;; std_msgs/Float32 _yCord
       (send _yCord :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Float32 _xCord
     (send _xCord :deserialize buf ptr-) (incf ptr- (send _xCord :serialization-length))
   ;; std_msgs/Float32 _yCord
     (send _yCord :deserialize buf ptr-) (incf ptr- (send _yCord :serialization-length))
   ;;
   self)
  )

(setf (get fallen_analyser::cords :md5sum-) "6466c8bc21dcd864939639eb1c9ad609")
(setf (get fallen_analyser::cords :datatype-) "fallen_analyser/cords")
(setf (get fallen_analyser::cords :definition-)
      "std_msgs/Float32 xCord
std_msgs/Float32 yCord

================================================================================
MSG: std_msgs/Float32
float32 data
")



(provide :fallen_analyser/cords "6466c8bc21dcd864939639eb1c9ad609")


