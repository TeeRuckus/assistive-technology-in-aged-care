;; Auto-generated. Do not edit!


(when (boundp 'fallen_analyser::coords)
  (if (not (find-package "FALLEN_ANALYSER"))
    (make-package "FALLEN_ANALYSER"))
  (shadow 'coords (find-package "FALLEN_ANALYSER")))
(unless (find-package "FALLEN_ANALYSER::COORDS")
  (make-package "FALLEN_ANALYSER::COORDS"))

(in-package "ROS")
;;//! \htmlinclude coords.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass fallen_analyser::coords
  :super ros::object
  :slots (_xCord _yCord _rightCam ))

(defmethod fallen_analyser::coords
  (:init
   (&key
    ((:xCord __xCord) (instance std_msgs::Float32 :init))
    ((:yCord __yCord) (instance std_msgs::Float32 :init))
    ((:rightCam __rightCam) (instance std_msgs::Bool :init))
    )
   (send-super :init)
   (setq _xCord __xCord)
   (setq _yCord __yCord)
   (setq _rightCam __rightCam)
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
  (:rightCam
   (&rest __rightCam)
   (if (keywordp (car __rightCam))
       (send* _rightCam __rightCam)
     (progn
       (if __rightCam (setq _rightCam (car __rightCam)))
       _rightCam)))
  (:serialization-length
   ()
   (+
    ;; std_msgs/Float32 _xCord
    (send _xCord :serialization-length)
    ;; std_msgs/Float32 _yCord
    (send _yCord :serialization-length)
    ;; std_msgs/Bool _rightCam
    (send _rightCam :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Float32 _xCord
       (send _xCord :serialize s)
     ;; std_msgs/Float32 _yCord
       (send _yCord :serialize s)
     ;; std_msgs/Bool _rightCam
       (send _rightCam :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Float32 _xCord
     (send _xCord :deserialize buf ptr-) (incf ptr- (send _xCord :serialization-length))
   ;; std_msgs/Float32 _yCord
     (send _yCord :deserialize buf ptr-) (incf ptr- (send _yCord :serialization-length))
   ;; std_msgs/Bool _rightCam
     (send _rightCam :deserialize buf ptr-) (incf ptr- (send _rightCam :serialization-length))
   ;;
   self)
  )

(setf (get fallen_analyser::coords :md5sum-) "06455b0f277279021e59afa6a4c4a32a")
(setf (get fallen_analyser::coords :datatype-) "fallen_analyser/coords")
(setf (get fallen_analyser::coords :definition-)
      "std_msgs/Float32 xCord
std_msgs/Float32 yCord
std_msgs/Bool rightCam

================================================================================
MSG: std_msgs/Float32
float32 data
================================================================================
MSG: std_msgs/Bool
bool data
")



(provide :fallen_analyser/coords "06455b0f277279021e59afa6a4c4a32a")


