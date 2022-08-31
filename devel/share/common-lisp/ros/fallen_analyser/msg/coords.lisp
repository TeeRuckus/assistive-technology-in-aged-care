; Auto-generated. Do not edit!


(cl:in-package fallen_analyser-msg)


;//! \htmlinclude coords.msg.html

(cl:defclass <coords> (roslisp-msg-protocol:ros-message)
  ((xCord
    :reader xCord
    :initarg :xCord
    :type std_msgs-msg:Float32
    :initform (cl:make-instance 'std_msgs-msg:Float32))
   (yCord
    :reader yCord
    :initarg :yCord
    :type std_msgs-msg:Float32
    :initform (cl:make-instance 'std_msgs-msg:Float32))
   (rightCam
    :reader rightCam
    :initarg :rightCam
    :type std_msgs-msg:Bool
    :initform (cl:make-instance 'std_msgs-msg:Bool)))
)

(cl:defclass coords (<coords>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <coords>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'coords)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name fallen_analyser-msg:<coords> is deprecated: use fallen_analyser-msg:coords instead.")))

(cl:ensure-generic-function 'xCord-val :lambda-list '(m))
(cl:defmethod xCord-val ((m <coords>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader fallen_analyser-msg:xCord-val is deprecated.  Use fallen_analyser-msg:xCord instead.")
  (xCord m))

(cl:ensure-generic-function 'yCord-val :lambda-list '(m))
(cl:defmethod yCord-val ((m <coords>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader fallen_analyser-msg:yCord-val is deprecated.  Use fallen_analyser-msg:yCord instead.")
  (yCord m))

(cl:ensure-generic-function 'rightCam-val :lambda-list '(m))
(cl:defmethod rightCam-val ((m <coords>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader fallen_analyser-msg:rightCam-val is deprecated.  Use fallen_analyser-msg:rightCam instead.")
  (rightCam m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <coords>) ostream)
  "Serializes a message object of type '<coords>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'xCord) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'yCord) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'rightCam) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <coords>) istream)
  "Deserializes a message object of type '<coords>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'xCord) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'yCord) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'rightCam) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<coords>)))
  "Returns string type for a message object of type '<coords>"
  "fallen_analyser/coords")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'coords)))
  "Returns string type for a message object of type 'coords"
  "fallen_analyser/coords")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<coords>)))
  "Returns md5sum for a message object of type '<coords>"
  "06455b0f277279021e59afa6a4c4a32a")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'coords)))
  "Returns md5sum for a message object of type 'coords"
  "06455b0f277279021e59afa6a4c4a32a")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<coords>)))
  "Returns full string definition for message of type '<coords>"
  (cl:format cl:nil "std_msgs/Float32 xCord~%std_msgs/Float32 yCord~%std_msgs/Bool rightCam~%~%================================================================================~%MSG: std_msgs/Float32~%float32 data~%================================================================================~%MSG: std_msgs/Bool~%bool data~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'coords)))
  "Returns full string definition for message of type 'coords"
  (cl:format cl:nil "std_msgs/Float32 xCord~%std_msgs/Float32 yCord~%std_msgs/Bool rightCam~%~%================================================================================~%MSG: std_msgs/Float32~%float32 data~%================================================================================~%MSG: std_msgs/Bool~%bool data~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <coords>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'xCord))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'yCord))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'rightCam))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <coords>))
  "Converts a ROS message object to a list"
  (cl:list 'coords
    (cl:cons ':xCord (xCord msg))
    (cl:cons ':yCord (yCord msg))
    (cl:cons ':rightCam (rightCam msg))
))
