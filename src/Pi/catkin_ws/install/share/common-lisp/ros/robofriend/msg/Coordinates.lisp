; Auto-generated. Do not edit!


(cl:in-package robofriend-msg)


;//! \htmlinclude Coordinates.msg.html

(cl:defclass <Coordinates> (roslisp-msg-protocol:ros-message)
  ((y_top
    :reader y_top
    :initarg :y_top
    :type cl:fixnum
    :initform 0)
   (right
    :reader right
    :initarg :right
    :type cl:fixnum
    :initform 0)
   (bottom
    :reader bottom
    :initarg :bottom
    :type cl:fixnum
    :initform 0)
   (x_left
    :reader x_left
    :initarg :x_left
    :type cl:fixnum
    :initform 0)
   (face_name
    :reader face_name
    :initarg :face_name
    :type cl:string
    :initform ""))
)

(cl:defclass Coordinates (<Coordinates>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Coordinates>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Coordinates)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name robofriend-msg:<Coordinates> is deprecated: use robofriend-msg:Coordinates instead.")))

(cl:ensure-generic-function 'y_top-val :lambda-list '(m))
(cl:defmethod y_top-val ((m <Coordinates>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader robofriend-msg:y_top-val is deprecated.  Use robofriend-msg:y_top instead.")
  (y_top m))

(cl:ensure-generic-function 'right-val :lambda-list '(m))
(cl:defmethod right-val ((m <Coordinates>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader robofriend-msg:right-val is deprecated.  Use robofriend-msg:right instead.")
  (right m))

(cl:ensure-generic-function 'bottom-val :lambda-list '(m))
(cl:defmethod bottom-val ((m <Coordinates>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader robofriend-msg:bottom-val is deprecated.  Use robofriend-msg:bottom instead.")
  (bottom m))

(cl:ensure-generic-function 'x_left-val :lambda-list '(m))
(cl:defmethod x_left-val ((m <Coordinates>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader robofriend-msg:x_left-val is deprecated.  Use robofriend-msg:x_left instead.")
  (x_left m))

(cl:ensure-generic-function 'face_name-val :lambda-list '(m))
(cl:defmethod face_name-val ((m <Coordinates>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader robofriend-msg:face_name-val is deprecated.  Use robofriend-msg:face_name instead.")
  (face_name m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Coordinates>) ostream)
  "Serializes a message object of type '<Coordinates>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'y_top)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'y_top)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'right)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'right)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'bottom)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'bottom)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'x_left)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'x_left)) ostream)
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'face_name))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'face_name))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Coordinates>) istream)
  "Deserializes a message object of type '<Coordinates>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'y_top)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'y_top)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'right)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'right)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'bottom)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'bottom)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'x_left)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'x_left)) (cl:read-byte istream))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'face_name) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'face_name) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Coordinates>)))
  "Returns string type for a message object of type '<Coordinates>"
  "robofriend/Coordinates")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Coordinates)))
  "Returns string type for a message object of type 'Coordinates"
  "robofriend/Coordinates")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Coordinates>)))
  "Returns md5sum for a message object of type '<Coordinates>"
  "be7c1e838538f5e60d0dc085dffee546")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Coordinates)))
  "Returns md5sum for a message object of type 'Coordinates"
  "be7c1e838538f5e60d0dc085dffee546")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Coordinates>)))
  "Returns full string definition for message of type '<Coordinates>"
  (cl:format cl:nil "uint16 y_top~%uint16 right~%uint16 bottom~%uint16 x_left~%string face_name~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Coordinates)))
  "Returns full string definition for message of type 'Coordinates"
  (cl:format cl:nil "uint16 y_top~%uint16 right~%uint16 bottom~%uint16 x_left~%string face_name~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Coordinates>))
  (cl:+ 0
     2
     2
     2
     2
     4 (cl:length (cl:slot-value msg 'face_name))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Coordinates>))
  "Converts a ROS message object to a list"
  (cl:list 'Coordinates
    (cl:cons ':y_top (y_top msg))
    (cl:cons ':right (right msg))
    (cl:cons ':bottom (bottom msg))
    (cl:cons ':x_left (x_left msg))
    (cl:cons ':face_name (face_name msg))
))
