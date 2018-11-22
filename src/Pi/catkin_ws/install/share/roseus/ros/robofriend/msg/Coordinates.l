;; Auto-generated. Do not edit!


(when (boundp 'robofriend::Coordinates)
  (if (not (find-package "ROBOFRIEND"))
    (make-package "ROBOFRIEND"))
  (shadow 'Coordinates (find-package "ROBOFRIEND")))
(unless (find-package "ROBOFRIEND::COORDINATES")
  (make-package "ROBOFRIEND::COORDINATES"))

(in-package "ROS")
;;//! \htmlinclude Coordinates.msg.html


(defclass robofriend::Coordinates
  :super ros::object
  :slots (_y_top _right _bottom _x_left _face_name ))

(defmethod robofriend::Coordinates
  (:init
   (&key
    ((:y_top __y_top) 0)
    ((:right __right) 0)
    ((:bottom __bottom) 0)
    ((:x_left __x_left) 0)
    ((:face_name __face_name) "")
    )
   (send-super :init)
   (setq _y_top (round __y_top))
   (setq _right (round __right))
   (setq _bottom (round __bottom))
   (setq _x_left (round __x_left))
   (setq _face_name (string __face_name))
   self)
  (:y_top
   (&optional __y_top)
   (if __y_top (setq _y_top __y_top)) _y_top)
  (:right
   (&optional __right)
   (if __right (setq _right __right)) _right)
  (:bottom
   (&optional __bottom)
   (if __bottom (setq _bottom __bottom)) _bottom)
  (:x_left
   (&optional __x_left)
   (if __x_left (setq _x_left __x_left)) _x_left)
  (:face_name
   (&optional __face_name)
   (if __face_name (setq _face_name __face_name)) _face_name)
  (:serialization-length
   ()
   (+
    ;; uint16 _y_top
    2
    ;; uint16 _right
    2
    ;; uint16 _bottom
    2
    ;; uint16 _x_left
    2
    ;; string _face_name
    4 (length _face_name)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint16 _y_top
       (write-word _y_top s)
     ;; uint16 _right
       (write-word _right s)
     ;; uint16 _bottom
       (write-word _bottom s)
     ;; uint16 _x_left
       (write-word _x_left s)
     ;; string _face_name
       (write-long (length _face_name) s) (princ _face_name s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint16 _y_top
     (setq _y_top (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; uint16 _right
     (setq _right (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; uint16 _bottom
     (setq _bottom (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; uint16 _x_left
     (setq _x_left (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; string _face_name
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _face_name (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(setf (get robofriend::Coordinates :md5sum-) "be7c1e838538f5e60d0dc085dffee546")
(setf (get robofriend::Coordinates :datatype-) "robofriend/Coordinates")
(setf (get robofriend::Coordinates :definition-)
      "uint16 y_top
uint16 right
uint16 bottom
uint16 x_left
string face_name

")



(provide :robofriend/Coordinates "be7c1e838538f5e60d0dc085dffee546")


