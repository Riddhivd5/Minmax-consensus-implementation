;; Auto-generated. Do not edit!


(when (boundp 'mascot::Force)
  (if (not (find-package "MASCOT"))
    (make-package "MASCOT"))
  (shadow 'Force (find-package "MASCOT")))
(unless (find-package "MASCOT::FORCE")
  (make-package "MASCOT::FORCE"))

(in-package "ROS")
;;//! \htmlinclude Force.msg.html
(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))


(defclass mascot::Force
  :super ros::object
  :slots (_force ))

(defmethod mascot::Force
  (:init
   (&key
    ((:force __force) (instance geometry_msgs::Vector3 :init))
    )
   (send-super :init)
   (setq _force __force)
   self)
  (:force
   (&rest __force)
   (if (keywordp (car __force))
       (send* _force __force)
     (progn
       (if __force (setq _force (car __force)))
       _force)))
  (:serialization-length
   ()
   (+
    ;; geometry_msgs/Vector3 _force
    (send _force :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; geometry_msgs/Vector3 _force
       (send _force :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; geometry_msgs/Vector3 _force
     (send _force :deserialize buf ptr-) (incf ptr- (send _force :serialization-length))
   ;;
   self)
  )

(setf (get mascot::Force :md5sum-) "d2c6d945b64e1a5c23ef82706f0df5c7")
(setf (get mascot::Force :datatype-) "mascot/Force")
(setf (get mascot::Force :definition-)
      "geometry_msgs/Vector3  force
================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space. 
# It is only meant to represent a direction. Therefore, it does not
# make sense to apply a translation to it (e.g., when applying a 
# generic rigid transformation to a Vector3, tf2 will only apply the
# rotation). If you want your data to be translatable too, use the
# geometry_msgs/Point message instead.

float64 x
float64 y
float64 z
")



(provide :mascot/Force "d2c6d945b64e1a5c23ef82706f0df5c7")


