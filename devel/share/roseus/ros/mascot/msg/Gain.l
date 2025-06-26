;; Auto-generated. Do not edit!


(when (boundp 'mascot::Gain)
  (if (not (find-package "MASCOT"))
    (make-package "MASCOT"))
  (shadow 'Gain (find-package "MASCOT")))
(unless (find-package "MASCOT::GAIN")
  (make-package "MASCOT::GAIN"))

(in-package "ROS")
;;//! \htmlinclude Gain.msg.html


(defclass mascot::Gain
  :super ros::object
  :slots (_k_p _k_d _k_i ))

(defmethod mascot::Gain
  (:init
   (&key
    ((:k_p __k_p) 0.0)
    ((:k_d __k_d) 0.0)
    ((:k_i __k_i) 0.0)
    )
   (send-super :init)
   (setq _k_p (float __k_p))
   (setq _k_d (float __k_d))
   (setq _k_i (float __k_i))
   self)
  (:k_p
   (&optional __k_p)
   (if __k_p (setq _k_p __k_p)) _k_p)
  (:k_d
   (&optional __k_d)
   (if __k_d (setq _k_d __k_d)) _k_d)
  (:k_i
   (&optional __k_i)
   (if __k_i (setq _k_i __k_i)) _k_i)
  (:serialization-length
   ()
   (+
    ;; float32 _k_p
    4
    ;; float32 _k_d
    4
    ;; float32 _k_i
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float32 _k_p
       (sys::poke _k_p (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _k_d
       (sys::poke _k_d (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _k_i
       (sys::poke _k_i (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float32 _k_p
     (setq _k_p (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _k_d
     (setq _k_d (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _k_i
     (setq _k_i (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get mascot::Gain :md5sum-) "42218fccfcf87f05cc02314f682ea6ae")
(setf (get mascot::Gain :datatype-) "mascot/Gain")
(setf (get mascot::Gain :definition-)
      "float32 k_p
float32 k_d
float32 k_i

")



(provide :mascot/Gain "42218fccfcf87f05cc02314f682ea6ae")


