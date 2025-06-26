
(cl:in-package :asdf)

(defsystem "mascot-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg
)
  :components ((:file "_package")
    (:file "Force" :depends-on ("_package_Force"))
    (:file "_package_Force" :depends-on ("_package"))
    (:file "Gain" :depends-on ("_package_Gain"))
    (:file "_package_Gain" :depends-on ("_package"))
  ))