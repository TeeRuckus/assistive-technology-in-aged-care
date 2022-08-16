
(cl:in-package :asdf)

(defsystem "fallen_analyser-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :std_msgs-msg
)
  :components ((:file "_package")
    (:file "cords" :depends-on ("_package_cords"))
    (:file "_package_cords" :depends-on ("_package"))
  ))