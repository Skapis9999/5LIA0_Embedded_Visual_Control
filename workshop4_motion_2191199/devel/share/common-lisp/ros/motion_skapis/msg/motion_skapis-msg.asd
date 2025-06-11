
(cl:in-package :asdf)

(defsystem "motion_skapis-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :sensor_msgs-msg
)
  :components ((:file "_package")
    (:file "dualImage" :depends-on ("_package_dualImage"))
    (:file "_package_dualImage" :depends-on ("_package"))
  ))