
(cl:in-package :asdf)

(defsystem "msg_check-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg
               :std_msgs-msg
)
  :components ((:file "_package")
    (:file "BoxMsg" :depends-on ("_package_BoxMsg"))
    (:file "_package_BoxMsg" :depends-on ("_package"))
    (:file "PlotDataMsg" :depends-on ("_package_PlotDataMsg"))
    (:file "_package_PlotDataMsg" :depends-on ("_package"))
    (:file "xyz" :depends-on ("_package_xyz"))
    (:file "_package_xyz" :depends-on ("_package"))
  ))