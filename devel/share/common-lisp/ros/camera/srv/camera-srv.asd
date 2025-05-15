
(cl:in-package :asdf)

(defsystem "camera-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "PhotoService" :depends-on ("_package_PhotoService"))
    (:file "_package_PhotoService" :depends-on ("_package"))
    (:file "PhotoboxService" :depends-on ("_package_PhotoboxService"))
    (:file "_package_PhotoboxService" :depends-on ("_package"))
    (:file "PhotoshelfService" :depends-on ("_package_PhotoshelfService"))
    (:file "_package_PhotoshelfService" :depends-on ("_package"))
  ))