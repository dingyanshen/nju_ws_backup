
(cl:in-package :asdf)

(defsystem "motion_control-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "getDiffAngle" :depends-on ("_package_getDiffAngle"))
    (:file "_package_getDiffAngle" :depends-on ("_package"))
    (:file "getFrontDist" :depends-on ("_package_getFrontDist"))
    (:file "_package_getFrontDist" :depends-on ("_package"))
    (:file "getLeftDist" :depends-on ("_package_getLeftDist"))
    (:file "_package_getLeftDist" :depends-on ("_package"))
    (:file "getLeftFrontDist" :depends-on ("_package_getLeftFrontDist"))
    (:file "_package_getLeftFrontDist" :depends-on ("_package"))
    (:file "getRightDist" :depends-on ("_package_getRightDist"))
    (:file "_package_getRightDist" :depends-on ("_package"))
    (:file "getRightFrontDist" :depends-on ("_package_getRightFrontDist"))
    (:file "_package_getRightFrontDist" :depends-on ("_package"))
  ))