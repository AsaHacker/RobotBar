#!/usr/bin/env roseus

(ros::load-ros-manifest "simple_service")
(require "baxter-util.l")

(ros::roseus "table_touch_it")

(defun service-cb (msg)
  (let* ((res (send msg :response)))
    (send res :res
	  (format nil "~A"
		  (table-reaching-until-collision
		   (read-from-string (send msg :req)))))
    res))

(defun table-reaching-until-collision
  (arm
   &key
   (max-height 151)
   (step-height 30)
   (ret nil))
  (real2model)
  (do-until-key
   (if (minusp
	(setq max-height (- max-height step-height)))
       (return-from nil nil))
   (if (null
	(ufo-ik
	 arm
	 (make-coords
	  :pos
	  (float-vector 0 0 (* -1 step-height)))))
       (return-from nil (print 'ik-unresolve)))
   (model2real :sleep 1000 :wait t)
   (unix:usleep (* 500 1000))
   (if (print (flatten (move-and-collision-check :max-time 0.51)))
       (return-from nil (print (setq ret 'collision-detected)))))
  ret)

(defun call-table-touch-it-server
  (key)
  (ros::service-call
   "/table_touch_it"
   (instance simple_service::stringRequest :init :req (format nil "~A" key))))

(defvar *touch-it-context* "__main__")
(cond
 ((string-equal *touch-it-context* "__main__")
  (print 'start-touch-it-server)
  (ros::advertise-service "/table_touch_it" simple_service::string #'service-cb)
  (ros::rate 10)
  (do-until-key
   (if (not (ros::ok)) (return-from nil nil))
   (unless (or (null x::*display*) (= x::*display* 0))
     (x::window-main-one))
   (ros::sleep)
   (ros::spin-once))
  ))

