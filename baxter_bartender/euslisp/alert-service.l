#!/usr/bin/env roseus

(ros::load-ros-manifest "simple_service")
(ros::roseus "alert_service")

(require "euslib/demo/s-noda/util/button-sample.l")

(defvar *user-command* nil)
(defvar *button-panel* nil)

(defun click-callback
  (obj &rest args)
  (setq *user-command* (send obj :name))
  (cond
   (*button-panel*
    (send *button-panel* :quit)
    (setq *button-panel* nil))))

(defun service-cb (msg)
  (let* ((res (send msg :response))
	 (req (send msg :req))
	 (name-list (if (plusp (length req)) (read-from-string req)))
	 (time-step 100)
	 (time-out 5000))
    (cond
     ((null name-list)
      (format t "[alert-service] invalid serivec call ~A~%" req))
     (t
      (if (atom name-list) (setq name-list (list name-list)))
      (format t "[alert-service] ~A -> ~A~%" req name-list)
      (setq *user-command* nil)
      (setq *button-panel*
	    (instance button-sample :create
		      :window-name "Alert"
		      :name-list
		      (mapcar
		       #'(lambda (k) (format nil "~A" k))
		       name-list)
		      :w 102 :h 102))
      ;;(send *button-panel* :redraw)
      ;;(send *button-panel* :flush)
      (place-alert-center)
      (while (and (null *user-command*) (plusp time-out))
	(x::window-main-one)
	(setq time-out (- time-out time-step))
	(unix:usleep (* time-step 1000)))
      (cond
       ((not (plusp time-out))
	(format t "[alert-service] time-out~%")))
      (cond
       (*button-panel*
	(send *button-panel* :quit)
	(setq *button-panel* nil)
	(setq *user-command* :time-out)))
      (send res :res (format nil "~A" *user-command*))))
    res))

(defun call-alert-service
  (key-list)
  (let* ((res
	  (ros::service-call
	   "/alert_service"
	   (instance simple_service::stringRequest :init :req
		     (format nil "~A" key-list))))
	 (res-str (send res :res)))
    (if (plusp (length res-str)) (read-from-string res-str))))

(defun place-alert-center
  (&key
   (gui *button-panel*)
   (size (list (send gui :width)
               (send gui :height)))
   (pos (list
	 (/ (- (send x::*root* :width) (car size)) 2)
	 (/ (- (send x::*root* :height) (cadr size)) 2)))
   )
  (send gui :move (nth 0 pos) (nth 1 pos))
  (send gui :redraw)
  (send gui :flush))


(defvar *alert_service_context* "__main__")
(cond
 ((string-equal *alert_service_context* "__main__")
  (print 'start-alert-service)
  (ros::advertise-service "/alert_service"
			  simple_service::string #'service-cb)
  (ros::rate 10)
  (do-until-key
   (if (not (ros::ok)) (return-from nil nil))
   (unless (or (null x::*display*) (= x::*display* 0))
     (x::window-main-one))
   (ros::sleep)
   (ros::spin-once))
  ))

#|

