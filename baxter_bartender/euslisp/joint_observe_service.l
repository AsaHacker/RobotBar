#!/usr/bin/env roseus

(ros::load-ros-manifest "simple_service")
(ros::roseus "joint_observe_service")

(defun service-cb (msg)
  (let* ((res (send msg :response)))
    (print (send msg :req))
    (cond
     ((substringp ":wait-interpolation" (send msg :req))
      (send res :res (format nil "~A" (wait-interpolation))))
     ((substringp ":check" (send msg :req))
      (send res :res (format nil "~A"  (move-and-collision-check)))))
    res))

(defvar *move?* nil)
(defun move-callback (msg) (setq *move?* t))
(defvar *collision?* nil)
(defun collision-callback (msg) (setq *collision?* t))

(defun move-and-collision-check
  (&key (time-step 0.1) ;; sec
	(max-time 0.51))
  (setq *move?* nil)
  (setq *collision?* nil)
  (while (plusp (setq max-time
		      (- max-time time-step)))
    (ros::spin-once)
    (unix:usleep (round (* time-step 1e+6))))
  (list *move?* *collision?*))

(defun wait-interpolation
  (&key (max-time 10) (step 0.5) ret)
  (do-until-key
   (move-and-collision-check :time-step 0.1 :max-time (+ step 0.01))
   (if (minusp (+ (setq max-time (- max-time step)) 0.01))
       (return-from nil nil))
   (if (null *move?*) (return-from nil (setq ret t))))
  ret)

(defun call-joint-observe-service
  (key)
  (ros::service-call
   "/joint_observe_service"
   (instance simple_service::stringRequest :init :req
	     (format nil "~A" key))))

(defvar *joint_observe_service_context* "__main__")
(cond
 ((string-equal *joint_observe_service_context* "__main__")
  (print 'start-joint-observer)
  (ros::subscribe
   "/baxter_joint_observer/move"
   std_msgs::empty
   #'move-callback)
  (ros::subscribe
   "/baxter_joint_observer/collision"
   std_msgs::empty
   #'collision-callback)
  (ros::advertise-service "/joint_observe_service"
			  simple_service::string #'service-cb)
  (ros::rate 10)
  (do-until-key
   (if (not (ros::ok)) (return-from nil nil))
   (unless (or (null x::*display*) (= x::*display* 0))
     (x::window-main-one))
   (ros::sleep)
   (ros::spin-once))
  ))
