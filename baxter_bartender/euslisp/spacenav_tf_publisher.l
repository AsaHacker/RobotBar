#!/usr/bin/env roseus

(ros::roseus "spacenav_tf_publisher")
(ros::roseus-add-msgs "sensor_msgs")


(defvar *tfb* (instance ros::transform-broadcaster :init))

(defvar *controll-coords* (make-coords))
(if (probe-file "camera-coords.l") (require "camera-coords.l"))

(defun spacenav-callback
  (msg)
  ;; (format t "~A~%" (send msg :axes))
  (let* ((bs (send msg :buttons))
         (rotscl 1e-2)
         (posscl 10.0))
    (cond
     ((> (norm bs) 1e-3)
      (unix:system
       (format nil
	       (concatenate
		string
		"echo \"(setq *controll-coords* "
		"(make-coords :pos ~A :rot ~A))\""
		"> camera-coords.l")
	       (send *controll-coords* :worldpos)
	       (send *controll-coords* :worldrot))))
     ((< (norm (send msg :axes)) 1e-3))
     (t
      (setq *controll-coords*
	    (make-coords
	     :pos
	     (v+ (send *controll-coords* :worldpos)
		 (scale posscl (subseq (send msg :axes) 0 3)))
	     :rot
	     (send
	      (send (make-coords
		     :rot (copy-object (send *controll-coords* :worldrot)))
		    :transform
		    (make-coords
		     :rpy (scale rotscl (reverse (subseq (send msg :axes) 3 6))))
		    :world)
	      :worldrot))))
     )))

(ros::subscribe
 "/spacenav/joy"
 sensor_msgs::Joy #'spacenav-callback)

(defun publish-tf
  (&key
   (coords (make-coords))
   (from "BASE")
   (to "camera_link"))
  (format t "[tf_tranfrom] ~A~%"
	  (concatenate
	   float-vector
	   (scale 1e-3 (send coords :worldpos))
	   (let ((v (matrix2quaternion (send coords :worldrot))))
	     (concatenate float-vector (subseq v 3 4) (subseq v 0 3)))
	   ))
  (send *tfb* :send-transform coords from to))

(do-until-key
 (ros::spin-once)
 ;; (x::window-main-one)
 (ros::sleep)
 (publish-tf :coords *controll-coords*
	     :from "torso"
	     :to"camera_link")
 (unless (ros::ok) (return)))
