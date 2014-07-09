#!/usr/bin/env roseus

(ros::roseus-add-msgs "sensor_msgs")
(ros::roseus "spacenav_controller")

(defvar *target-limb* :larm)

(defun publish-posestamped
  (coords
   &key
   (pos (send coords :worldpos))
   (rot (send coords :worldrot))
   (q (matrix2quaternion rot))
   )
  (format t "[publish-posestamped] pos=~A/q=~A~%" pos q)
  (ros::publish
   (if (eq *target-limb* :rarm)
       "/rel2_posture_controller/command/posestamped/rarm"
     "/rel2_posture_controller/command/posestamped/larm")
   (instance geometry_msgs::posestamped
	     :init
	     :header
	     (instance std_msgs::header
		       :init
		       :stamp (ros::time 0)
		       :frame_id (format nil "~A" *target-limb*))
	     :pose
	     (instance geometry_msgs::pose
		       :init
		       :position
		       (instance geometry_msgs::Point :init
				 :x (* 1e-3 (aref pos 0))
				 :y (* 1e-3 (aref pos 1))
				 :z (* 1e-3 (aref pos 2)))
		       :orientation
		       (instance geometry_msgs::Quaternion :init
				 :w (aref q 0)
				 :x (aref q 1)
				 :y (aref q 2)
				 :z (aref q 3))))))

(defvar *spacenav-cnt* 0)
(defun spacenav-controller-callback
  (msg)
  (cond
   ((and (> (aref (send msg :buttons) 0) 0.1)
	 (> (aref (send msg :buttons) 1) 0.1))
    (cond
     ((eq *target-limb* :rarm)
      (setq *target-limb* :larm))
     (t (setq *target-limb* :rarm)))
    (format t "[spacenav2posestamped] ~A mode~%" *target-limb*)
    (publish-posestamped (make-coords))
    (unix::usleep (round (* 1000 1000 0.5)))
    )
   ((> (aref (send msg :buttons) 0) 0.1)
    (setq *updated* t)
    (format t "[spacenav2posestamped] grasp~%")
    (ros::publish
     "/posture_controller/command/string"
     (instance std_msgs::string
	       :init
	       :data
	       (format nil "(send *ri* :start-grasp ~A)" *target-limb*))))
   ((> (aref (send msg :buttons) 1) 0.1)
    (setq *updated* t)
    (format t "[spacenav2posestamped] hook~%")
    (ros::publish
     "/posture_controller/command/string"
     (instance std_msgs::string
	       :init
	       :data
	       (format nil "(send *ri* :stop-grasp ~A)" *target-limb*)))
    )
   ((> (norm (send msg :axes)) 0.1)
    (let* ((move
	    (scale
	     100
	     (coerce
	      (subseq (send msg :axes) 0 3)
	      float-vector)))
	   (rpy
	    (reverse
	     (scale
	      0.75
	      (coerce
	       (subseq (send msg :axes) 3 6)
	       float-vector))))
	   (pos move)
	   ;;(if (> (norm move) 30)
	   ;;(setq move (scale 30 (normalize-vector move)))
	   ;;move))
	   (rot
	    (send
	     (make-coords :rpy rpy)
	     :worldrot)))
      (format t "[spacenav2posestamped] ~A move ~A ~A x~A~%"
	      *target-limb*
	      move rpy
	      *spacenav-cnt*)
      (publish-posestamped
       (make-coords :pos pos :rot rot))
      (incf *spacenav-cnt*)
      (setq *updated* t)))
   (t (setq *spacenav-cnt* 0)
      (format t "[spacenav-controller] spacenav skip~%"))))

(ros::subscribe
 "/spacenav/joy"
 sensor_msgs::Joy
 #'spacenav-controller-callback 1)

(ros::advertise
 "/rel2_posture_controller/command/posestamped/larm"
 geometry_msgs::posestamped)
(ros::advertise
 "/rel2_posture_controller/command/posestamped/rarm"
 geometry_msgs::posestamped)

(ros::advertise
 "/posture_controller/command/string"
 std_msgs::string)

(ros::rate 1)
(do-until-key
 (ros::spin-once)
 (ros::sleep))

