#!/usr/bin/env roseus

(ros::roseus "oshibori_reacher")
(ros::roseus-add-msgs "sensor_msgs")
(ros::roseus-add-msgs "geometry_msgs")

(ros::advertise
 "/abs_posture_controller/command/posestamped/rarm/ufo"
 geometry_msgs::posestamped)
(ros::advertise
 "/abs_posture_controller/command/posestamped/larm/ufo"
 geometry_msgs::posestamped)

(require "package://baxtereus/baxter.l")

(defvar *target-coords* (make-coords))
(defvar *target-limb* nil)
(defvar *robot* (baxter))

(defun oshibori-pos-callback
  (msg)
  (format t "[oshibori] detected!~%")
  (setq *detect-cnt* 5)
  (let* ((move
	  (mapcar
	   #'(lambda (k)
	       (v-
		(send msg :data)
		(send *robot* k :end-coords :worldpos)
		))
	   '(:larm)))
	 arm
	 (min-move
	  (cond
	   ((> (norm (car move))
	       (norm (cadr move)))
	    (setq arm :larm)
	    (cadr move))
	   (t
	    (setq arm :rarm)
	    (car move))))
	 buf)
    (setq *target-limb* arm)
    (setq *target-coords*
	  (send (send (car *robot* :links) :copy-worldcoords)
		;;(send *robot* arm :end-coords :copy-worldcoords)
		:transformation
		(make-coords :pos move)))))

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
       "/abs_posture_controller/command/posestamped/rarm/oc"
     "/abs_posture_controller/command/posestamped/larm/oc")
   (instance geometry_msgs::posestamped
	     :init
	     :header
	     (instance std_msgs::header
		       :init
		       :stamp (ros::time 0)
		       :frame_id (format nil "~A" (send (car (send *robot* :links)) :name)))
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

(defun oshibori-reaching
  (msg)
  (if *target-limb* (publish-posestamped *target-coords*)))

(ros::subscribe
 "/oshibori/position"
 std_msgs::float32multiarray
 #'oshibori-pos-callback 1)

(ros::subscribe
 "/oshibori_reacher/request"
 std_msgs::empty
 #'oshibori-reaching 1)

(ros::rate 10)
(do-until-key
 (if (not (ros::ok)) (return-from nil nil))
 (ros::spin-once)
 (ros::sleep))
