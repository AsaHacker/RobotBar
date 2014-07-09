#!/usr/bin/env roseus

(ros::roseus "table_reacher")
(ros::roseus-add-msgs "sensor_msgs")
(ros::roseus-add-msgs "geometry_msgs")

(ros::advertise
 "/abs_posture_controller/command/posestamped/rarm/force"
 geometry_msgs::posestamped)
(ros::advertise
 "/abs_posture_controller/command/posestamped/larm/force"
 geometry_msgs::posestamped)

(require "package://baxtereus/baxter.l")
(require "package://baxter_bartender/euslisp/model/table-audience.l")

(defvar *robot* (baxter))
(table-audience)

(defun publish-posestamped
  (coords
   &key
   (limb :rarm)
   (pos (send coords :worldpos))
   (rot (send coords :worldrot))
   (q (matrix2quaternion rot))
   )
  (format t "[publish-posestamped] pos=~A/q=~A~%" pos q)
  (ros::publish
   (if (eq limb :rarm)
       "/abs_posture_controller/command/posestamped/rarm/force"
     "/abs_posture_controller/command/posestamped/larm/force")
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

(defun table-reaching
  (msg)
  (if (or (null (send msg :data))
	  (zerop (length (send msg :data))))
      (return-from nil nil))
  (let* ((req (read-from-string (send msg :data)))
	 (arm (find (car req) '(:rarm :larm)))
	 (target
	  (find-if
	   #'(lambda (tag) (string-equal
			    (send tag :name)
			    (format nil "~A" (cadr req))))
	   (send *table-audience* :get-val 'audience-list))))
    (cond
     ((and arm target)
      (setq target
	    (make-coords
	     :pos (send target :worldpos)
	     :rot
	     (send
	      (send *robot* arm :end-coords :copy-worldcoords)
	      :worldrot)))
      (publish-posestamped
       (send
	(send (car (send *robot* :links)) :copy-worldcoords)
	:transformation
	(send (send target :copy-worldcoords) :translate #f(0 0 200) :world)
	:local)
       :limb arm)))))

(ros::subscribe
 "/table_reacher/request"
 std_msgs::string
 #'table-reaching 1)

(ros::rate 10)
(do-until-key
 (if (not (ros::ok)) (return-from nil nil))
 (ros::spin-once)
 (ros::sleep))
