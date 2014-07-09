#!/usr/bin/env roseus

(ros::roseus "joint_observer")
(ros::roseus-add-msgs "sensor_msgs")

(defvar *now-joint-state* (make-hash-table))
(defvar *prv-joint-state* (make-hash-table))
(defvar *ref-joint-state* (make-hash-table))
(defvar *move-thre* (deg2rad 0.3))
(defvar *collision-thre* (deg2rad 4))
(defvar *black-list* '(torso_t0))

(defun now-joint-state-callback
  (msg)
  (let* ((name-list
	  (mapcar
	   #'(lambda (name)
	       (let* ((name-symbol (read-from-string name)))
		 (if (find name-symbol *black-list*) nil name-symbol)))
	   (send msg :name))))
    (flatten
     (map cons
	  #'(lambda (name pos)
	      (cond
	       (name
		(send *prv-joint-state* :enter name
		      (or (send *now-joint-state* :get name) 0))
		(send *now-joint-state* :enter name pos))))
	  name-list (send msg :position)))
    ))

(defun ref-joint-state-callback
  (msg)
  (let* ((name-list
	  (mapcar
	   #'(lambda (name)
	       (let* ((name-symbol (read-from-string name)))
		 (if (find name-symbol *black-list*) nil name-symbol)))
	   (send msg :name))))
    (flatten
     (map cons
	  #'(lambda (name pos)
	      (cond
	       (name (send *ref-joint-state* :enter name pos))))
	  name-list (send msg :position)))
    ))

(defun diff-joint-state
  (js1 js2)
  (map float-vector
       #'(lambda (name pos)
	   (- pos (or (send js2 :get name) 0)))
       (send js1 :list-keys) (send js1 :list-values)))

(ros::subscribe "/robot/ref_joint_states"
		sensor_msgs::jointstate
		#'ref-joint-state-callback 1)
(ros::subscribe "/robot/joint_states"
		sensor_msgs::jointstate
		#'now-joint-state-callback 1)

(ros::advertise "/baxter_joint_observer/move"
		std_msgs::empty)
(ros::advertise "/baxter_joint_observer/collision"
		std_msgs::empty)

(ros::rate 30)
(do-until-key
 (let* ((buf))
   (if (not (ros::ok)) (return-from nil nil))
   (ros::spin-once)
   (cond
    ((> (setq buf (norm (diff-joint-state *now-joint-state* *prv-joint-state*)))
	*move-thre*)
     (format t "[joint_observer] moving ~A~%" buf)
     (ros::publish "/baxter_joint_observer/move"
		   (instance std_msgs::empty :init))))
   (cond
    ((> (setq buf (norm (diff-joint-state *now-joint-state* *ref-joint-state*)))
	*collision-thre*)
     (format t "[joint_observer] collision ~A~%" buf)
     (ros::publish "/baxter_joint_observer/collision"
		   (instance std_msgs::empty :init))))
   (ros::sleep)))
