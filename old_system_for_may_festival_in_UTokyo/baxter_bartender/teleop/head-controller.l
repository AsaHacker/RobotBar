#!/usr/bin/env roseus

(ros::roseus "baxter_head_controller")
(ros::roseus-add-msgs "sensor_msgs")
(ros::roseus-add-msgs "geometry_msgs")
(ros::roseus-add-msgs "baxter_core_msgs")

(require "package://baxtereus/baxter.l")

(defvar *robot* (baxter))
(defvar *pan* 0)
(defvar *pan-max* 1.0)
(defvar *pan-min* -1.0)
(defvar *speed* 0)
(defvar *speed-max* 100)
(defvar *speed-min* 0)

(defvar *face-tracking-on* nil)
(defvar *turtlebot-tracking-on* nil)
(defvar *arm-tracking-on* nil)

(defun pan-updater
  (msg)
  (setq *pan* (send msg :pan)))

(defun relative-head-controller
  (msg)
  (setq *pan*
	(max (min *pan-max* (+ *pan* (send msg :target))) *pan-min*))
  (setq *speed*
	(max (min *speed-max* (send msg :speed)) *speed-min*))
  (format t "[relative-head-controller] pan ~A x ~A~%" *pan* *speed*)
  (ros::publish
   "/robot/head/command_head_pan"
   (instance baxter_core_msgs::headpancommand
	     :init
	     :target *pan*
	     :speed *speed*)))

(defun head-pose-callback
  (msg
   &key
   (frame (send (send msg :header) :frame_id))
   (pos (send (send msg :pose) :position))
   (rot (send (send msg :pose) :orientation))
   (c (send
       (send (car (send *robot* :links)) :copy-worldcoords)
       :transform
       (make-coords
	:pos
	(map float-vector #'(lambda (k) (send pos k)) '(:x :y :z))
	:rot
	(quaternion2matrix
	 (map float-vector #'(lambda (k) (send rot k)) '(:w :x :y :z))))))
   (x (float-vector 1 0 0))
   (pan) (pan-vector))
  (if (not (string-equal frame "base"))
      (format t "[head-controller] causion!! transform from torso~%"))
  (setq pan-vector
	(normalize-vector
	 (map float-vector #'*
	      '(1 1 0)
	      (send c :worldpos))))
  (setq pan
	(* (if (plusp (aref (v* x pan-vector) 2)) +1 -1)
	   (acos (v. pan-vector x))))
  (ros::publish
   "/robot/head/command_head_pan"
   (instance baxter_core_msgs::headpancommand
	     :init
	     :target pan
	     :speed 30))
  )

(defun face-track-controller
  (msg)
  (let* ((v (send msg :data))
	 (center (float-vector
		  (+ (aref v 0) (/ (aref v 2) 2.0))
		  (+ (aref v 1) (/ (aref v 3) 2.0)))))
    (if *face-tracking-on*
	(ros::publish
	 "/robot/head/command_head_pan"
	 (instance baxter_core_msgs::headpancommand
		   :init
		   :target (- (aref center 0) 0.5)
		   :speed 50)))))

(defun turtlebot-track-controller
  (msg)
  (if *turtlebot-tracking-on*
      (head-pose-callback msg)))

(defun face-track-swicher
  (msg)
  (format t "[face-track-switch] ~A~%" (send msg :data))
  (cond
   ((string-equal (send msg :data) "ON")
    (setq *face-tracking-on* t))
   ((string-equal (send msg :data) "OFF")
    (setq *face-tracking-on* nil)))
  (format t "   ---->    ~A~%" *face-tracking-on*)
  )

(defun rarm-track-controller
  (msg)
  (if (eq *arm-tracking-on* :rarm)
      (head-pose-callback msg)))

(defun larm-track-controller
  (msg)
  (if (eq *arm-tracking-on* :larm)
      (head-pose-callback msg)))

(defun turtlebot-track-swicher
  (msg)
  (format t "[turtlebot-track-switch] ~A~%" (send msg :data))
  (cond
   ((string-equal (send msg :data) "ON")
    (setq *turtlebot-tracking-on* t))
   ((string-equal (send msg :data) "OFF")
    (setq *turtlebot-tracking-on* nil)))
  (format t "   ---->    ~A~%" *turtlebot-tracking-on*)
  )

(defun arm-track-swicher
  (msg)
  (format t "[arm-track-switch] ~A~%" (send msg :data))
  (let* ((str (send msg :data)) key)
    (cond
     ((plusp (length str))
      (setq key (read-from-string str))
      (setq *arm-tracking-on* key))
     (t (setq *arm-tracking-on* nil))))
  (format t "   ---->    ~A~%" *arm-tracking-on*)
  )

(ros::advertise
 "/robot/head/command_head_pan"
 baxter_core_msgs::headpancommand)

(ros::subscribe
 "/robot/head/head_state"
 baxter_core_msgs::headstate
 #'pan-updater 1)

(ros::subscribe
 "/robot/head/command_head_pan/relative"
 baxter_core_msgs::headpancommand
 #'relative-head-controller 1)

(ros::subscribe
 "/robot/head/command/posestamped"
 geometry_msgs::posestamped
 #'head-pose-callback 1)

(ros::subscribe
 "/face_detector/normalized_rect"
 std_msgs::Float32MultiArray
 #'face-track-controller 1)

(ros::subscribe
 "/head_controller/face_tracking/switch"
 std_msgs::string
 #'face-track-swicher 1)

(ros::subscribe
 "/turtlebot/posestamped"
 ;; "/turtlebot/posestamped"
 geometry_msgs::posestamped
 #'turtlebot-track-controller 1)

(ros::subscribe
 "/baxter/larm/endcoords/posestamped"
 geometry_msgs::posestamped
 #'larm-track-controller 1)

(ros::subscribe
 "/baxter/rarm/endcoords/posestamped"
 geometry_msgs::posestamped
 #'rarm-track-controller 1)

(ros::subscribe
 "/head_controller/turtlebot_tracking/switch"
 std_msgs::string
 #'turtlebot-track-swicher 1)

(ros::subscribe
 "/head_controller/arm_tracking/switch"
 std_msgs::string
 #'arm-track-swicher 1)

(ros::rate 1)
(do-until-key
 (if (not (ros::ok)) (return-from nil nil))
 (ros::spin-once)
 (ros::sleep)
 )
