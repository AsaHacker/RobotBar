#!/usr/bin/env roseus

(ros::roseus-add-msgs "sensor_msgs")
(ros::roseus "analog_rel_controller")

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
       "/rel_posture_controller/command/posestamped/rarm/ufo"
     "/rel_posture_controller/command/posestamped/larm/ufo")
   (instance geometry_msgs::posestamped
	     :init
	     :header
	     (instance std_msgs::header
		       :init
		       :stamp (ros::time 0)
		       :frame_id (format nil "~A" limb))
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


(defun right-analog-callback
  (msg)
  (analog-callback msg :rarm))

(defun left-analog-callback
  (msg)
  (analog-callback msg :larm))

(defun analog-callback
  (msg limb)
  (let ((cm (send msg :data))
	(move 100)
	pos)
    (cond
     ((string-equal cm "north")
      (setq pos (float-vector move 0 0)))
     ((string-equal cm "south")
      (setq pos (float-vector (* -1 move) 0 0)))
     ((string-equal cm "east")
      (setq pos (float-vector 0 (* +1 move) 0)))
     ((string-equal cm "west")
      (setq pos (float-vector 0 (* -1 move) 0)))
     ((string-equal cm "forward")
      (setq pos (float-vector 0 0 (* +1.5 move))))
     ((string-equal cm "backward")
      (setq pos (float-vector 0 0 (* -1.5 move) 0)))
     ((string-equal cm "clicked")
      (setq pos (float-vector 0 0 move))))
    (format t "[analog-controller] ~A -> ~A~%" cm pos)
    (if pos
	(publish-posestamped (make-coords :pos pos) :limb limb))))

(ros::subscribe
 "/kubi_interface/right_hand/event/string"
 std_msgs::string
 #'right-analog-callback)
(ros::subscribe
 "/kubi_interface/left_hand/event/string"
 std_msgs::string
 #'left-analog-callback)

(ros::advertise
 "/rel_posture_controller/command/posestamped/larm/ufo"
 geometry_msgs::posestamped)
(ros::advertise
 "/rel_posture_controller/command/posestamped/rarm/ufo"
 geometry_msgs::posestamped)

(ros::advertise
 "/posture_controller/command/string"
 std_msgs::string)

(ros::rate 10)
(do-until-key
 (ros::spin-once)
 (ros::sleep))

