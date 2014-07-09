#!/usr/bin/env roseus

;; (ros::roseus "turtlebot_posestamped_publisher")
(ros::roseus-add-msgs "geometry_msgs")
(ros::advertise "/turtlebot/posestamped"
		geometry_msgs::posestamped 1)

(defvar *tf* (instance ros::transform-listener :init))
(defvar *turtlebot-coords* (make-coords))
(defvar *turtlebot-posestamped* nil)

(defvar *turtlebot-default-coords*
  (make-coords :pos (float-vector 967.789 209.32 -56.879)
	       :rpy '(0.68 -0.01 0.033)))

(defvar *baxter_footprint*
  (make-coords :pos #F(55.503 -1094.959 -212.208)
	       :rpy '(-0.012 -0.004 0.001)))
  ;; (send
  ;;  (send *tf* :lookup-transform
  ;; 	 "/baxter/base_footprint"
  ;; 	 "/odom"
  ;; 	 (ros::time 0.0))
  ;;  :copy-worldcoords))

(defun publish-posestamped
  (coords
   &key
   (pos (send coords :worldpos))
   (rot (send coords :worldrot))
   (topic "/turtlebot/posestamped")
   (frame_id "/world")
   (q (matrix2quaternion rot))
   )
  ;; (format t "[publish-posestamped] pos=~A/q=~A~%" pos q)
  (ros::publish
   topic
   (setq
    *turtlebot-posestamped*
    (instance geometry_msgs::posestamped
	      :init
	      :header
	      (instance std_msgs::header
			:init
			:stamp (ros::time 0)
			:frame_id frame_id)
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
				  :z (aref q 3)))))))

;; (defvar *turtlbot-coords* (make-coords))
(defun turtlebot-posestamped-publish
  nil
  (if (not (find "odom"
		 (send *tf* :get-frame-strings)
		 :test #'string-equal))
      (return-from turtlebot-posestamped-publish nil))
  (let* ((coords
	  (send
	   (copy-object *baxter_footprint*)
	   :transform
	   (send
	    (send *tf* :lookup-transform
		  ;;"/baxter/base_footprint"
		  "/odom"
		  "/turtlebot/base_footprint"
		  (ros::time 0.0))
	    :copy-worldcoords)
	   :local))
  	 (check-coords
  	  (if (null coords)
  	      (return-from turtlebot-posestamped-publish nil)))
  	 (offset
  	  (make-coords :rpy (list 0.9 0 0)))
  	 (target-coords
  	  (send offset :transform coords :local)))
    ;; (setq target-coords coords)
    (send target-coords :translate (float-vector -100 0 0) :world)
    (setq *turtlebot-coords*
  	  (copy-object target-coords))
    ;;(send *viewer* :draw-objects)
    (publish-posestamped
     target-coords
     :topic "/turtlebot/posestamped"
     :frame_id "/world")
    ;;(print *turtlbot-coords*)
    *turtlebot-coords*
    )
  )

#| 

(defvar *baxter_footprint*
  (send
   (send *tf* :lookup-transform
	 "/baxter/base_footprint"
	 "/odom"
	 (ros::time 0.0))
   :copy-worldcoords))

(defvar *turtlebot_footprint*
  (send
   (send *tf* :lookup-transform
	 "/odom"
	 "/turtlebot/base_footprint"
	 (ros::time 0.0))
   :copy-worldcoords))

(defvar *ref*
  (send *tf* :lookup-transform
	"/baxter/base_footprint"
	"/turtlebot/base_footprint"
	(ros::time 0.0)))
