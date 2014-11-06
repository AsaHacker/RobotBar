#!/usr/bin/env roseus

(ros::roseus "audio_reverse_filtere")

(ros::load-ros-manifest "audio_common_msgs")

(defun audio-callback
  (msg)
  (print msg))

(ros::subscribe "/audio_in"
		audio_common_msgs::audiodata
		#'audio-callback)
(ros::advertise "/audio_out"
		audio_common_msgs::audiodata)

(ros::rate 100)
(do-until-key
 (ros::spin-once)
 (ros::sleep))
