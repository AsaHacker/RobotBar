#!/usr/bin/env roseus

(defvar *tf* (instance ros::transform-listener :init))
(defvar *pcd-transform*)

(defun point-cloud-callback
  (msg)
  (print msg)
  nil)

(defvar *point-cloud*)
(defun point-cloud-callback-main
  (msg)
  (if (null *pcd-transform*)
      (setq *pcd-transform*
	    (send *tf* :lookup-transform
		  "torso" "camera_rgb_optical_frame"
		  (ros::time)))
    (let ((ret (make-eus-pointcloud-from-ros-msg msg :remove-nan t)))
      (setf (get ret :frame_id) (send msg :header :frame_id))
      (setf (get ret :stamp) (send msg :header :stamp))
      (setf (get ret :point_step) (* 3 (send msg :point_step)))
      (send ret :transform
	    *pcd-transform*
	    :local)
      (send ret :convert-to-world)
      (setq *point-cloud* ret) ;;(remove-unreachable-pcd ret))
      (point-cloud-callback msg)
      )))

(defmethod pointcloud
  (:append
   (b)
   (instance pointcloud :init
             :points (append (send self :point-list) (send b :point-list))
             :colors (append (send self :color-list) (send b :color-list))
	     :normals (append (send self :normal-list) (send b :normal-list)))))

(defvar *unsubscribe?* nil)
(defun unsubscribe-point-cloud
  nil
  (if (not *unsubscribe?*)
      (ros::unsubscribe
       "/camera/depth_registered/points")))

(ros::subscribe ;;"/camera/depth/points"
 "/camera/depth_registered/points"
 sensor_msgs::PointCloud2
 #'point-cloud-callback-main 1)
