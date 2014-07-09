#!/usr/bin/env roseus

#-:jsk (jsk)
#-:rbrain-basic (rbrain)

(pickview :no-menu t)
(send *pickview* :viewer :viewsurface :bg-color #F(1 1 1 0))

(defun dump-pose-data
  (&key
   (name "test")
   (path (format nil "~A.jpg" name))
   (links
    (remove-if
     #'(lambda (l)
	 (not (find-method (send l :joint) :joint-angle)))
     (flatten
      (append
       (mapcar
	#'(lambda (k) (send *robot* k :links))
	'(:rarm :larm))
       nil))))
   )
  (send *viewer* :draw-objects)
  (send *viewer* :viewsurface :write-to-jpg-file path)
  (list
   'list
   (list 'cons :name name)
   (list 'cons :path path)
   (list 'cons :joint_states
	 (list
	  'list
	  (list 'cons :name
		(cons
		 'list
		 (mapcar #'(lambda (n) (format nil "~A" n)) (send-all links :name))))
	  (list 'cons :position
		(cons
		 'list
		 (mapcar #'deg2rad
			 (send-all (send-all links :joint) :joint-angle))))))))


#|

roseus pose-play-panel-util.l
(require "package://baxtereus/baxter.l")
(defvar *robot* (baxter))
(objects *robot*)

(defmethod baxter-robot
  (:reset-manip-pose
   nil
   (send self :angle-vector
	 (copy-seq #f(-8.52539 -85.1221 0.966797 150.0 -4.87793 20.3467 8.52539 -85.1221 -0.966797 150.0 4.87793 20.3467 -1.2085))))
  (:reset-manip-pose2
   nil
   (send self :angle-vector
	 (copy-seq #f(-8.52539 -85.1221 0.966797 150.0 -4.87793 20.3467 -12.3792 5.80091 -1.35362 83.8349 74.6549 91.3975 0.966797))))
  (:reset-bartender-pose
   nil
   (send self :angle-vector
	 (copy-seq #f(77.3218 24.5215 48.6255 96.5039 -152.644 0.0 -77.3218 24.5215 -48.6255 96.5039 152.644 0.0 -1.29639))))
  (:reset-side-arm-pose
   nil
   (send self :angle-vector
	 (copy-seq #f(60.9961 -71.1694 14.0405 141.196 -13.5571 9.03076 -60.9961 -71.1694 -14.0405 141.196 13.5571 9.03076 -3.0542))))
  (:reset-fight-pose
   nil
   (send self :angle-vector
	 (copy-seq #f(96.6797 -14.7656 -13.3154 102.656 -42.3633 84.5068 -96.6797 -14.7656 13.3154 102.656 42.3633 84.5068 1.05469))))
  (:reset-hook-pose
   nil
   (send self :angle-vector
	 (copy-seq #f(12.3792 5.80091 1.35362 83.8349 -74.6549 91.3975 -12.3792 5.80091 -1.35362 83.8349 74.6549 91.3975 0.966797))))
  (:mirror-arm-angle-vector
   (&key (from :rarm) (to :larm))
   (send self to :angle-vector
	 (map float-vector
	      #'*
	      #F(-1 1 -1 1 -1 1)
	      (send self from :angle-vector))))
  )
