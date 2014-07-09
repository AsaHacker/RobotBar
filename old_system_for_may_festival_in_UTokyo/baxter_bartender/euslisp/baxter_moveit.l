(require :pr2eus-moveit "package://pr2eus_moveit/euslisp/pr2eus-moveit.l")
(require "package://pr2eus_moveit/tutorials/collision-object-sample.l")

(defclass baxter-moveit-environment
  :super moveit-environment
  :slots ())

(defmethod baxter-moveit-environment
  (:init
   (&rest args &key ((:robot rb) (baxter)) &allow-other-keys)
   (send-super* :init :robot rb
		:frame-id "base"
		:multi-dof-joint-name "virtual_joint"
		:multi-dof-frame-id   "/odom"
		args)
   )
  (:default-configuration ()
   (list (list :rarm
               (cons :group-name "right_arm")
               (cons :target-link
                     (send self :search-link-from-name "right_lower_forearm"))
               (cons :joint-list
		     (send robot :rarm :joint-list))
	       )
         (list :larm
               (cons :group-name "left_arm")
               (cons :target-link
                     (send self :search-link-from-name "left_lower_forearm"))
               (cons :joint-list
		     (send robot :larm :joint-list)))
         (list :both
               (cons :group-name "both_arms")
               (cons :target-link
                     (send self :search-link-from-name "right_lower_forearm"))
               (cons :joint-list
		     (append (send robot :larm :joint-list)
			     (send robot :rarm :joint-list)))
               )
	 )
   )
  )

(defun add-moveit-object
  (obj &key
       (robot *robot*)
       (frame-id "base")
       (collision-object-publisher *co*)
       (frame-link
	(or
	 (find-if
	  #'(lambda (l)
	      (substringp frame-id
			  (format nil "~A" (send l :name))))
	  (send robot :links))
	 (car (send robot :links))))
       (coords
	(send (send frame-link :copy-worldcoords)
	      :transformation
	      (copy-object (send obj :worldcoords)))))
  (send collision-object-publisher :delete-object obj)
  (send collision-object-publisher
	:add-object obj
	:frame-id frame-id
	:relative-pose coords))

(defun plan-real->model
  (&rest args &key (arm :rarm) &allow-other-keys)
  (if (not (and (boundp '*me*) *me*))
      (defvar *me* (instance baxter-moveit-environment :init)))
  (send *me* :sync-robot-model *robot*)
  (send* *me* :motion-plan arm args))

(defun animate-trajectory
  (&key
   (response)
   (trajectory
    (send (send response :trajectory) :joint_trajectory))
   (name-list (send trajectory :joint_names))
   (position-list (send-all (send trajectory :points) :positions))
   (time-span 1.0)
   (time-step (/ time-span (length position-list)))
   )
  (dolist (position position-list)
    (map cons
	 #'(lambda (pos name)
	     (print name)
	     (let ((j (find-if #'(lambda (j) (string-equal name (send j :name)))
			       (send *robot* :joint-list))))
	       (cond
		((null j)
		 (format t "[animate-trajectory] invalid link name=~A~%" name))
		(t (send j :joint-angle (rad2deg pos))))))
	 position name-list)
    (send *viewer* :draw-objects)
    (x::window-main-one)
    (unix:usleep (round (* 1000 1000 time-step)))))

(defun gui-yes-or-no-p
  nil
  (defvar *alert_service_context* "__require__")
  (require "package://baxter_bartender/euslisp/alert-service.l")
  (eq (call-alert-service '(:yes :no)) :yes))

(defun moveit-interpole
  (&key (start
	 (progn
	   (real2model)
	   (copy-seq (send *robot* :angle-vector))))
	(end-key :reset-manip-pose)
	(end (progn (send *robot* end-key)
		    (copy-seq (send *robot* :angle-vector))))
	(real? #'yes-or-no-p)
	(draw? t)
	(arms '(:rarm))
	buf)
  (mapcar
   #'(lambda (arm)
       (send *robot* :angle-vector (copy-seq end))
       (setq buf (plan-real->model :arm arm))
       (if (and buf draw?)
	   (animate-trajectory :response buf))
       (cond
	((and real? buf)
	 (cond
	  ((and (functionp real?)
		(not (funcall real?)))
	   (print 'reject))
	  (t
	   (setq buf (send buf :trajectory :joint_trajectory))
	   (send *ri* :joint-trajectory-to-angle-vector-list arm buf)
	   ;;(print 'move-robot-dummy)
	   )))))
   arms)
  buf)

(defmethod robot-interface
  (:joint-trajectory-to-angle-vector-list
   (move-arm joint-trajectory
             &key ((:diff-sum diff-sum) 0) ((:diff-thre diff-thre) 50)
             (show-trajectory nil) (send-trajectory t)
             ((:speed-scale speed-scale) 1.0) &allow-other-keys)
   (let* (joint-trajectory-points
          joints avs tms  (tm (ros::time -1)) link-coords-list
          (org-av (send robot :angle-vector))
          (dae-link-list
	   (if show-trajectory
	       (send self :find-descendants-dae-links (send robot move-arm :root-link)))))
     (if joint-trajectory
         (setq joint-trajectory-points (send joint-trajectory :points)
               joints (mapcar #'(lambda (x)
                                  (send robot (intern (string-upcase x) *keyword-package*)))
                              (send joint-trajectory :joint_names))))
     (if (= (length joint-trajectory-points) 0)
         (return-from :joint-trajectory-to-angle-vector-list nil))
     (dolist (point joint-trajectory-points)
       (mapc '(lambda (ajoint aposition)
                (send ajoint :ros-joint-angle aposition))
             joints (coerce (send point :positions) cons))
       (push (send robot :angle-vector) avs)
       (when (cadr avs)
         (incf diff-sum (reduce #'+ (map float-vector #'(lambda(x) (abs x)) (v- (car avs) (cadr avs)))))
         (when (> diff-sum diff-thre)
           (setq diff-sum 0)
           (when show-trajectory
             (push (send-all (flatten (send-all dae-link-list :bodies)) :copy-worldcoords)
                   link-coords-list))))
       (push (* (send (ros::time- (send point :time_from_start) tm) :to-sec) 1000 speed-scale) tms)
       (setq tm (send point :time_from_start))
       )
     (when show-trajectory
       (push (send-all (flatten (send-all dae-link-list :bodies)) :copy-worldcoords)
             link-coords-list)
       (send self :show-mesh-traj-with-color
             (flatten (send-all dae-link-list :bodies))
             link-coords-list :lifetime (+ (/ (apply #'+ tms) 1000.0) 10)))
     (cond
      (send-trajectory
       (send robot :angle-vector (car (last avs)))
       (send self :angle-vector-sequence (setq avs (reverse avs)) (setq tms (reverse tms))))
      (t
       (send robot :angle-vector org-av)
       ))
     (if joint-trajectory (list (reverse avs) (reverse tms)))
     ))
  )

#|

(add-moveit-object *table-circle*)
(add-moveit-object *table-audience*)

(defun animate-display
  (msg)
  (let* ((jt (send (car (send msg :trajectory)) :joint_trajectory))
	 (name (send jt :joint_names))
	 (jlist
	  (mapcar
	   #'(lambda (name)
	       (find-if #'(lambda (j) (string-equal name (send j :name))) (send *robot* :joint-list)))
	   name))
	 (points (send jt :points)))
    (mapcar
     #'(lambda (point)
	 (map cons
	      #'(lambda (p j)
		  (if j (send j :joint-angle (rad2deg p))))
	      (send point :positions) jlist)
	 (send *viewer* :draw-objects)
	 (x::window-main-one)
	 (unix:usleep (* 300 1000)))
     (send jt :points))))


(ros::subscribe
 "move_group/display_planned_path"
 moveit_msgs::displaytrajectory
 #'animate-display)
