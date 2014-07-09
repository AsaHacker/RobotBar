#!/usr/bin/env roseus

(require "package://baxtereus/baxter-interface.l")
(defvar *joint_observe_service_context* "_require_")
(require "joint_observe_service.l")
;; (require "table-toich-it.l")

(baxter-init)
(setq *robot* *baxter*)

(mapcar
 #'(lambda (k)
     (if (send *robot* k)
	 (and (send *robot* k :min-angle -1e+10)
	      (send (send *ri* :robot) k :min-angle -1e+10)
	      (send *robot* k :max-angle +1e+10)
	      (send (send *ri* :robot) k :max-angle +1e+10)
	      )))
 '(:left_w2 :torso_t0 :right_w2))

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

(defun ufo-ik
  (arm coords &key (relative? t) (debug-view nil))
  (send *robot* :inverse-kinematics
	(print
	 (make-coords
	  :pos (v+
		(if relative?
		    (send *robot* arm :end-coords :worldpos)
		  (float-vector 0 0 0))
		(send coords :worldpos))
	  :rpy (list -3.14 0 -3.14)))
	:move-target (send *robot* arm :end-coords)
	:link-list
	(send *robot* :link-list
	      (send *robot* arm :end-coords :parent))
	:rotation-axis :z
	:debug-view debug-view
	))

(defun hook-ik
  (arm coords &key (relative? t) (debug-view nil))
  (send *robot* :inverse-kinematics
	(print
	 (make-coords
	  :pos (v+
		(if relative?
		    (send *robot* arm :end-coords :worldpos)
		  (float-vector 0 0 0))
		(send coords :worldpos))
	  :rpy (if (eq arm :rarm)
		   (list (deg2rad 180) (deg2rad -90) (deg2rad 90))
		 (list (deg2rad -180) (deg2rad -90) (deg2rad -90)))))
	:move-target (send *robot* arm :end-coords)
	:link-list
	(send *robot* :link-list
	      (send *robot* arm :end-coords :parent))
	:rotation-axis :x
	:debug-view debug-view
	))

(defun normal-ik
  (arm coords &key (relative? t) (debug-view nil))
  (send *robot* :inverse-kinematics
	coords
	:move-target (send *robot* arm :end-coords)
	:link-list
	(send *robot* :link-list
	      (send *robot* arm :end-coords :parent))
	;;:rotation-axis :x
	:debug-view debug-view
	))

(defun oc-ik
  (arm coords &key (relative? t) (debug-view nil))
  (if (eq arm :rarm)
      (hook-ik arm coords
	       :relative? relative? :debug-view debug-view)
    (ufo-ik arm coords
	    :relative? relative? :debug-view debug-view)))

(defun real2model
  nil
  (send *robot* :angle-vector
	(send *ri* :state :potentio-vector))
  (if (and (boundp '*viewer*) *viewer*)
      (send *viewer* :draw-objects)))

(defvar *min-angle-veclocity* 5)
(defun model2real
  (&key (sleep 3000)
	(wait nil)
	(min-sleep 1000)
	(min-check *min-angle-veclocity*))
  (cond
   ((and
     min-check
     (< (norm
	 (v- (send *robot* :angle-vector)
	     (send *ri* :state :potentio-vector)))
	min-check))
    (format t " [model2real] min-check ~A~%"
	    min-check)
    nil)
   (t
    (send *ri* ;;:angle-vector
	  :angle-vector
	  (send *robot* :angle-vector)
	  sleep)
    (unix:usleep (round (* min-sleep 1000)))
    (if wait (send *ri* :wait-interpolation))
    t)
   ))

(ros::advertise
 "/gripper_controller_node/rarm/command/string"
 std_msgs::string)
(ros::advertise
 "/gripper_controller_node/larm/command/string"
 std_msgs::string)

(defun move-and-collision-check
  (&key (time-step 0.1) ;; sec
	(max-time 0.49))
  (let* ((res (call-joint-observe-service :check)))
    (read-from-string (send res :res))))

(defmethod baxter-interface
  (:wait-interpolation
   ()
   (unix:sleep 1)
   (call-joint-observe-service :wait-interpolation))
  (:angle-vector
   (av &optional
       (tm 3000)
       (time-step 100)
       (ctype controller-type)
       (pav (copy-seq (send robot :angle-vector))))
   (let ((idx 0)
         (diff-av (v- av (or (send self :state :potentio-vector)
			     (send robot :angle-vector)))))
     (dolist (jt (send robot :joint-list))
       (cond ((derivedp jt linear-joint)
              (setq tm (max tm (abs (/ (elt diff-av idx) (send jt :max-joint-velocity))))))
             ((derivedp jt rotational-joint) ;; msec = deg2rad(deg) / [rad/s] / 0.001
              (setq tm (max tm (abs (/ (deg2rad (elt diff-av idx))
                                       (send jt :max-joint-velocity) 0.001))))))
       (incf idx)
       ))
   ;; for simulation mode
   (when (send self :simulation-modep)
     (if av
	 (let* ((prev-av (send robot :angle-vector))
		(scale-av (send self :sub-angle-vector av prev-av)))
	   (do ((curr-tm 0.0 (+ curr-tm 100.0)))
	       ((>= curr-tm tm))
	       (send robot :angle-vector (v+ prev-av (scale (/ curr-tm tm) scale-av)))
	       (send self :publish-joint-state)
	       (if viewer (send self :draw-objects))))))
   (send robot :angle-vector av)
   (let ((cacts (gethash ctype controller-table))
	 buf (size (/ tm time-step)))
     (unless cacts
       (warn ";; controller-type: ~A not found" ctype)
       (return-from :angle-vector))
     (cond
      ((< size 1)
       (setq buf (list av (scale 0 av) (* tm 1e-3))))
      (t
       (dotimes (i (+ 1 size))
	 (dotimes (j (if (eq i 0) 5 1))
	   (push
	    (list (v+ (scale (/ (* 1.0 i) size) av)
		      (scale (- 1.0 (/ (* 1.0 i) size)) pav))
		  (scale 0 av)
		  ;; 0)
		  (* i time-step 1e-3))
	    buf)))
       ))
     ;; (print buf)
     (mapcar
      #'(lambda (action param)
          (send self :send-ros-controller
                action (cdr (assoc :joint-names param)) ;; action server and joint-names
                0  ;; start time
		buf))
      cacts (send self ctype)))
   av)
  (:smooth-angle-vector
   (av &optional
       (tm 3000)
       (time-step 100)
       (ctype controller-type)
       (pav (copy-seq (send robot :angle-vector)))
       (dav (v- av pav)))
   (let ((idx 0)
         (diff-av (v- av (or (send self :state :potentio-vector)
			     (send robot :angle-vector)))))
     (dolist (jt (send robot :joint-list))
       (cond ((derivedp jt linear-joint)
              (setq tm (max tm (abs (/ (elt diff-av idx) (send jt :max-joint-velocity))))))
             ((derivedp jt rotational-joint) ;; msec = deg2rad(deg) / [rad/s] / 0.001
              (setq tm (max tm (abs (/ (deg2rad (elt diff-av idx))
                                       (send jt :max-joint-velocity) 0.001))))))
       (incf idx)
       ))
   ;; for simulation mode
   (when (send self :simulation-modep)
     (if av
	 (let* ((prev-av (send robot :angle-vector))
		(scale-av (send self :sub-angle-vector av prev-av)))
	   (do ((curr-tm 0.0 (+ curr-tm 100.0)))
	       ((>= curr-tm tm))
	       (send robot :angle-vector (v+ prev-av (scale (/ curr-tm tm) scale-av)))
	       (send self :publish-joint-state)
	       (if viewer (send self :draw-objects))))))
   (send robot :angle-vector av)
   (let ((cacts (gethash ctype controller-table))
	 buf (size (/ tm time-step))
	 tnow x dx ddx sc)
     (unless cacts
       (warn ";; controller-type: ~A not found" ctype)
       (return-from :angle-vector))
     (cond
      ((< size 1)
       (setq buf (list (list av (scale 0 av) (* tm 1e-3)))))
      (t
       (setq sc (/ 2.0 tm))
       (dotimes (i (+ 1 size))
	 (setq tnow (* tm (- 1.0 (/ (* 1.0 i) size))))
	 (setq x
	       (*
		sc
		(+
		 (/ tnow 2.0)
		 (* (/ tm (* -4.0 3.14)) (sin (* sc 3.14 tnow))))))
	 (setq dx
	       (*
		sc
		(+
		 0.5
		 (* -0.5 (cos (* sc 3.14 tnow))))))
	 (setq ddx
	       (*
		sc
		(* sc 3.14 (sin (* sc 3.14 tnow)))))
	 (push
	  (list (v+ pav (scale x dav))
		(scale dx dav)
		;; (scale ddx dav)
		(* i time-step 1e-3)) ;;(* time-step 1e-3))
	  buf))))
     ;; (print buf)
     (mapcar
      #'(lambda (action param)
          (send self :send-ros-controller
                action (cdr (assoc :joint-names param)) ;; action server and joint-names
                0  ;; start time
		buf))
      cacts (send self ctype))
     ;;av
     buf
     ))
  (:start-grasp
   (&optional (arm :arms))
   (mapcar
    #'(lambda (str)
	(ros::publish str (instance std_msgs::string :init :data "close")))
    (case arm
	  (:rarm (list "/gripper_controller_node/rarm/command/string"))
	  (:larm (list "/gripper_controller_node/larm/command/string"))
	  (t (list "/gripper_controller_node/rarm/command/string"
		   "/gripper_controller_node/larm/command/string")))))
  (:stop-grasp
   (&optional (arm :arms))
   (mapcar
    #'(lambda (str)
	(ros::publish str (instance std_msgs::string :init :data "open")))
    (case arm
	  (:rarm (list "/gripper_controller_node/rarm/command/string"))
	  (:larm (list "/gripper_controller_node/larm/command/string"))
	  (t (list "/gripper_controller_node/rarm/command/string"
		   "/gripper_controller_node/larm/command/string")))))
  (:hand-calibrate
   (&optional (arm :arms))
   (mapcar
    #'(lambda (str)
	(ros::publish str (instance std_msgs::string :init :data "calib")))
    (case arm
	  (:rarm (list "/gripper_controller_node/rarm/command/string"))
	  (:larm (list "/gripper_controller_node/larm/command/string"))
	  (t (list "/gripper_controller_node/rarm/command/string"
		   "/gripper_controller_node/larm/command/string")))))
  )

;; (print 'hoge)
#|

(defmethod baxter-interface
  (:send-ros-controller
   (action joint-names starttime trajpoints)
   (when (send self :simulation-modep)
     (return-from :send-ros-controller nil))
   (if (and warningp
            (yes-or-no-p (format nil "~C[3~CmAre you sure to move the real robot? (~A) ~C[0m" #x1b 49 (send action :name) #x1b)))
       (return-from :send-ros-controller nil))
   (let* ((goal (send action :make-goal-instance))
          (goal-points nil)
          (st (ros::time+ (ros::time-now) (ros::time starttime)))
          (joints (mapcar #'(lambda (x)
                              (send robot (intern (string-upcase x) *keyword-package*)))
                          joint-names)))
     (send goal :header :seq 1)
     (send goal :header :stamp st)
     (send goal :goal :trajectory :joint_names joint-names)
     (send goal :goal :trajectory :header :stamp st)
     (dolist (trajpt trajpoints)
       (let* ((all-positions (elt trajpt 0))
              (all-velocities (elt trajpt 1))
	      (all-effort
	       (if (> (length trajpt) 3)
		   (elt trajpt 2)
		 (mapcar #'(lambda (v) (scale 0 v)) all-velocities)))
              (duration (elt trajpt (if (> (length trajpt) 3) 3 2)))
              (positions (instantiate float-vector (length joint-names)))
              (velocities (instantiate float-vector (length joint-names)))
	      (efforts (instantiate float-vector (length joint-names))))
         (dotimes (i (length joints))
           (let* ((joint (elt joints i))
                  (id (position joint (send robot :joint-list)))
                  p v e)
             (setq p (elt all-positions id)
                   v (elt all-velocities id)
		   e (elt all-effort id))
             (cond
              ((derivedp joint rotational-joint)
               (setq p (deg2rad p))
               (setq v (deg2rad v))
	       (setq e (deg2rad e)))
              (t
               (setq p (* 0.001 p))
               (setq v (* 0.001 v))
	       (setq e (* 0.001 e))))
             (setf (elt positions i) p)
             (setf (elt velocities i) v)))
         (push (instance trajectory_msgs::JointTrajectoryPoint
                         :init
                         :positions positions
                         :velocities velocities
			 :accelerations efforts
			 :effort efforts
                         :time_from_start (ros::time duration))
               goal-points)
         ))
     (send self :spin-once)
     (send goal :goal :trajectory :points goal-points)
     (send action :send-goal goal)
     ))
  (:angle-vector
   (av &optional
       (tm 3000)
       (time-step 500)
       (ctype controller-type)
       (pav (copy-seq (send robot :angle-vector))))
   (let ((idx 0)
         (diff-av (v- av (or (send self :state :potentio-vector)
			     (send robot :angle-vector)))))
     (dolist (jt (send robot :joint-list))
       (cond ((derivedp jt linear-joint)
              (setq tm (max tm (abs (/ (elt diff-av idx) (send jt :max-joint-velocity))))))
             ((derivedp jt rotational-joint) ;; msec = deg2rad(deg) / [rad/s] / 0.001
              (setq tm (max tm (abs (/ (deg2rad (elt diff-av idx))
                                       (send jt :max-joint-velocity) 0.001))))))
       (incf idx)
       ))
   ;; for simulation mode
   (when (send self :simulation-modep)
     (if av
	 (let* ((prev-av (send robot :angle-vector))
		(scale-av (send self :sub-angle-vector av prev-av)))
	   (do ((curr-tm 0.0 (+ curr-tm 100.0)))
	       ((>= curr-tm tm))
	       (send robot :angle-vector (v+ prev-av (scale (/ curr-tm tm) scale-av)))
	       (send self :publish-joint-state)
	       (if viewer (send self :draw-objects))))))
   (send robot :angle-vector av)
   (let ((cacts (gethash ctype controller-table))
	 buf (size (/ tm time-step)))
     (unless cacts
       (warn ";; controller-type: ~A not found" ctype)
       (return-from :angle-vector))
     (if (< size 1)
	 (setq buf (list av (scale 0 av) (* tm 1e-3)))
       (dotimes (i (+ 1 size))
	 (push
	  (list (v+ (scale (/ (* 1.0 i) size) av)
		    (scale (- 1.0 (/ (* 1.0 i) size)) pav))
		(scale 0 av)
		0) ;;(* time-step 1e-3))
	  buf)))
     ;; (print buf)
     (mapcar
      #'(lambda (action param)
          (send self :send-ros-controller
                action (cdr (assoc :joint-names param)) ;; action server and joint-names
                0  ;; start time
		buf))
      cacts (send self ctype)))
   av)
  (:smooth-angle-vector
   (av &optional
       (tm 3000)
       (time-step 50)
       (ctype controller-type)
       (pav (copy-seq (send robot :angle-vector)))
       (dav (v- av pav)))
   (let ((idx 0)
         (diff-av (v- av (or (send self :state :potentio-vector)
			     (send robot :angle-vector)))))
     (dolist (jt (send robot :joint-list))
       (cond ((derivedp jt linear-joint)
              (setq tm (max tm (abs (/ (elt diff-av idx) (send jt :max-joint-velocity))))))
             ((derivedp jt rotational-joint) ;; msec = deg2rad(deg) / [rad/s] / 0.001
              (setq tm (max tm (abs (/ (deg2rad (elt diff-av idx))
                                       (send jt :max-joint-velocity) 0.001))))))
       (incf idx)
       ))
   ;; for simulation mode
   (when (send self :simulation-modep)
     (if av
	 (let* ((prev-av (send robot :angle-vector))
		(scale-av (send self :sub-angle-vector av prev-av)))
	   (do ((curr-tm 0.0 (+ curr-tm 100.0)))
	       ((>= curr-tm tm))
	       (send robot :angle-vector (v+ prev-av (scale (/ curr-tm tm) scale-av)))
	       (send self :publish-joint-state)
	       (if viewer (send self :draw-objects))))))
   (send robot :angle-vector av)
   (let ((cacts (gethash ctype controller-table))
	 buf (size (/ tm time-step))
	 tnow x dx ddx sc)
     (unless cacts
       (warn ";; controller-type: ~A not found" ctype)
       (return-from :angle-vector))
     (cond
      ((< size 1)
       (setq buf (list (list av (scale 0 av) (* tm 1e-3)))))
      (t
       (setq sc (/ 2.0 tm))
       (dotimes (i (+ 1 size))
	 (setq tnow (* tm (- 1.0 (/ (* 1.0 i) size))))
	 (setq x
	       (*
		sc
		(+
		 (/ tnow 2.0)
		 (* (/ tm (* -4.0 3.14)) (sin (* sc 3.14 tnow))))))
	 (setq dx
	       (*
		sc
		(+
		 0.5
		 (* -0.5 (cos (* sc 3.14 tnow))))))
	 (setq ddx
	       (*
		sc
		(* sc 3.14 (sin (* sc 3.14 tnow)))))
	 (push
	  (list (v+ pav (scale x dav))
		(scale dx dav)
		(scale ddx dav)
		(* time-step 1e-3)) ;;(* time-step 1e-3))
	  buf))))
     ;; (print buf)
     (mapcar
      #'(lambda (action param)
          (send self :send-ros-controller
                action (cdr (assoc :joint-names param)) ;; action server and joint-names
                0  ;; start time
		buf))
      cacts (send self ctype))
     ;;av
     buf
     ))
  )

(defun smooth-angle-vector-test
  nil
  (let ((avl
	 (send *ri* :smooth-angle-vector
	       (send *robot* :angle-vector) 6000)))
    (mapcar
     #'(lambda (av tm)
	 (send *robot* :angle-vector av)
	 (send *viewer* :draw-objects)
	 (x::window-main-one)
	 (unix::usleep
	  (round (* tm 1000 1000)))
	 )
     (mapcar #'car avl)
     (mapcar #'cadddr avl))))


(let* ((mt (send *robot* :rarm :end-coords))
       (ll (send *robot* :link-list (send mt :parent)))
       (c
	(make-coords
	 :pos (copy-seq (send mt :worldpos))
	 :rpy (list (deg2rad 180) (deg2rad -90) (deg2rad 90)))))
  (send *robot* :inverse-kinematics
	c
	:move-target mt
	:link-list ll
	:rotation-axis :x
	;;:translation-axis nil
	:debug-view :no-message
	))
