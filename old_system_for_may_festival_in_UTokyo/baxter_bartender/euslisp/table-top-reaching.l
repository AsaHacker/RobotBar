#!/usr/bin/env roseus

(ros::roseus "tabletop_reaching_test")
(ros::roseus-add-msgs "sensor_msgs")

(require "package://baxtereus/baxter.l")
(require "baxter-util.l")
(require "baxter_moveit.l")
(require "model/table-circle.l")
(require "model/table-audience.l")
(require "baxter-bar-scene.l")
(defvar *touch-it-context* "__require__")
(require "table-touch-it.l")

(defvar *time-step* 3.0)
(defvar *co* (instance collision-object-publisher :init))
(defvar *me* (instance baxter-moveit-environment :init))
(defvar *table-circle* (table-circle))
(defvar *table-audience* (table-audience))

;; (ros::rate (round (/ 1.0 *time-step*)))

(defvar *move-convergence?*)
(defvar *detect-cnt* 5)
(defun test-tabletop-reaching
  nil
  (do-until-key
   (let ((tm (instance mtimer :init)) timer)
     (if (not (ros::ok)) (return-from nil nil))
     (real2model)
     (ros::spin-once)
     (decf *detect-cnt*)
     (if (minusp *detect-cnt*) (send *robot* :reset-manip-pose2))
     (cond
      (*point-cloud*
       (unsubscribe-point-cloud)
       (cond
	((not (and (boundp '*irtviewer*) *irtviewer*))
	 (objects (flatten (list *robot* *point-cloud*)))
	 (send *irtviewer* :change-background #F(0.9 1 0.9))
	 ))
       (send *irtviewer* :objects nil)
       (send *irtviewer* :objects *robot*)
       (send *irtviewer* :objects *point-cloud*)
       (send *irtviewer* :objects *table-circle*)
       (send *irtviewer* :objects *table-audience*)
       (send *viewer* :draw-objects)
       (setq *move-convergence?*
	     (model2real :sleep (* 1000 *time-step*)
			 :wait t))
       ))
     (x::window-main-one)
     (if (< (setq timer (send tm :stop)) *time-step*)
	 (unix:usleep (round (* 1000 1000 (- *time-step* timer))))
       (format t "overslept ~A~%" timer))
     ;; (unix:sleep 3)
     ;; (send *ri* :start-grasp)
     ;; (send *robot* :reset-manip-pose)
     ;; (model2real :sleep 5000)
     ;; (unix:sleep 5)
     ;; (send *ri* :stop-grasp)
     )))

(defun reach-table-until-touch
  (arm estimated-move
       &key
       (max-height 500)
       (res))
  (oc-ik arm
	 (make-coords
	  :pos
	  (scale
	   (min max-height (norm estimated-move))
	   (normalize-vector estimated-move)))
	 :relative? t
	 :debug-view :no-message)
  (model2real :wait t)
  (unix:sleep 2)
  (setq res
	(read-from-string
	 (print (send (call-joint-observe-service :check) :res))))
  (cond
   ((and (null (flatten res)) (eq arm :larm))
    (print 'call-touch-it-server)
    (call-table-touch-it-server arm))
   (t (print 'good-estimation))))

(defun oshibori-reaching
  (msg)
  (format t "[oshibori] detected!~%")
  (setq *detect-cnt* 5)
  (let* ((move
	  (mapcar
	   #'(lambda (k)
	       (v-
		(send msg :data)
		(send *robot* k :end-coords :worldpos)
		))
	   '(:rarm :larm)))
	 (max-move 500)
	 arm
	 (min-move
	  (v+
	   (if (eq arm :larm) #F(-50 -150 0)
	       #F(10 150 0))
	   (cond
	    ((> (norm (car move))
		(norm (cadr move)))
	     (setq arm :larm)
	     (cadr move))
	    (t
	     (setq arm :rarm)
	     (car move)))))
	 (min-move-thre 80)
	 buf)
    (setq buf (copy-seq min-move))
    (setf (aref min-move 2) 0)
    (cond
     ((> (norm min-move) min-move-thre)
      (format t "    move ~A~%" min-move)
      (if (null
	   (oc-ik arm
		   (make-coords
		    :pos
		    (scale
		     (min max-move (norm min-move))
		     (normalize-vector min-move)))
		   :relative? t
		   :debug-view :no-message))
	  (format t "    ik unresolve~%"))))
    (cond
     ((and
       (or (not (> (norm min-move) min-move-thre))
	   (< (norm (v- (send *robot* :angle-vector)
			(send *ri* :state :potentio-vector)))
	      *min-angle-veclocity*))
       (or (format t "    call-touch-it ~A~%" buf) t)
       (reach-table-until-touch arm buf))
      ;;(call-table-touch-it-server arm))
      (send *ri* :start-grasp arm)
      (send *robot* :reset-manip-pose2)
      (model2real :wait t :min-sleep 2000)
      (oc-ik arm
	      (make-coords
	       :pos
	       (v+ (map float-vector #'* #F(1 1 0)
			(random-vector 300.0))
		   #F(700 0 50)))
	      :relative? nil
	      :debug-view :no-message)
      (model2real :wait t :min-sleep 2000)
      (send *ri* :stop-grasp arm)
      (send *robot* :reset-manip-pose2)
      (model2real :wait t :min-sleep 2000)
      ))))

(ros::subscribe
 "/oshibori/position"
 std_msgs::float32multiarray
 #'oshibori-reaching)

#|

(objects (list *robot* *table-audience* *table-circle*))
(add-moveit-object *table-circle*)
(add-moveit-object *table-audience*)
(setq ret (plan-real->model :arm :rarm))
(animate-trajectory :response ret)

(setq traj (send ret :trajectory :joint_trajectory))
(send *ri* :joint-trajectory-to-angle-vector-list :rarm traj)


(defmethod moveit-environment
  (:motion-plan-raw
   (group-name
    &key (scene) (planner-id "")
    (planning-attempts 1) (planning-time 5.0)
    (workspace-x 1.0) (workspace-y 1.0) (workspace-z 1.6) (additional_goal_constraints)
    (goal_constraints) (extra_goal_constraints) (path_constraints) (trajectory_constraints)
    &allow-other-keys)
   (let ((mpr (instance moveit_msgs::motionplanrequest :init))
         res)
     (unless scene
       (setq scene (send self :get-planning-scene)))
     (send mpr :workspace_parameters :header :stamp (ros::time-now))
     (send mpr :workspace_parameters :header :frame_id multi-dof-frame)
     (send mpr :workspace_parameters :max_corner :x workspace-x)
     (send mpr :workspace_parameters :max_corner :y workspace-y)
     (send mpr :workspace_parameters :max_corner :z workspace-z)
     (send mpr :workspace_parameters :min_corner :x (- workspace-x))
     (send mpr :workspace_parameters :min_corner :y (- workspace-y))
     (send mpr :workspace_parameters :min_corner :z (- workspace-z))
     (send mpr :start_state (send scene :robot_state))
     (when goal_constraints
       (if (atom goal_constraints) (setq goal_constraints (list goal_constraints)))
       (send mpr :goal_constraints goal_constraints)
       (if additional_goal_constraints
           (send mpr :goal_constraints
                 (merge-goal-constraints (flatten (list (send mpr :goal_constraints)
                                                        additional_goal_constraints)))))
       )
     (when extra_goal_constraints
       (if (atom extra_goal_constraints)
           (setq extra_goal_constraints (list extra_goal_constraints)))
       (nconc (send mpr :goal_constraints) extra_constraints))
     (if path_constraints (send mpr :path_constraints path_constraints))
     (if trajectory_constraints (send mpr :trajectory_constraints trajectory_constraints))
     (send mpr :planner_id planner-id) ;; select from :query-planner-interface
     (send mpr :group_name group-name)
     (send mpr :num_planning_attempts planning-attempts)
     (send mpr :allowed_planning_time planning-time)
     (setq res
           (ros::service-call planning-service
                              (instance moveit_msgs::GetMotionPlanRequest
                                        :init :motion_plan_request mpr)))
     (cond
      ((not (= (send res :motion_plan_response :error_code :val) 1)) ;; success
       (warn ";; motion plan error at ~A~%"
             (assoc (send res :motion_plan_response :error_code :val)
                    *moveit-error-code-list*))))
     res ;;(send res :motion_plan_response)
     ))
  )

#|

#f(7.60254 55.1074 -67.8955 101.667 -24.917 -8.78906 -10.9424 16.9629 37.5073 66.687 28.3887 73.3008 1.12061)
