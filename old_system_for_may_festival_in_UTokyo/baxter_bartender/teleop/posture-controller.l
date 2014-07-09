#!/usr/bin/env roseus

#-:jsk (jsk)
#-:rbrain-basic (rbrain)

(ros::roseus "baxter_posture_controller")
(ros::roseus-add-msgs "sensor_msgs")
(ros::roseus-add-msgs "geometry_msgs")

(ros::advertise
 "/robot/head/command/posestamped"
 geometry_msgs::posestamped 1)
(ros::advertise
 "/baxter/rarm/endcoords/posestamped"
 geometry_msgs::posestamped 1)
(ros::advertise
 "/baxter/larm/endcoords/posestamped"
 geometry_msgs::posestamped 1)

(require "package://baxter_bartender/euslisp/turtlebot.l")
(require "package://baxtereus/baxter.l")
(require "package://baxter_bartender/euslisp/baxter-util.l")
(require "package://baxter_bartender/euslisp/baxter_moveit.l")
(require "package://baxter_bartender/euslisp/model/table-circle.l")
(require "package://baxter_bartender/euslisp/model/table-audience.l")
(require "package://baxter_bartender/euslisp/baxter-bar-scene.l")
(defvar *touch-it-context* "__require__")
(require "package://baxter_bartender/euslisp/table-touch-it.l")
(require "turtlebot-posestamped.l")
(require "spline-interpole.l")
(require "angle-vector-logger.l")

(defvar *co* (instance collision-object-publisher :init))
(setq *me* (instance baxter-moveit-environment :init))
(defvar *table-circle* (table-circle))
(defvar *table-audience* (table-audience))
(defvar *turtlebot* (turtlebot))

(defvar *updated* nil)
(defvar *updated-small* nil)
(defvar *update-table* nil)

(pickview :no-menu t)
(objects (list *robot* *turtlebot* *table-audience* *table-circle*))

(defvar *world-transformation*
  (make-coords :pos #F(-100 -550 0)
	       :rpy (list (deg2rad -30) 0 0)))
(let* ((tf (send
	    (send (car (send *robot* :links)) :copy-worldcoords)
	    :transformation
	    *world-transformation*)))
  (send *table-circle* :transform tf :world)
  (send *table-audience* :transform tf :world)
  )
;; (send *robot* :init-pose)
;; (model2real)

(defun ufo-ik
  (arm coords
       &rest args
       &key (relative? t) (debug-view nil)
       &allow-other-keys)
  (send* *robot* :inverse-kinematics
	 coords
	 :move-target (send *robot* arm :end-coords)
	 :link-list
	 (send *robot* :link-list
	       (send *robot* arm :end-coords :parent))
	 :rotation-axis :z
	 :thre 30
	 :rthre (deg2rad 10)
	 :stop 30
	 :debug-view debug-view
	 args
	 ))

(defun hook-ik
  (arm coords
       &rest args
       &key (relative? t) (debug-view nil)
       &allow-other-keys
       )
  (send* *robot* :inverse-kinematics
	 coords
	 :move-target (send *robot* arm :end-coords)
	 :link-list
	 (send *robot* :link-list
	       (send *robot* arm :end-coords :parent))
	 :rotation-axis :x
	 :thre 30
	 :rthre (deg2rad 10)
	 :stop 30
	 :debug-view debug-view
	 args
	 ))

(defun table-ik
  (arm coords
       &rest args
       &key (relative? t) (debug-view nil)
       &allow-other-keys)
  (send* *robot* :inverse-kinematics
	 (print
	  (make-coords
	   :pos (v+
		 (if relative?
		     (send *robot* arm :end-coords :worldpos)
		   (float-vector 0 0 0))
		 (send coords :worldpos))
	   :rpy (if (eq arm :rarm)
		    (list (deg2rad 180) (deg2rad 90) (deg2rad 90))
		  (list (deg2rad -180) (deg2rad 90) (deg2rad -90)))))
	 :move-target (send *robot* arm :end-coords)
	 :link-list
	 (send *robot* :link-list
	       (send *robot* arm :end-coords :parent))
	 :rotation-axis :x
	 :debug-view debug-view
	 args
	 ))

(defun normal-ik
  (arm coords
       &rest args
       &key
       (relative? t)
       (rotation-axis t)
       (stop 30)
       (debug-view nil)
       &allow-other-keys)
  (send* *robot* :inverse-kinematics
	 coords
	 :move-target (send *robot* arm :end-coords)
	 :link-list
	 (send *robot* :link-list
	       (send *robot* arm :end-coords :parent))
	 :rotation-axis rotation-axis
	 :thre 30
	 :rthre (deg2rad 10)
	 :stop stop
	 :debug-view debug-view
	 args
	 ))

(defun string-command-callback
  (msg)
  (let* ((com (send msg :data))
	 (parsed (if (and com (plusp (length com)))
		     (read-from-string com))))
    (cond
     ((null com)
      (format t "[string-command-callback] null command~%"))
     ((and (listp parsed)
	   (functionp (car parsed)))
      (format t "[string-command-callback] command ~A~%" parsed)
      (setq *updated-small* t)
      (eval parsed))
     ))
  )

(defun abs-joint-state-command-callback
  (msg)
  (let ((names (send msg :name))
	(positions (send msg :position))
	(velocitys (send msg :velocity))
	key j
	(link-list
	 (remove-if #'(lambda (l) (not (send l :joint))) (send *robot* :links))))
    (cond
     ((not (eq (length positions) (length names)))
      (format t "[joint-state-callback] position skip~%")
      (setq positions (make-list (length names))))
     ((not (eq (length velocitys) (length names)))
      (format t "[joint-state-callback] velocity skip~%")
      (setq velocitys (make-list (length names)))))
    (map cons
	 #'(lambda (n pos vel)
	     (format t "[joint-state-callback] ~A ~A ~A~%" n pos vel)
	     (cond
	      ((setq j
		     (find-if #'(lambda (j)
				  (substringp n (format nil "~A" (send j :name))))
			      (send-all link-list :joint)))
	       (setq link-list (remove (send j :child-link) link-list))
	       (send j :joint-angle
		     (cond
		      (pos (rad2deg pos))
		      (vel (+ (send j :joint-angle)
			      (rad2deg vel)))
		      (t (send j :joint-angle))))
	       (setq *updated* :both))
	      ((setq j
		     (find-if #'(lambda (j)
				  (substringp n (format nil "~A" (send j :name))))
			      link-list))
	       (setq link-list (remove j link-list))
	       (setq j (send j :joint))
	       (send j :joint-angle
		     (cond
		      (pos (rad2deg pos))
		      (vel (+ (send j :joint-angle)
			      (rad2deg vel)))
		      (t (send j :joint-angle))))
	       (setq *updated* :both)
	       )))
	 names positions velocitys)
    ))

(defun limb-posestamped-callback
  (msg
   &key
   (limb :rarm)
   (tag :normal)
   (flag :absolute)
   (target-coords (send *robot* limb :end-coords :copy-worldcoords))
   (newcoords
    (make-coords
     :pos (scale 1e+3 (float-vector (send (send msg :pose) :position :x)
				    (send (send msg :pose) :position :y)
				    (send (send msg :pose) :position :z)))
     :rot (quaternion2matrix
	   (float-vector
	    (send (send msg :pose) :orientation :w)
	    (send (send msg :pose) :orientation :x)
	    (send (send msg :pose) :orientation :y)
	    (send (send msg :pose) :orientation :z))))))
  (format t "[posestamped-callback] ~A ~A ~A ~A~%" limb tag flag newcoords)
  (cond
   ((eq flag :absolute)
    (setq *updated* limb)
    (send
     target-coords
     :newcoords
     (send (send (car (send *robot* :links)) :copy-worldcoords)
	   :transform newcoords :local)))
   ((eq flag :relative)
    (setq *updated-small* t)
    (send target-coords :transform newcoords :local))
   ((eq flag :relative2)
    (setq *updated-small* t)
    (send target-coords
	  :newcoords
	  (make-coords
	   :pos
	   (v+ (send target-coords :worldpos)
	       (send newcoords :worldpos))
	   :rot
	   (m*
	    (send newcoords :worldrot)
	    (send target-coords :worldrot)))))
   ((eq flag :table)
    (setq *update-table* limb)
    (send
     target-coords
     :newcoords
     (send (send (car (send *robot* :links)) :copy-worldcoords)
	   :transform newcoords :local)))
   )
  (cond
   ((not
     (cond
      ((eq tag :ufo) (ufo-ik limb target-coords :relative? nil :debug-view :no-message))
      ((eq tag :oc) (hook-ik limb target-coords :relative? nil :debug-view :no-message :revert-if-fail (if *update-table* nil t)))
      ((eq tag :table) (table-ik limb target-coords :relative? nil :debug-view :no-message :revert-if-fail (if *update-table* nil t)))
      ((eq tag :rot) (normal-ik limb target-coords :relative? nil :rotation-axis nil :debug-view :no-message))
      ((eq tag :force) (normal-ik limb target-coords :relative? nil :revert-if-fail nil :debug-view :no-message))
      (t (normal-ik limb target-coords :relative? nil :debug-view :no-message))))
    (setq *updated-small* nil)
    (setq *updated* nil)))
  )

(ros::subscribe
 "/abs_posture_controller/command/jointstate"
 sensor_msgs::JointState
 #'abs-joint-state-command-callback)

(ros::subscribe
 "/posture_controller/command/string"
 std_msgs::string
 #'string-command-callback 1)


(defun relative-rarm-posestamped-callback (msg) (limb-posestamped-callback msg :limb :rarm :tag :normal :flag :relative))
(defun relative2-rarm-posestamped-callback (msg) (limb-posestamped-callback msg :limb :rarm :tag :normal :flag :relative2))
(defun absolute-rarm-posestamped-callback (msg) (limb-posestamped-callback msg :limb :rarm :tag :normal :flag :absolute))
(defun relative-larm-posestamped-callback (msg) (limb-posestamped-callback msg :limb :larm :tag :normal :flag :relative))
(defun relative2-larm-posestamped-callback (msg) (limb-posestamped-callback msg :limb :larm :tag :normal :flag :relative2))
(defun absolute-larm-posestamped-callback (msg) (limb-posestamped-callback msg :limb :larm :tag :normal :flag :absolute))


(ros::subscribe
 "/rel_posture_controller/command/posestamped/rarm"
 geometry_msgs::posestamped
 #'relative-rarm-posestamped-callback 1)
(ros::subscribe
 "/rel2_posture_controller/command/posestamped/rarm"
 geometry_msgs::posestamped
 #'relative2-rarm-posestamped-callback 1)
(ros::subscribe
 "/abs_posture_controller/command/posestamped/rarm"
 geometry_msgs::posestamped
 #'absolute-rarm-posestamped-callback 1)
(ros::subscribe
 "/rel_posture_controller/command/posestamped/larm"
 geometry_msgs::posestamped
 #'relative-larm-posestamped-callback 1)
(ros::subscribe
 "/rel2_posture_controller/command/posestamped/larm"
 geometry_msgs::posestamped
 #'relative2-larm-posestamped-callback 1)
(ros::subscribe
 "/abs_posture_controller/command/posestamped/larm"
 geometry_msgs::posestamped
 #'absolute-larm-posestamped-callback 1)


(defun relative-rarm-ufo-posestamped-callback (msg) (limb-posestamped-callback msg :limb :rarm :tag :ufo :flag :relative))
(defun relative2-rarm-ufo-posestamped-callback (msg) (limb-posestamped-callback msg :limb :rarm :tag :ufo :flag :relative2))
(defun absolute-rarm-ufo-posestamped-callback (msg) (limb-posestamped-callback msg :limb :rarm :tag :ufo :flag :absolute))
(defun relative-larm-ufo-posestamped-callback (msg) (limb-posestamped-callback msg :limb :larm :tag :ufo :flag :relative))
(defun relative2-larm-ufo-posestamped-callback (msg) (limb-posestamped-callback msg :limb :larm :tag :ufo :flag :relative2))
(defun absolute-larm-ufo-posestamped-callback (msg) (limb-posestamped-callback msg :limb :larm :tag :ufo :flag :absolute))

(ros::subscribe
 "/rel_posture_controller/command/posestamped/rarm/ufo"
 geometry_msgs::posestamped
 #'relative-rarm-ufo-posestamped-callback 1)
(ros::subscribe
 "/rel2_posture_controller/command/posestamped/rarm/ufo"
 geometry_msgs::posestamped
 #'relative2-rarm-ufo-posestamped-callback 1)
(ros::subscribe
 "/abs_posture_controller/command/posestamped/rarm/ufo"
 geometry_msgs::posestamped
 #'absolute-rarm-ufo-posestamped-callback 1)
(ros::subscribe
 "/rel_posture_controller/command/posestamped/larm/ufo"
 geometry_msgs::posestamped
 #'relative-larm-ufo-posestamped-callback 1)
(ros::subscribe
 "/rel2_posture_controller/command/posestamped/larm/ufo"
 geometry_msgs::posestamped
 #'relative2-larm-ufo-posestamped-callback 1)
(ros::subscribe
 "/abs_posture_controller/command/posestamped/larm/ufo"
 geometry_msgs::posestamped
 #'absolute-larm-ufo-posestamped-callback 1)


(defun relative-rarm-oc-posestamped-callback (msg) (limb-posestamped-callback msg :limb :rarm :tag :oc :flag :relative))
(defun relative2-rarm-oc-posestamped-callback (msg) (limb-posestamped-callback msg :limb :rarm :tag :oc :flag :relative2))
(defun absolute-rarm-oc-posestamped-callback (msg) (limb-posestamped-callback msg :limb :rarm :tag :oc :flag :absolute))
(defun relative-larm-oc-posestamped-callback (msg) (limb-posestamped-callback msg :limb :larm :tag :oc :flag :relative))
(defun relative2-larm-oc-posestamped-callback (msg) (limb-posestamped-callback msg :limb :larm :tag :oc :flag :relative2))
(defun absolute-larm-oc-posestamped-callback (msg) (limb-posestamped-callback msg :limb :larm :tag :oc :flag :absolute))


(ros::subscribe
 "/rel_posture_controller/command/posestamped/rarm/oc"
 geometry_msgs::posestamped
 #'relative-rarm-oc-posestamped-callback 1)
(ros::subscribe
 "/rel2_posture_controller/command/posestamped/rarm/oc"
 geometry_msgs::posestamped
 #'relative2-rarm-oc-posestamped-callback 1)
(ros::subscribe
 "/abs_posture_controller/command/posestamped/rarm/oc"
 geometry_msgs::posestamped
 #'absolute-rarm-oc-posestamped-callback 1)
(ros::subscribe
 "/rel_posture_controller/command/posestamped/larm/oc"
 geometry_msgs::posestamped
 #'relative-larm-oc-posestamped-callback 1)
(ros::subscribe
 "/rel2_posture_controller/command/posestamped/larm/oc"
 geometry_msgs::posestamped
 #'relative2-larm-oc-posestamped-callback 1)
(ros::subscribe
 "/abs_posture_controller/command/posestamped/larm/oc"
 geometry_msgs::posestamped
 #'absolute-larm-oc-posestamped-callback 1)


(defun relative-rarm-rot-posestamped-callback (msg) (limb-posestamped-callback msg :limb :rarm :tag :rot :flag :relative))
(defun relative2-rarm-rot-posestamped-callback (msg) (limb-posestamped-callback msg :limb :rarm :tag :rot :flag :relative2))
(defun absolute-rarm-rot-posestamped-callback (msg) (limb-posestamped-callback msg :limb :rarm :tag :rot :flag :absolute))
(defun relative-larm-rot-posestamped-callback (msg) (limb-posestamped-callback msg :limb :larm :tag :rot :flag :relative))
(defun relative2-larm-rot-posestamped-callback (msg) (limb-posestamped-callback msg :limb :larm :tag :rot :flag :relative2))
(defun absolute-larm-rot-posestamped-callback (msg) (limb-posestamped-callback msg :limb :larm :tag :rot :flag :absolute))

(ros::subscribe
 "/rel_posture_controller/command/posestamped/rarm/rot"
 geometry_msgs::posestamped
 #'relative-rarm-rot-posestamped-callback 1)
(ros::subscribe
 "/rel2_posture_controller/command/posestamped/rarm/rot"
 geometry_msgs::posestamped
 #'relative2-rarm-rot-posestamped-callback 1)
(ros::subscribe
 "/abs_posture_controller/command/posestamped/rarm/rot"
 geometry_msgs::posestamped
 #'absolute-rarm-rot-posestamped-callback 1)
(ros::subscribe
 "/rel_posture_controller/command/posestamped/larm/rot"
 geometry_msgs::posestamped
 #'relative-larm-rot-posestamped-callback 1)
(ros::subscribe
 "/rel2_posture_controller/command/posestamped/larm/rot"
 geometry_msgs::posestamped
 #'relative2-larm-rot-posestamped-callback 1)
(ros::subscribe
 "/abs_posture_controller/command/posestamped/larm/rot"
 geometry_msgs::posestamped
 #'absolute-larm-rot-posestamped-callback 1)


(defun relative-rarm-force-posestamped-callback (msg) (limb-posestamped-callback msg :limb :rarm :tag :force :flag :relative))
(defun relative2-rarm-force-posestamped-callback (msg) (limb-posestamped-callback msg :limb :rarm :tag :force :flag :relative2))
(defun absolute-rarm-force-posestamped-callback (msg) (limb-posestamped-callback msg :limb :rarm :tag :force :flag :absolute))
(defun relative-larm-force-posestamped-callback (msg) (limb-posestamped-callback msg :limb :larm :tag :force :flag :relative))
(defun relative2-larm-force-posestamped-callback (msg) (limb-posestamped-callback msg :limb :larm :tag :force :flag :relative2))
(defun absolute-larm-force-posestamped-callback (msg) (limb-posestamped-callback msg :limb :larm :tag :force :flag :absolute))


(ros::subscribe
 "/rel_posture_controller/command/posestamped/rarm/force"
 geometry_msgs::posestamped
 #'relative-rarm-force-posestamped-callback 1)
(ros::subscribe
 "/rel2_posture_controller/command/posestamped/rarm/force"
 geometry_msgs::posestamped
 #'relative2-rarm-force-posestamped-callback 1)
(ros::subscribe
 "/abs_posture_controller/command/posestamped/rarm/force"
 geometry_msgs::posestamped
 #'absolute-rarm-force-posestamped-callback 1)
(ros::subscribe
 "/rel_posture_controller/command/posestamped/larm/force"
 geometry_msgs::posestamped
 #'relative-larm-force-posestamped-callback 1)
(ros::subscribe
 "/rel2_posture_controller/command/posestamped/larm/force"
 geometry_msgs::posestamped
 #'relative2-larm-force-posestamped-callback 1)
(ros::subscribe
 "/abs_posture_controller/command/posestamped/larm/force"
 geometry_msgs::posestamped
 #'absolute-larm-force-posestamped-callback 1)

(defun absolute-rarm-table-posestamped-callback (msg) (limb-posestamped-callback msg :limb :rarm :tag :table :flag :table))
(defun absolute-larm-table-posestamped-callback (msg) (limb-posestamped-callback msg :limb :larm :tag :table :flag :table))

(ros::subscribe
 "/table_posture_controller/command/posestamped/larm"
 geometry_msgs::posestamped
 #'absolute-larm-table-posestamped-callback 1)
(ros::subscribe
 "/table_posture_controller/command/posestamped/rarm"
 geometry_msgs::posestamped
 #'absolute-rarm-table-posestamped-callback 1)

(defun absolute-table-string-callback (arm msg)
  (let ((target-coords
	 (find-if
	  #'(lambda (a) (string-equal (send a :name)
				      (send msg :data)))
	  (send *table-audience* :get-val 'audience-list))))
    (cond
     (target-coords
      (limb-posestamped-callback
       nil :limb arm
       :newcoords
       (send
	(send (car (send *robot* :links)) :copy-worldcoords)
	:transformation
	(send
	 (send target-coords :copy-worldcoords)
	 :translate #F(0 0 150)))
       :tag :table :flag :table)))))

(defun absolute-larm-table-string-callback (msg) (absolute-table-string-callback :larm msg))
(defun absolute-rarm-table-string-callback (msg) (absolute-table-string-callback :rarm msg))

(ros::subscribe
 "/table_posture_controller/command/string/rarm"
 std_msgs::string
 #'absolute-rarm-table-string-callback 1)
(ros::subscribe
 "/table_posture_controller/command/string/larm"
 std_msgs::string
 #'absolute-larm-table-string-callback 1)


(defun turtlebot-empty-callback
  (arm msg &optional (target-coords *turtlebot-coords*))
  (cond
   (target-coords
    (limb-posestamped-callback
     nil :limb arm
     :newcoords
     (send
      (send (car (send *robot* :links)) :copy-worldcoords)
      :transformation
      (send
       (send target-coords :copy-worldcoords)
       :translate #F(0 0 150)))
     :tag :table :flag :table))))

(defun turtlebot-larm-table-empty-callback (msg) (turtlebot-empty-callback :larm msg))
(defun turtlebot-rarm-table-empty-callback (msg) (turtlebot-empty-callback :rarm msg))
(defun turtlebot-larm-table-default-empty-callback (msg) (turtlebot-empty-callback :larm msg *turtlebot-default-coords*))
(defun turtlebot-rarm-table-default-empty-callback (msg) (turtlebot-empty-callback :rarm msg *turtlebot-default-coords*))

(ros::subscribe
 "/turtlebot_posture_controller/command/empty/larm"
 std_msgs::Empty
 #'turtlebot-larm-table-empty-callback 1)
(ros::subscribe
 "/turtlebot_posture_controller/command/empty/rarm"
 std_msgs::Empty
 #'turtlebot-larm-table-empty-callback 1)
(ros::subscribe
 "/turtlebot_posture_controller/command/default/empty/larm"
 std_msgs::Empty
 #'turtlebot-larm-table-default-empty-callback 1)
(ros::subscribe
 "/turtlebot_posture_controller/command/default/empty/rarm"
 std_msgs::Empty
 #'turtlebot-larm-table-default-empty-callback 1)

(ros::advertise "/voice_echo/mei" std_msgs::string 1)
(defun my-talk (str)
  (ros::publish "/voice_echo/mei"
		(instance std_msgs::string :init :data str)))


(my-talk "起動します")
(unsubscribe-point-cloud)
(ros::rate 3)
(do-until-key
 (if (not (ros::ok)) (return-from nil nil))
 (real2model)
 (ros::spin-once)
 (turtlebot-posestamped-publish)
 (publish-posestamped
  (send (send (car (send *robot* :links)) :copy-worldcoords)
	:transformation
	(send *robot* :rarm :end-coords :copy-worldcoords)
	:local)
  :topic "/baxter/rarm/endcoords/posestamped")
 (publish-posestamped
  (send (send (car (send *robot* :links)) :copy-worldcoords)
	:transformation
	(send *robot* :larm :end-coords :copy-worldcoords)
	:local)
  :topic "/baxter/larm/endcoords/posestamped")
 ;;(print *turtlebot-coords*)
 (send *turtlebot* :newcoords (copy-object *turtlebot-coords*))
 (send *viewer* :draw-objects)
 (x::window-main-one)
 (cond
  (*updated*
   (format t "[posture-controller] update abs posture~%")
   (my-talk "動きます")
   ;;(model2real :wait t)
   (moveit-interpole :end (send *robot* :angle-vector)
		     :real? #'gui-yes-or-no-p
		     :draw? t
		     :arms (list
			    (or (find *updated*
				      '(:rarm :larm :both))
				:both)))
   (unix:sleep 1)
   (send *ri* :wait-interpolation)
   (setq *updated* nil))
  (*update-table*
   (format t "[posture-controller] update table posture~%")
   (my-talk "テーブルに手を伸ばします。気をつけてください。")
   (reach-audience-table
    :arm *update-table*
    :init-position (Copy-Object (send *ri* :state :potentio-vector))
    :counter-reaching-coords (send *robot* *update-table* :end-coords :copy-worldcoords)
    :real? #'gui-yes-or-no-p
    :interpole-time 6000
    :animate t
    )
   (unix:sleep 1)
   (send *ri* :wait-interpolation)
   (setq *update-table* nil))
  (*updated-small*
   (format t "[posture-controller] update rel posture~%")
   (model2real :wait t :sleep 3000)
   (setq *updated-small* nil))
  (t
   ))
 (step-log (Copy-Object (send *robot* :angle-vector)))
 (ros::sleep))


#|

:can-bringup-pose
#f(57.4805 -33.8379 -72.4219 100.305 142.405 -67.1045 -41.2207 -68.291 56.8433 145.151 -39.0894 27.2681 -64.7314)

:can-reaching-pose
#f(-9.36035 -8.63525 49.8999 16.9849 -31.6626 30.2783 -41.2427 -68.291 56.8433 145.107 -39.1113 27.312 -64.7314)



