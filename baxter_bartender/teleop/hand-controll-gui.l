#!/usr/bin/env roseus

(ros::roseus "hand_controller_gui")
(ros::roseus-add-msgs "sensor_msgs")
(ros::roseus-add-msgs "geometry_msgs")

(ros::advertise
 "/posture_controller/command/string"
 std_msgs::string)

(defvar *controlled-limb* :rarm)

(defun get-choice-callback
  (tag obj &rest args)
  ;; overwrite here
  (let ((cm (if (and (stringp tag) (plusp (length tag)))
		(read-from-string tag) :nop)))
    (format t "~A() -> ~A~%" obj args cm)
    (case cm
	  (:rarm (setq *controlled-limb* cm))
	  (:larm (setq *controlled-limb* cm))
	  (:open
	   (ros::publish
	    "/posture_controller/command/string"
	    (instance std_msgs::string
		      :init
		      :data
		      (format nil "(send *ri* :stop-grasp ~A)" *controlled-limb*))))
	  (:close
	   (ros::publish
	    "/posture_controller/command/string"
	    (instance std_msgs::string
		      :init
		      :data
		      (format nil "(send *ri* :start-grasp ~A)" *controlled-limb*))))))
  )

(defun get-button-callback
  (obj &rest args)
  (ros::publish
   "/posture_controller/command/string"
   (instance std_msgs::string
	     :init
	     :data
	     (format nil "(send *ri* :hand-calibrate)")))
  )

;; create window to set joint-angle
(defclass choice-sample
  :super x::panel
  :slots (radio-list choices-list)
  )

(defmethod choice-sample
  (:create
   (&key
    (name-list (list "" ""))
    ((:choices-list cl)
     (list (list ":rarm" ":larm")
	   (list ":open" ":close")))
    (button-name "CALIB")
    (width (* 100 1))
    (height (+ 40 (* 40 (length name-list))))
    (window-name "choice-sample")
    &rest
    args)
   (send-super* :create
		:title window-name
		:width width :height height
		:font x::font-courb12
		:background #xFEFFFE
		args)
   (setq choices-list cl)
   (send-super :create-item x::button-item
	       button-name self :pre-get-button-value)
   (setq radio-list
	 (mapcar #'(lambda (name choices)
		     (send-super :create-item x::choice-item
				 name self :pre-get-choice-value
				 :choices choices))
		 name-list choices-list))
   self)
  (:pre-get-choice-value
   (obj &rest args) ;; bug?
   (get-choice-callback (send self :get-tag-from-value obj) obj args))
  (:pre-get-button-value
   (obj &rest args) ;; bug?
   (get-button-callback obj args))
  (:get-tag-from-value
   (obj)
   (nth (send obj :value)
	(cdr (assoc obj (mapcar #'cons radio-list choices-list)))))
  (:nomethod
   (&rest args)
   (let (sym val)
     (cond
      ((keywordp (car args))
       (setq sym (read-from-string (send (car args) :pname)))
       (setq val (assoc sym (send self :slots)))))
     (cond
      ((or (null sym) (null val)) nil)
      ((> (length args) 1)
       (eval (list 'setq sym '(cadr args))))
      (t (cdr val)))))
  )

(defvar *hand-controll-gui* (instance choice-sample :create))
(send *hand-controll-gui* :redraw)
(send *hand-controll-gui* :flush)


#|

(setq a (instance choice-sample :create))
(send a :flush)
