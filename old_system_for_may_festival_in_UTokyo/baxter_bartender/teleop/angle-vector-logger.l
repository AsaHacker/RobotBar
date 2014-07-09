#!/usr/bin/env roseus

;; (ros::roseus "angle_vector_logger")

(defun log-surfix
  nil
  (let ((ret
         (map string
              #'(lambda (a) (if (= a #\ ) #\_ a))
              (unix:asctime (unix:localtime)))))
    (remove-if #'(lambda (c) (find c (list #\: #\_ #\ )))
	       (subseq ret 0 (- (length ret) 1)))))

(defvar *av-seq* nil)
(defvar *logging* nil)
(defvar *time-buf*)
(defvar *mtimer*)

(defun step-log
  (av)
  (cond
   (*logging*
    (format t "[av-logger] logging ... ~%")
    (if (null *mtimer*)
	(setq *mtimer* (instance mtimer :init)))
    (push (cons (* 1e+3 (send *mtimer* :stop)) av) *av-seq*)
    (setq *mtimer* (instance mtimer :init)))))

(defvar *log-root*
  (format nil
	  "~A/teleop/log"
	  (read-line
	   (piped-fork "rospack find baxter_bartender"))))

(defun play-log
  (path &key
	(whole-path (format nil "~A/~A" *log-root* path))
	(av-list
	 (if (probe-file whole-path)
	     (progn
	       (load whole-path)
	       *av-seq*)))
	;;(callback #'gui-yes-or-no-p)
	)
  (print av-list)
  (cond
   (av-list
    (mapcar #'(lambda (a)
		(send *robot* :angle-vector (cdr a))
		(send *viewer* :draw-objects)
		(x::window-main-one)
		(unix:usleep (round (* 1000 100))))
	    av-list)
    (if (gui-yes-or-no-p)
    ;;(or (not (functionp callback)) (funcall callback))
	(send *ri* :angle-vector-sequence
	      (mapcar #'cdr av-list)
	      (mapcar #'car av-list))))))

(defun angle-vector-logger-start
  (msg)
  (format t "[av-logger] start logger~%")
  (setq *av-seq* nil)
  (setq *logging* t))

(defun angle-vector-logger-stop
  (msg)
  (format t "[av-logger] stop logger save file to ")
  (setq *av-seq* (reverse *av-seq*))
  (dump-loadable-structure
   (print (format nil "log/~A.log" (log-surfix)))
   *av-seq*)
  (setq *mtimer* nil)
  (setq *logging* nil))

(defun angle-vector-logger-play
  (msg)
  (play-log (send msg :data)))

(ros::subscribe
 "/angle_vector_logger/command/start"
 std_msgs::Empty
 #'angle-vector-logger-start 1)

(ros::subscribe
 "/angle_vector_logger/command/stop"
 std_msgs::Empty
 #'angle-vector-logger-stop 1)

(ros::subscribe
 "/angle_vector_logger/command/log_player"
 std_msgs::string
 #'angle-vector-logger-play 1)

