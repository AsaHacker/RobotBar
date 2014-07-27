#!/usr/bin/env roseus


;; #-:jsk (jsk)
;; #-:rbrain-basic (rbrain)

(ros::roseus "talk_sample_node")
(ros::load-ros-manifest "std_msgs")
;; (ros::load-ros-manifest "jsk_gui_msgs")

(defvar *response-key-pairs*)
(require "package://simple_function_talker/euslisp/function_talk_data.l")
(require "package://simple_function_talker/euslisp/regix-util.l")

(defvar *msg-queue* nil)
(defvar *msg-queue-length* 100)

(defun _voice-message-callback
  (str-list)
  (push str-list *msg-queue*)
  (if (> (length *msg-queue*) *msg-queue-length*)
      (setq *msg-queue* (subseq *msg-queue* 0 *msg-queue-length*)))
  (format t "[talk_sample] input ~A~%" str-list)
  (ros::publish
   "/function_talker/response/string"
   (instance
    std_msgs::string
    :init
    :data
    (print (msg-list->response *msg-queue*))))
  )

(defun voice-message-callback
  (msg) (_voice-message-callback (send msg :texts)))

(defun string-message-callback
  (msg) (_voice-message-callback (list (send msg :data))))

(defun string-matcher
  (cand msg-list)
  (format t "[string-matcher] ~A vs ~A~%" cand msg-list)
  (find-if #'(lambda (msg) (regix-match-check cand msg)) msg-list))

(defun function-matcher
  (cand msg-list)
  (format t "[function-matcher] ~A vs ~A~%" cand msg-list)
  (find-if cand msg-list))

(defun msg-list->response
  (msg-list)
  (let* (buf ret
	 (response
	  (find-if #'(lambda (cand-list)
		       (setq buf (cadr cand-list))
		       (if (or (atom buf) (functionp buf)) (setq buf (list buf)))
		       (and
			(>= (length msg-list) (length buf))
			(null
			 (remove-if
			  #'identity
			  (mapcar
			   #'(lambda (cand msg)
			       (cond
				((stringp cand) (string-matcher cand msg))
				((functionp cand) (function-matcher cand msg))
				(t nil)))
			   buf msg-list)))))
		   *response-key-pairs*))
	 )
    (setq response (caddr response))
    (while (keywordp response)
      (setq response (caddr (assoc response *response-key-pairs*))))
    (if (null response)
	(format t " invalid response detected~%"))
    (cond
     ((stringp response) response)
     ((functionp response)
      (setq response (funcall response msg-list))
      (if (listp response) (nth (random (length response)) response)
	response))
     ((listp response) (nth (random (length response)) response))
     (t "ぬる"))))

(ros::advertise "/function_talker/response/string" std_msgs::string)

(ros::subscribe
 "/function_talker/request/string"
 std_msgs::string
 #'string-message-callback 1)

;; (ros::subscribe
;;  "/Tablet/voice"
;;  jsk_gui_msgs::VoiceMessage
;;  #'voice-message-callback 1)

(ros::rate 3)
(do-until-key
 (if (not (ros::ok)) (return-from nil nil))
 (print 'running)
 (ros::spin-once)
 (ros::sleep)
 )
