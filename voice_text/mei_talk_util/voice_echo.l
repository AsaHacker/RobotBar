#!/usr/bin/env roseus

(ros::roseus "voice_echo")
(ros::load-ros-manifest "jsk_gui_msgs")

(defvar *text2wave-jp-path*
  (format nil "~A" (read-line (piped-fork "rospack find mei_talk_util"))))
(defvar *text-cnt* 0)

(defun voice-message-callback
  (msg)
  ;;(print msg)
  (unix:system
   (format nil "ROOT_DIR=~A ~A/text2wave-jp ~A happy /tmp/tmp.wav && aplay /tmp/tmp.wav"
	   *text2wave-jp-path*
	   *text2wave-jp-path*
	   (car (send msg :texts))
	   )))

(defun mei-voice-message-callback
  (msg)
  ;; (setq *text-cnt* (mod *text-cnt* 30))
  (incf *text-cnt*)
  (cond
   ((> *text-cnt* 30)
    (unix:system "rm -rf /tmp/*.wav")
    (setq *text-cnt* 0)))
  (let ((text (remove-if #'(lambda (a) (find a (list #\ ))) (send msg :data))))
    (unix:system
     (format nil "ROOT_DIR=~A ~A/text2wave-jp ~A happy /tmp/~A.wav && aplay /tmp/~A.wav"
	     *text2wave-jp-path*
	     *text2wave-jp-path*
	     (remove-if #'(lambda (a) (find a (list #\ ))) (send msg :data))
	     *text-cnt*
	     *text-cnt*)
     )))

;; (ros::subscribe
;;  "/voice_echo/mei"
;;  jsk_gui_msgs::VoiceMessage
;;  #'voice-message-callback 1)

(ros::subscribe
 "/voice_echo/mei"
 std_msgs::string
 #'mei-voice-message-callback 1)

(ros::rate 3)
(do-until-key
 (if (not (ros::ok)) (return-from nil nil))
 (print 'running)
 (ros::spin-once)
 (ros::sleep)
 )
