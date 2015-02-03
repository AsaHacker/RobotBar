
#-:jsk (jsk)
#-:rbrain-basic (rbrain)

(ros::roseus "talk_sample_node")
(ros::load-ros-manifest "std_msgs")
(ros::load-ros-manifest "jsk_gui_msgs")

;; usage
;; string, funciton, list of them
(setq *response-key-pairs*
  (list
   (list :hello-hello
	 (list "こんにちわ|ハロー|良い天気"
	       "こんにちわ|ハロー|良い天気")
	 "ご丁寧にどうも")
   (list :hello
	 "こんにちわ|ハロー|良い天気"
	 (list "こんにちわ" "ご機嫌うるわしゅう"))
   (list :duplicate
	 (list #'(lambda (msg) (setq hoge msg))
	       #'(lambda (msg) (string-equal hoge msg)))
	 (list "同じことを言わないでください" "私のことをバカだと思っていますね？"))
   (list :weather
	 ".*天気.*"
	 (list "今日の天気は晴れです" "今日の天気は雨です"))
   (list :debug
	 "調子"
	 #'(lambda (msg) "調子良好"))
   (list :name
	 "名前"
	 "私の名前は名無しです")
   ;;
   (list :start-ros
	 "起動|スタート|発進"
	 "ROSを起動しますか？")
   (list :start-ros-yes
	 (list "はい|イエス|オーケー" "起動|スタート|発進")
	 #'(lambda (msg)
	     ;; implement here
	     "ROSを起動します"))
   (list :start-ros-no
	 (list "いいえ|ノー|止める" "起動|スタート|発進")
	 "ROSの起動をキャンセルしました")
   ;;
   (list :echo #'(lambda (&rest args) t) #'caar)
   ))

(defvar *msg-queue* nil)

(defun _voice-message-callback
  (str-list)
  (push str-list *msg-queue*)
  (if (> (length *msg-queue*) 10)
      (setq *msg-queue* (subseq *msg-queue* 0 10)))
  (format t "[talk_sample] input ~A~%" str-list)
  (ros::publish
   "/voice_echo/mei"
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
  (find-if #'(lambda (msg) (reg-match cand msg)) msg-list))

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
		       ;; (print buf)
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
    ;; (print response)
    (setq response (caddr response))
    (while (keywordp response)
      (setq response (caddr (assoc response *response-key-pairs*))))
    (if (null response)
	(format t " invalid response detected~%"))
    (cond
     ((stringp response) response)
     ((functionp response) (funcall response msg-list))
     ((listp response) (nth (random (length response)) response))
     (t "ぬる"))))

(ros::advertise "/voice_echo/mei" std_msgs::string)

(ros::subscribe
 "/voice_echo/mei/request"
 std_msgs::string
 #'string-message-callback 1)

(ros::subscribe
 "/Tablet/voice"
 jsk_gui_msgs::VoiceMessage
 #'voice-message-callback 1)

(ros::rate 3)
(do-until-key
 (if (not (ros::ok)) (return-from nil nil))
 (print 'running)
 (ros::spin-once)
 (ros::sleep)
 )
