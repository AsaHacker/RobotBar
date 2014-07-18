
(ros::roseus "look_face_node")
(ros::load-ros-manifest "std_msgs")

(defvar *mirror-flag* -1)
(defvar *ros-rate* 3)
(defvar *detect-cnt* (* 2 *ros-rate*))

(defun face-pos-callback
  (msg)
  (let* ((org (send msg :data))
	 (center-org
	  (float-vector
	   (+ (aref org 0) (* 0.5 (aref org 2)))
	   (+ (aref org 1) (* 0.5 (aref org 3)))))
	 (pos (float-vector
	       (* *mirror-flag* 2.0 (- (aref center-org 0) 0.5))
	       (* -2.0 (- (aref center-org 1) 0.5)))))
    ;; (ros::publish
    ;;  "/pad_face_controller/emotion_vector/relative"
    ;;  (instance std_msgs::float64multiarray
    ;; 	       :init
    ;; 	       :data #F(0.4 0.4 -0.4)))
    (setq *detect-cnt* (* 3 *ros-rate*))
    (face-lookat pos :relative? t)
    ))

(defun face-lookat
  (pos &key (relative? t))
  (format t "[look_face] look at ~A~%" pos)
  (ros::publish
   (if relative? "/kubi_sample/pan_tilt_vector/relative"
     "/kubi_sample/pan_tilt_vector")
   (instance std_msgs::float32multiarray
	     :init
	     :data (scale 5 pos)))
  (ros::publish
   "/pad_face_controller/lookpos_vector"
   (instance std_msgs::float64multiarray
	     :init
	     :data pos))
  )

(defun random-lookat
  nil
  (face-lookat (subseq (random-vector 1.0) 0 2) :relative? t))

(ros::advertise
 "/pad_face_controller/lookpos_vector"
 std_msgs::float64multiarray)

(ros::advertise
 "/kubi_sample/pan_tilt_vector"
 std_msgs::float32multiarray)

(ros::advertise
 "/kubi_sample/pan_tilt_vector/relative"
 std_msgs::float32multiarray)

(ros::advertise
 "/pad_face_controller/emotion_vector/relative"
 std_msgs::float64multiarray)

(ros::subscribe
 "/face_detector/normalized_rect"
 std_msgs::float32multiarray
 #'face-pos-callback 1)

(ros::rate *ros-rate*)
(do-until-key
 (if (not (ros::ok)) (return-from nil nil))
 (print 'running)
 (ros::spin-once)
 (decf *detect-cnt*)
 (if (minusp *detect-cnt*) (random-lookat))
 (ros::sleep)
 )
