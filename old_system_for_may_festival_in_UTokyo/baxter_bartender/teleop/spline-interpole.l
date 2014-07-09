
#-:jsk (jsk)
#-:rbrain-basic (rbrain)

(defun minjerk-coords-interpole
  (&key
   (coords-list (list (make-coords)
		      ;;(make-coords :pos #F(1 0 0))
		      (make-coords :pos #F(0 1 0))))
   (time-list (list 1.0))
   (cnt 30)
   (step (/ (apply #'+ time-list) (* 1.0 cnt)))
   ;;
   (coords-list-check
    (cond
     ((< (length coords-list) 2) 'error)
     ((eq (length coords-list) 2)
      (setq coords-list (list (nth 0 coords-list)
			      (apply #'midcoords (cons 0.5 coords-list))
			      (nth 1 coords-list))))
     (t 'ok)))
   (6dof-list
    (mapcar #'(lambda (c pre_c)
		(concatenate float-vector
			     (send c :worldpos)
			     (v+ (matrix-log (send pre_c :worldrot))
				 (send pre_c :difference-rotation c))))
	    coords-list (cons (make-coords) (cdr coords-list))))
   (spline
    (pos-list-interpolate-spline-minjerk 6dof-list time-list step))
   ;;(list #F(0 0) #F(0 1) #F(1 0)) (list 10) 1.0)
   ret
   )
  (mapcar
   #'(lambda (v)
       (make-coords :pos (subseq v 0 3)
		    :rot (matrix-exponent (subseq v 3 6))))
   (cadr (member :data spline)))
  )

(defun minjerk-coords-interpole-angle-vector
  (&key
   (robot *robot*)
   (coords-list (list (make-coords)
		      (make-coords :pos #F(1 0 0))
		      (make-coords :pos #F(0 1 0))))
   (cnt 15)
   (interpolated-coords-list
    (minjerk-coords-interpole
     :coords-list coords-list :cnt cnt))
   (arm :rarm)
   (move-target (send robot arm :end-coords))
   (link-list (send robot :link-list (send move-target :parent)))
   (animate t)
   (real? nil)
   (interpole-time 3000)
   ret
   )
  (dolist (c interpolated-coords-list)
    (send robot :inverse-kinematics c
	  :move-target move-target
	  :link-list link-list
	  :dif-pos-ratio 0.1
	  :dif-rot-ratio 0.1
	  ;;:debug-view :no-message
	  :stop 7
	  :revert-if-fail nil)
    (cond
     ((and animate (boundp '*viewer*) *viewer*)
      (send *viewer* :draw-objects)
      (unix:usleep (* 100 1000))))
    (push (copy-object (send robot :angle-vector)) ret))
  (cond
   ((and real? (or (and (functionp real?) (funcall real?))
		   (not (functionp real?))))
    (send *ri* :angle-vector-sequence
	  (reverse ret)
	  (make-list (length ret)
		     :initial-element
		     (/ (* 1.0 interpole-time) (length ret))))))
  (reverse ret))

(defun reach-audience-table
  (&key
   (robot *robot*)
   (arm :larm)
   (init-position (copy-object (send robot :angle-vector)))
   (counter-reaching-coords
    (progn
      (send robot :init-pose)
      (copy-object
       (send robot arm :end-coords :copy-worldcoords))))
   (start-coords
    (progn
      (send robot :angle-vector (copy-object init-position))
      (send robot arm :end-coords :copy-worldcoords)))
   (mid-coords
    (let* ((c (midcoords
	       0.5
	       start-coords counter-reaching-coords)))
      (setf (aref (send c :worldpos) 2)
	    (aref (send counter-reaching-coords :worldpos) 2))
      c))
   (animate t)
   (real? nil)
   (interpole-time 3000)
   )
  (minjerk-coords-interpole-angle-vector
   :robot robot
   :coords-list (list start-coords
		      mid-coords
		      counter-reaching-coords)
   :arm arm
   :animate animate
   :real? real?
   :interpole-time interpole-time
   ))

(defun test-spline-interpole
  nil
  (let* ((end (progn
		(send *robot* :init-pose)
		(copy-object (send *robot* :larm :end-coords :copy-worldcoords))))
	 (start (progn
		  (send *robot* :reset-fight-pose)
		  (copy-object (send *robot* :larm :end-coords :copy-worldcoords)))))
    (minjerk-coords-interpole-angle-vector
     :robot *robot*
     :coords-list (list start end)
     :arm :larm)))

#| (progn (send *robot* :reset-fight-pose) (reach-audience-table :real? t))