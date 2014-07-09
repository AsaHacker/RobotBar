(defclass table-circle
  :super cascaded-link
  :slots (root table-prism virtual-joint))

(defmethod table-circle
  (:init
   (&rest
    args
    &key
    (outer-radius 2500)
    (inner-radius 1550)
    (height 10)
    (split 10)
    (step (/ 180 split))
    (outer-pos-list)
    (inner-pos-list)
    sin cos
    &allow-other-keys
    )
   (send-super* :init args)
   ;;
   (dotimes (i (+ 1 split))
     (setq sin (sin (deg2rad (* i step))))
     (setq cos (cos (deg2rad (* i step))))
     (push (float-vector (* outer-radius sin)
			 (* outer-radius cos) 0)
	   outer-pos-list)
     (push (float-vector (* inner-radius sin)
			 (* inner-radius cos) 0)
	   inner-pos-list))
   (setq
    table-prism
    (make-prism (append outer-pos-list (reverse inner-pos-list)) 10))
   (send table-prism :set-color #F(0 1 0))
   (gl::transparent table-prism 0.3)
   (setq root (instance bodyset-link :init (make-cascoords)
			:bodies (list table-prism) :name :table-prism))
   ;; (setq virtual-joint (make-cube 1 1 1))
   ;; (send virtual-joint :set-color #F(1 1 1))
   ;; (setq virtual-joint (instance bodyset-link :init (make-cascoords)
   ;; 				 :bodies (list virtual-joint) :name :virtual-joint))
   (send self :assoc root)
   ;; (send root :assoc virtual-joint)
   (setq links (list root))
   (setq joint-list nil)
	 ;; (list (instance rotational-joint :init
	 ;; 		 :parent-link virtual-joint :child-link nil
	 ;; 		 :name :sweeper-r :axis :x
	 ;; 		 :min-angle -90 :max-angle 90)
	 ;;       ))
   ;;
   (send self :init-ending)
   self))


(defun table-circle
  nil
  (setq *table-circle* (instance table-circle :init))
  ;; (send *table-circle* :set-color #F(0 1 0))
  (send *table-circle* :translate #F(0 0 -200) :world)
  (send *table-circle* :translate #F(-1400 0 0) :world)
;;  (objects *table-circle*)
  )

;; (table-circle)