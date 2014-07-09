(defclass table-audience
  :super cascaded-link
  :slots (root table-pos-list table1 table2 table3 audience-list))

(defmethod table-audience
  (:init
   (&rest
    args
    &key
    (w 500)
    (h 1800)
    (s (/ h 4.0))
    (table-pos-list1
     (list (float-vector 0 0 0)
	   (float-vector w 0 0)
	   (float-vector w h 0)
	   (float-vector 0 h 0)))
    (table-pos-list2
     (list (float-vector 0 0 0)
	   (float-vector w 0 0)
	   (float-vector w (* -1 h) 0)
	   (float-vector 0 (* -1 h) 0)))
    (table-pos-list3
     (list (float-vector w h 0)
	   (float-vector w (+ h w) 0)
	   (float-vector (- w h) (+ h w) 0)
	   (float-vector (- w h) h 0)))
    (audience
     (mapcar
      #'(lambda (name pos)
	  (let ((ret (make-cube 100 100 10)))
	    (send ret :set-color #F(1 0 0))
	    (send ret :name name)
	    (send ret :translate pos :world)))
      (list "A" "B" "C" "D" "E" "F")
      (list (float-vector 0 (* -3 s) 0)
	    (float-vector 0 (* -1 s) 0)
	    (float-vector 0 (* +1 s) 0)
	    (float-vector 0 (* +3 s) 0)
	    (float-vector (- w s) h 0)
	    (float-vector (- (+ s w) h) h 0))))
    (tb1 (make-prism table-pos-list1 10))
    (tb2 (make-prism table-pos-list2 10))
    (tb3 (make-prism table-pos-list3 10))
    &allow-other-keys
    )
   (send-super* :init args)
   ;;
   (send tb1 :set-color #F(0 1 0))
   (send tb2 :set-color #F(0 1 0))
   (send tb3 :set-color #F(0 1 0))
   (gl::transparent tb1 0.3)
   (gl::transparent tb2 0.3)
   (gl::transparent tb3 0.3)
   (setq table1 tb1)
   (setq table2 tb2)
   (setq table3 tb3)
   (setq audience-list audience)
   (setq root (instance bodyset-link
			:init (make-cascoords)
			:bodies (append (list table1 table2 table3)
					audience-list)
			:name :table-prism))
   (send self :assoc root)
   (send root :assoc table1)
   (send root :assoc table2)
   (send root :assoc table3)
   (mapcar
    #'(lambda (ad) (send root :assoc ad))
    audience-list)
   ;;(send table2 :assoc table3)
   (setq links (list root))
   (setq joint-list nil)
   (send self :init-ending)
   self))

(defun table-audience
  nil
  (setq *table-audience* (instance table-audience :init))
  (send *table-audience* :translate #F(0 0 100) :world)
  (send *table-audience* :translate #F(1000 -200 0) :world)
  )

#|

(table-audience)
(objects (list *table-audience*))

