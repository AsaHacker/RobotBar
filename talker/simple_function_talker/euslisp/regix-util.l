
(defun regix-split
  (pat str
       &optional (replace "\\n"))
  (let* ((sed (piped-fork
	       ;; (format nil "echo \"~A\" | sed s/\"~A\"/\"\\~A\"/g" str pat replace)))
	       (print (format nil "echo \"~A\" | sed -e \"s/~A/~A/g\"" str pat replace))))
	 buf ret)
    (while (setq buf (read-line sed nil))
      (push buf ret))
    (close sed)
    ret))

(defun regix-match-check
  (pat str)
  (> (length (regix-split pat str)) 1))

(defun regix-keyword
  (str)
  (cdr (regix-split "[^ァ-ヶa-zA-Z一-龠]\\+" str "\\n")))

(defun regix-predicate
  (verb str)
  (car
   (sort
    (regix-keyword (cadr (regix-split verb str)))
    #'(lambda (a b) (> (length a) (length b))))))

#|

(defun keyword-matcher
  (pattern msg id)
  (let* (buf)
    (setq buf (find-if #'(lambda (str) (reg-match pattern str))
		       msg))
    ((nth id buf)
     (regix-split "[^ァ-ヶa-zA-Z一-龠\-ー 　]+" "最近夫の帰りが遅くて体調が心配です")

(regix-split "[^ァ-ヶa-zA-Z一-龠]\\+" "最近夫の帰りが遅くて体調が心配です" "\\n")
(regix-split "\\([^ァ-ヶa-zA-Z\\一-龠]\\+\\)"
	     "最近夫の帰りが遅くて体調が心配です" "\\n\\1\\n")
