
(require "regix-util.l")

;; usage
;; string, funciton, list of them
(setq *response-key-pairs*
      (append
       ;;
       (list
	(list :hello-hello
	      (list "こんにちわ\\|こんにちは\\|ハロー\\|良い天気"
		    "こんにちわ\\|こんにちは\\|ハロー\\|良い天気")
	      (list "何度もどうも" "何度もこんにちわ"))
	(list :hello
	      "こんにちわ\\|こんにちは\\|ハロー\\|良い天気"
	      (list "こんにちわ" "ご機嫌うるわしゅう"))
	(list :duplicate4
	      (list '(lambda (msg) (setq hoge msg))
		    '(lambda (msg) (string-equal hoge msg))
		    '(lambda (msg) (string-equal hoge msg))
		    '(lambda (msg) (string-equal hoge msg))
		    '(lambda (msg) (string-equal hoge msg))
		    '(lambda (msg) (string-equal hoge msg))
		    '(lambda (msg) (string-equal hoge msg))
		    '(lambda (msg) (string-equal hoge msg))
		    )
	      (list "・・・・・・"))
	(list :duplicate3
	      (list '(lambda (msg) (setq hoge msg))
		    '(lambda (msg) (string-equal hoge msg))
		    '(lambda (msg) (string-equal hoge msg))
		    '(lambda (msg) (string-equal hoge msg))
		    '(lambda (msg) (string-equal hoge msg))
		    '(lambda (msg) (string-equal hoge msg))
		    '(lambda (msg) (string-equal hoge msg)))
	      (list "別のことを言うまで、返事しませんからね"))
	(list :duplicate2
	      (list '(lambda (msg) (setq hoge msg))
		    '(lambda (msg) (string-equal hoge msg))
		    '(lambda (msg) (string-equal hoge msg))
		    '(lambda (msg) (string-equal hoge msg))
		    '(lambda (msg) (string-equal hoge msg)))
	      (list "その言葉にはもうあきあきです"
		    "私のことをバカだと思っていますね？"))
	(list :duplicate
	      (list '(lambda (msg) (setq hoge msg))
		    '(lambda (msg) (string-equal hoge msg)))
	      (list "同じことを言わないでください"
		    "もう少しわかりやすくお願いできますか？"))
	(list :weather
	      "天気"
	      (list "今日の天気は晴れです" "今日の天気は雨です"))
	(list :debug
	      "調子"
	      '(lambda (msg) "調子良好"))
	(list :name
	      "名前"
	      "私の名前はすんすんです")
	)

       ;; eliza
       (list
	(list :computer
	      "computer\\|機械\\|パソコン"
	      (list "機械が悩みですか?" "機械をどう思うのですか？"
		    "機械をどうしたいのですか？"))
	(list :sorry
	      "すいません\\|sorry\\|ソーリー\\|申し訳ない\\|ごめん\\|悪い\\|悪かった"
	      (list "あやまらないでください"
		    "謝る必要は有りません" "どうしてあやまるのです？"))
	(list :remember
	      "思い出す\\|忘れられ\\|夢にみる\\|怖い"
	      '(lambda (msg)
		 (let* ((pd (regix-predicate "[思い出す|忘れられない]" (car (flatten msg)))))
		   (cond
		    (pd
		     (list (format nil "~Aについて他に何がありますか?" pd)
			   (format nil "なぜ~Aなのですか?" pd)
			   (format nil "例えばいつ~Aが思い出されますか?" pd)
			   (format nil "~Aとはどういう関係ですか?" pd)))
		    (t (list "他に何が思い出されますか？"))))))
	)

	;;
	;; error
       (list
	(list :nomatch #'(lambda (&rest args) t)
	      (list "別の言葉で言い換えてください"
		    "難しいですね。続けてください"
		    "続けてください"))
	(list :echo #'(lambda (&rest args) t) #'caar)
	)
       ))

#|

	(((?* ?x) do you remember (?* ?y))
	 (Did you think I would forget ?y ?)
	 (Why do you think I should recall ?y now)
	 (What about ?y) (You mentioned ?y))
	(((?* ?x) if (?* ?y))
	 (Do you really think its likely that ?y) (Do you wish that ?y)
	 (What do you think about ?y) (Really-- if ?y))
	(((?* ?x) I dreamt (?* ?y))
	 (Really-- ?y) (Have you ever fantasized ?y while you were awake?)
	 (Have you dreamt ?y before?))
	(((?* ?x) dream about (?* ?y))
	 (How do you feel about ?y in reality?))
	(((?* ?x) dream (?* ?y))
	 (What does this dream suggest to you?) (Do you dream often?)
	 (What persons appear in your dreams?)
	 (Don't you believe that dream has to do with your problem?))
	(((?* ?x) my mother (?* ?y))
	 (Who else in your family ?y) (Tell me more about your family))
	(((?* ?x) my father (?* ?y))
	 (Your father) (Does he influence you strongly?)
	 (What else comes to mind when you think of your father?))
	(((?* ?x) I want (?* ?y))
	 (What would it mean if you got ?y)
	 (Why do you want ?y) (Suppose you got ?y soon))
	(((?* ?x) I am glad (?* ?y))
	 (How have I helped you to be ?y) (What makes you happy just now)
	 (Can you explain why you are suddenly ?y))
	(((?* ?x) I am sad (?* ?y))
	 (I am sorry to hear you are depressed)
	 (I'm sure its not pleasant to be sad))
	(((?* ?x) are like (?* ?y))   
	 (What resemblance do you see between ?x and ?y))
	(((?* ?x) is like (?* ?y))    
	 (In what way is it that ?x is like ?y)
	 (What resemblance do you see?)
	 (Could there really be some connection?) (How?))
	(((?* ?x) alike (?* ?y))      
	 (In what way?) (What similarities are there?))
	(((?* ?x) same (?* ?y))       
	 (What other connections do you see?))

   (((?* ?x) I was (?* ?y))       
    (Were you really?) (Perhaps I already knew you were ?y)
    (Why do you tell me you were ?y now?))
   (((?* ?x) was I (?* ?y))
    (What if you were ?y ?) (Do you thin you were ?y)
    (What would it mean if you were ?y))
   (((?* ?x) I am (?* ?y))       
    (In what way are you ?y) (Do you want to be ?y ?))
   (((?* ?x) am I (?* ?y))
    (Do you believe you are ?y) (Would you want to be ?y)
    (You wish I would tell you you are ?y)
    (What would it mean if you were ?y))
   (((?* ?x) am (?* ?y))
    (Why do you say "AM?") (I don't understand that))
   (((?* ?x) are you (?* ?y))
    (Why are you interested in whether I am ?y or not?)
    (Would you prefer if I weren't ?y)
    (Perhaps I am ?y in your fantasies))
   (((?* ?x) you are (?* ?y))   
    (What makes you think I am ?y ?))

   (((?* ?x) because (?* ?y))
    (Is that the real reason?) (What other reasons might there be?)
    (Does that reason seem to explain anything else?))
   (((?* ?x) were you (?* ?y))
    (Perhaps I was ?y) (What do you think?) (What if I had been ?y))
   (((?* ?x) I can't (?* ?y))    
    (Maybe you could ?y now) (What if you could ?y ?))
   (((?* ?x) I feel (?* ?y))     
    (Do you often feel ?y ?))
   (((?* ?x) I felt (?* ?y))     
    (What other feelings do you have?))
   (((?* ?x) I (?* ?y) you (?* ?z))   
    (Perhaps in your fantasy we ?y each other))
   (((?* ?x) why don't you (?* ?y))
    (Should you ?y yourself?)
    (Do you believe I don't ?y) (Perhaps I will ?y in good time))
   (((?* ?x) yes (?* ?y))
    (You seem quite positive) (You are sure) (I understand))
   (((?* ?x) no (?* ?y))
    (Why not?) (You are being a bit negative)
    (Are you saying "NO" just to be negative?))

   (((?* ?x) someone (?* ?y))
    (Can you be more specific?))
   (((?* ?x) everyone (?* ?y))
    (surely not everyone) (Can you think of anyone in particular?)
    (Who for example?) (You are thinking of a special person))
   (((?* ?x) always (?* ?y))
    (Can you think of a specific example) (When?)
    (What incident are you thinking of?) (Really-- always))
   (((?* ?x) what (?* ?y))
    (Why do you ask?) (Does that question interest you?)
    (What is it you really want to know?) (What do you think?)
    (What comes to your mind when you ask that?))
   (((?* ?x) perhaps (?* ?y))    
    (You do not seem quite certain))
   (((?* ?x) are (?* ?y))
    (Did you think they might not be ?y)
    (Possibly they are ?y))
   (((?* ?x))               
    (Very interesting) (I am not sure I understand you fully)
    (What does that suggest to you?) (Please continue) (Go on) 
    (Do you feel strongly about discussing such things?))))
