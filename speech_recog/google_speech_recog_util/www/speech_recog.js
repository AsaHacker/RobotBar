
var speech_recog = function(opt){

    // for ros
    this.ros = new ROSLIB.Ros({
        url: "ws://" + "127.0.0.1" + ":9090"
    });

    this.best_topic = new ROSLIB.Topic({
        ros: this.ros,
        name: "speech_recog/best",
        messageType: 'std_msgs/String'
    });

    this.all_topic = new ROSLIB.Topic({
        ros: this.ros,
        name: "speech_recog/all",
        messageType: 'std_msgs/String'
    });

    this.status_topic = new ROSLIB.Topic({
        ros: this.ros,
        name: "speech_recog/status/string",
        messageType: 'std_msgs/String'
    });

    speech_recog.prototype.status_publish = function(st){
	var st_msg = new ROSLIB.Message(
            {data: st}
        );
	this.status_topic.publish(st_msg) ;
    };
	
    speech_recog.prototype.speech_publish = function(best, all, rate){
        var best_msg = new ROSLIB.Message(
            {data: best}
        );
        this.best_topic.publish(best_msg);
	//
	var max = Math.min(all.length,rate.length);
	if ( max > 0 ){
	    var all_str = "" ;
	    for ( var i=0 ; i<max ; i++ ){
		all_str += all[i] + "/" + rate[i] ;
		if ( i != max - 1 ){
		    all_str += " " ;
		}
	    }
	    var all_msg  = new ROSLIB.Message(
		{data: all_str}
            );
	    this.all_topic.publish(all_msg) ;
	}
    };

    speech_recog.prototype.onresult = function(event){
	var results = event.results;
	for (var i = event.resultIndex; i<results.length; i++){
            if(results[i].isFinal){
		$("#recognizedText").text(results[i][0].transcript);
            }
            else{
		$("#recognizedText").text(results[i][0].transcript);
            }
	    this.speech_publish( results[i][0].transcript, [], [] ) ;
	}
	$("#recognizedDetail").empty();
	var str_buf = new Array() ;
	var id_buf = new Array() ;
	for (var i = event.resultIndex; i<results.length; i++){
            if(results[i].isFinal){
		for (var j = 0; j<results[i].length; j++){
		    if ( results[i][j] ){
			$("#recognizedDetail").append("<p>" + results[i][j].transcript + "(" + results[i][j].confidence + ")" + "</p>");
			str_buf[str_buf.length] = results[i][j].transcript;
			id_buf[id_buf.length] = results[i][j].confidence;
		    }
		}
		this.speech_publish( results[i][0].transcript, str_buf, id_buf ) ;
            }
	}
    };

    speech_recog.prototype.onsoundstart = function(){
	$("#state").text("speech detect");
	this.status_publish("speech") ;
    }

    speech_recog.prototype.onnomatch = function(){
	$("#state").text("no match");
	this.status_publish("nomatch") ;
    }
    
    speech_recog.prototype.onerror = function(event) {
	$("#recognizedText").text("error");
	if (event.error == 'no-speech') {
	    console.log('info_no_speech');
	    this.status_publish("nospeech") ;
	}
	if (event.error == 'audio-capture') {
	    console.log('nomic');
	    this.status_publish("nomic") ;
	}
	if (event.error == 'not-allowed') {
            console.log('forbit');
	    this.status_publish("nopermission") ;
	}
    };

    speech_recog.prototype.onsoundend = function(){
	$("#statet").text("idle");
	this.status_publish("idle") ;
    };
    
    speech_recog.prototype.start = function(){
	this.google_recognizer.start() ;
    } ;

    speech_recog.prototype.stop = function(){
	this.google_recognizer.stop() ;
    } ;

    // for google speech recog
    this.google_recognizer = new webkitSpeechRecognition();
    this.google_recognizer.lang = "ja-JP";
    // this.google_recognizer.interimResults = true;
    this.google_recognizer.continuous = true;
    this.google_recognizer.maxAlternatives = 3;
    
    this.google_recognizer.onsoundstart = this.onsoundstart.bind(this) ;
    this.google_recognizer.onnomatch = this.onnomatch.bind(this) ;
    this.google_recognizer.onerror = this.onerror.bind(this) ;
    this.google_recognizer.onsoundend = this.onsoundend.bind(this);
    this.google_recognizer.onresult = this.onresult.bind(this) ;

}

var recognition = new speech_recog() ;
