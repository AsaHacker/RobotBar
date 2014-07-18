
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

    // for google speech recog
    this.google_recognizer = new webkitSpeechRecognition();
    this.google_recognizer.lang = "ja-JP";
    this.google_recognizer.interimResults = true;
    this.google_recognizer.continuous = true;
    this.google_recognizer.maxAlternatives = 10;

    this.google_recognizer.onsoundstart = function(){
	$("#state").text("speech detect");
    };
    this.google_recognizer.onnomatch = function(){
	$("#recognizedText").text("no match");
    };
    this.google_recognizer.onerror = function(event) {
	$("#recognizedText").text("error");
	if (event.error == 'no-speech') {
	    console.log('info_no_speech');
	}
	if (event.error == 'audio-capture') {
	    console.log('info_no_microphone');
	}
	if (event.error == 'not-allowed') {
            console.log('info_denied');
	}
    };
    this.google_recognizer.onsoundend = function(){
	$("#statet").text("idle");
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

    this.google_recognizer.onresult = this.onresult.bind(this) ;
    
    speech_recog.prototype.start = function(){
	this.google_recognizer.start() ;
    } ;

    speech_recog.prototype.stop = function(){
	this.google_recognizer.stop() ;
    } ;

}

var recognition = new speech_recog() ;
