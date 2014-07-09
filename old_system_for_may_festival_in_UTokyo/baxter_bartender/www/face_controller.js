var face_controller = function(opt) {
    this.ros = new ROSLIB.Ros({
        url: "ws://" + "127.0.0.1" + ":9090"
    });

    this.turtlebot_track_status_topic = new ROSLIB.Topic({
        ros: this.ros,
        name: "/head_controller/turtlebot_tracking/switch",
        messageType: 'std_msgs/String'
    });

    this.face_track_status_topic = new ROSLIB.Topic({
        ros: this.ros,
        name: "/head_controller/face_tracking/switch",
        messageType: 'std_msgs/String'
    });

    this.nod_topic = new ROSLIB.Topic({
        ros: this.ros,
        name: "/robot/head/command_head_nod",
        messageType: 'std_msgs/Bool'
    });

    this.pan_topic = new ROSLIB.Topic({
        ros: this.ros,
        name: "/robot/head/command_head_pan/relative",
        messageType: 'baxter_core_msgs/HeadPanCommand'
    });

    this.talk_topic = new ROSLIB.Topic({
        ros: this.ros,
        name: "/voice_echo/mei",
        messageType: 'std_msgs/String'
    });

    face_controller.prototype.nod = function(){
        var msg = new ROSLIB.Message(
            {data: true}
        );
        this.nod_topic.publish(msg);
    };

    face_controller.prototype.look_at = function(num){
        var msg = new ROSLIB.Message(
            {target: num, speed: 10}
        );
        this.pan_topic.publish(msg);
    };

    face_controller.prototype.face_track_status = function(st){
        var msg = new ROSLIB.Message(
            {data: st}
        );
        this.face_track_status_topic.publish(msg);
    };

    face_controller.prototype.turtlebot_track_status = function(st){
        var msg = new ROSLIB.Message(
            {data: st}
        );
        this.turtlebot_track_status_topic.publish(msg);
    };

    face_controller.prototype.talk = function(msg){
	var cmp = document.getElementById("mei_text") ;
	if ( cmp ){
            var msg = new ROSLIB.Message(
		{data: cmp.value}
            );
            this.talk_topic.publish(msg);
	}
    };

    face_controller.prototype.talk2 = function(msg){
        var msg = new ROSLIB.Message(
	    {data: msg}
        );
        this.talk_topic.publish(msg);
    };
};

var fc = new face_controller() ;

