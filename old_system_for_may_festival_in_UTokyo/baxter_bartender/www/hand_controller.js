var hand_controller = function(opt) {
    this.ros = new ROSLIB.Ros({
        url: "ws://" + "127.0.0.1" + ":9090"
    });

    this.hand_controller_topic = new ROSLIB.Topic({
        ros: this.ros,
        name: "/posture_controller/command/string",
        messageType: 'std_msgs/String'
    });

    hand_controller.prototype.calib = function(arm){
        var msg = new ROSLIB.Message(
            {data: "(send *ri* :hand-calibrate)"}
        );
        this.hand_controller_topic.publish(msg);
    };

    hand_controller.prototype.open = function(arm){
        var msg = new ROSLIB.Message(
            {data: "(send *ri* :stop-grasp " + arm + ")"}
        );
        this.hand_controller_topic.publish(msg);
    };

    hand_controller.prototype.close = function(arm){
        var msg = new ROSLIB.Message(
            {data: "(send *ri* :start-grasp " + arm + ")"}
        );
        this.hand_controller_topic.publish(msg);
    };

    hand_controller.prototype.reach_turtlebot = function(arm){
        var msg = new ROSLIB.Message(
            {}
        );
	topic = new ROSLIB.Topic({
            ros: this.ros,
            name: "/turtlebot_posture_controller/command/empty/" + arm,
            messageType: 'std_msgs/Empty'
	});
        topic.publish(msg);
    };

    hand_controller.prototype.reach_default_turtlebot = function(arm){
        var msg = new ROSLIB.Message(
            {}
        );
	topic = new ROSLIB.Topic({
            ros: this.ros,
            name: "/turtlebot_posture_controller/command/default/empty/" + arm,
            messageType: 'std_msgs/Empty'
	});
        topic.publish(msg);
    };

    hand_controller.prototype.reach_table = function(arm, name){
        var msg = new ROSLIB.Message(
            {data: name}
        );
	topic = new ROSLIB.Topic({
            ros: this.ros,
            name: "/table_posture_controller/command/string/" + arm,
            messageType: 'std_msgs/String'
	});
        topic.publish(msg);
    };

    hand_controller.prototype.hand_track = function(arm){
        var msg = new ROSLIB.Message(
            {data: arm}
        );
	topic = new ROSLIB.Topic({
            ros: this.ros,
            name: "/head_controller/arm_tracking/switch",
            messageType: 'std_msgs/String'
	});
        topic.publish(msg);
    };

    hand_controller.prototype.move_control = function(arm, cmd){
        var msg = new ROSLIB.Message(
            {data: cmd}
        );
	topic = new ROSLIB.Topic({
            ros: this.ros,
            name: "/kubi_interface/" + arm + "_hand/event/string",
            messageType: 'std_msgs/String'
	});
        topic.publish(msg);
    };
};

var hc = new hand_controller() ;

