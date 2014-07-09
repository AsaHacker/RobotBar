var body_controller = function(opt) {
    this.ros = new ROSLIB.Ros({
        url: "ws://" + "127.0.0.1" + ":9090"
    });

    this.servo_topic = new ROSLIB.Topic({
        ros: this.ros,
        name: "posture_controller_node/body/command/string",
        messageType: 'std_msgs/String'
    });

    this.oshibori_topic = new ROSLIB.Topic({
        ros: this.ros,
        name: "/oshibori_reacher/request",
        messageType: 'std_msgs/Empty'
    });

    this.log_start_topic = new ROSLIB.Topic({
        ros: this.ros,
        name: "/angle_vector_logger/command/start",
        messageType: 'std_msgs/Empty'
    });

    this.log_stop_topic = new ROSLIB.Topic({
        ros: this.ros,
        name: "/angle_vector_logger/command/stop",
        messageType: 'std_msgs/Empty'
    });

    this.log_play_topic = new ROSLIB.Topic({
        ros: this.ros,
        name: "/angle_vector_logger/command/log_player",
        messageType: 'std_msgs/String'
    });

    body_controller.prototype.servo_status = function(st){
        var msg = new ROSLIB.Message(
            {data: st}
        );
        this.servo_topic.publish(msg);
    };

    body_controller.prototype.oshibori_reach = function(st){
        var msg = new ROSLIB.Message(
            {}
        );
        this.oshibori_topic.publish(msg);
    };

    body_controller.prototype.log_start = function(st){
        var msg = new ROSLIB.Message(
            {}
        );
        this.log_start_topic.publish(msg);
    };

    body_controller.prototype.log_stop = function(st){
        var msg = new ROSLIB.Message(
            {}
        );
        this.log_stop_topic.publish(msg);
    };

    body_controller.prototype.log_play = function(path){
        var msg = new ROSLIB.Message(
            {data: path}
        );
        this.log_play_topic.publish(msg);
    };
};

var bc = new body_controller() ;

