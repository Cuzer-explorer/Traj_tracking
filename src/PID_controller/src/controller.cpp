#include "ros/ros.h"
#include "teb_local_planner/FeedbackMsg.h"
#include <geometry_msgs/TwistStamped.h>
#include "pid.h"


class Controller{
private:
    ros::NodeHandle n;

    double v_tg, w_tg, v_fb, w_fb, v_ctrl=0, w_ctrl=0;
    bool tg_CB = false, fb_CB = false;

    PID v_pid, w_pid;
    double v_kp, v_ki, v_kd, max_v, max_v_iout;
    double w_kp, w_ki, w_kd, max_w, max_w_iout;

    ros::Subscriber target_vel_sub, feedback_vel_sub;
    ros::Publisher vel_pub;
    std::string target_vel_topic, feedback_vel_topic, pub_vel_topic;
    ros::Timer update_timer;
    double control_rate;

public:
    Controller(){
        n = ros::NodeHandle("~");
        n.getParam("target_vel_topic", target_vel_topic);
        n.getParam("feedback_vel_topic", feedback_vel_topic);
        n.getParam("pub_vel_topic", pub_vel_topic);
        n.getParam("control_rate", control_rate);

        n.getParam("v_kp", v_kp);
        n.getParam("v_ki", v_ki);
        n.getParam("v_kd", v_kd);
        n.getParam("max_v", max_v);
        n.getParam("max_v_iout", max_v_iout);

        n.getParam("w_kp", w_kp);
        n.getParam("w_ki", w_ki);
        n.getParam("w_kd", w_kd);
        n.getParam("max_w", max_w);
        n.getParam("max_w_iout", max_w_iout);

        v_pid = PID(v_kp, v_ki, v_kd, max_v, max_v_iout);
        w_pid = PID(w_kp, w_ki, w_kd, max_w, max_w_iout);
        
        target_vel_sub = n.subscribe(target_vel_topic, 10, &Controller::target_vel_CB, this);
        feedback_vel_sub = n.subscribe(feedback_vel_topic, 10, &Controller::feedback_vel_CB, this);
        vel_pub = n.advertise<geometry_msgs::TwistStamped>(pub_vel_topic, 10);

    }

    // void target_vel_CB(const teb_local_planner::FeedbackMsg::ConstPtr & msg){  // for teb
    //     // std::cout << "1" << std::endl;
    //     if (msg->trajectories.size() == 0)
    //         return;
    //     v_tg = msg->trajectories.front().trajectory[1].velocity.linear.x;
    //     w_tg = msg->trajectories.front().trajectory[1].velocity.angular.z;
    //     // std::cout << "2" << std::endl;
    //     if (!tg_CB)    tg_CB = true;
    // }
    void target_vel_CB(const geometry_msgs::Twist::ConstPtr & msg){  // for dwa
        v_tg = msg->linear.x;
        w_tg = msg->angular.z;
        if (!tg_CB)    tg_CB = true;
    }

    void feedback_vel_CB(const geometry_msgs::TwistStamped::ConstPtr& msg){
        v_fb = msg->twist.linear.x;
        w_fb = msg->twist.angular.z;
        if (!fb_CB)    fb_CB = true;
    }

    void run(){
        update_timer = n.createTimer(ros::Duration(control_rate), &Controller::update_output, this);
        ros::spin();
    }

    void update_output(const ros::TimerEvent&){
        if (!tg_CB || !fb_CB)
            return;
        // std::cout << "ddddd" << std::endl;
        // v_fb = 0;
        // w_fb = 0;
        // double v_delta, w_delta;
        // v_delta = v_pid.compute(v_tg, v_fb);
        // w_delta = w_pid.compute(w_tg,  w_fb);
        // v_ctrl += v_delta;
        // w_ctrl += w_delta;
        v_ctrl = v_pid.compute(v_tg, v_fb);
        w_ctrl = w_pid.compute(w_tg,  w_fb);
        vel_publish();
    }

    void vel_publish(){
        geometry_msgs::TwistStamped vel_msg;
        vel_msg.header.stamp = ros::Time();
        vel_msg.twist.linear.x = v_ctrl;
        vel_msg.twist.angular.z = w_ctrl;
        vel_pub.publish(vel_msg);
        std::cout << "v_fb: " << v_fb << ", v_ctrl: " << v_ctrl << std::endl;
        std::cout << "w_fb: " << w_fb << ", w_ctrl: " << w_ctrl << std::endl;
    }
};


int main(int argc,char **argv)
{
    ros::init(argc,argv,"chassis_controller");
    Controller ctrller;
    ctrller.run();
    return 0;
}