#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <actionlib/server/simple_action_server.h>

#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseStamped.h"
#include <test_action/MoveToAction.h>


#include "nav_msgs/Odometry.h"


class MoveTo
{
public:
    
  MoveTo(std::string name ) : 
    as_(nh_, name, false),
    action_name_(name)
  {
    //register the goal and feeback callbacks
    as_.registerGoalCallback(boost::bind(&MoveTo::goalCB, this));
    as_.registerPreemptCallback(boost::bind(&MoveTo::preemptCB, this));



    //subscribe to the data topic of interest
    sub_ = nh_.subscribe("odometry", 1, &MoveTo::subscribeCB, this);

    pub_pose_ = nh_.advertise<geometry_msgs::PoseStamped>("goalPose", 1000);

    as_.start();
  }

  ~MoveTo(void)
  {
  }

  void goalCB()
  {
    // accept the new goal
    goal_ = as_.acceptNewGoal()->pose_goal;

    ROS_INFO("Got new goal, [%f, %f, %f]", goal_.position.x, goal_.position.y, goal_.position.z);

    geometry_msgs::PoseStamped p;
    p.header.stamp = ros::Time::now();
    p.pose = goal_;

    pub_pose_.publish(p);
  }

  void preemptCB()
  {
    ROS_INFO("%s: Preempted", action_name_.c_str());
    // set the action state to preempted
    as_.setPreempted();
  }

  void subscribeCB(const nav_msgs::Odometry& msg)//const std_msgs::Float32::ConstPtr& msg)
  {

    // make sure that the action hasn't been canceled
    if (!as_.isActive())
      return;
    


    //ROS_INFO("Received ros data [%f, %f, %f]", msg->position.x, msg->position.y, msg->position.z);
    current_ = msg.pose.pose;

    //publish feedback
    feedback_.pose_current = current_;
    as_.publishFeedback(feedback_);


    float error = pow((goal_.position.x - current_.position.x), 2) + pow((goal_.position.y - current_.position.y), 2) + pow((goal_.position.z - current_.position.z), 2);
    float tol = 0.1; // tolerance to accept current position
    if(error < pow(tol, 2))
    {
        ROS_INFO("Action completed, position [%f, %f, %f]", current_.position.x, current_.position.y, current_.position.z);
        result_.pose_final = current_;
        as_.setSucceeded(result_);
    }




    //set the action state to aborted
    //as_.setAborted(result_);
    // set the action state to succeeded
    //as_.setSucceeded(result_);
  }

protected:
    
  ros::NodeHandle nh_;
  actionlib::SimpleActionServer<test_action::MoveToAction> as_;
  std::string action_name_;
  geometry_msgs::Pose goal_;
  geometry_msgs::Pose current_;

  test_action::MoveToFeedback feedback_;
  test_action::MoveToResult result_;
  ros::Subscriber sub_;
  ros::Publisher pub_pose_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "moveTo");


  MoveTo moveto(ros::this_node::getName());

  ros::spin();

  return 0;
}
