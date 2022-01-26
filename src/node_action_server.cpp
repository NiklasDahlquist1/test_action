#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float32.h"
#include "geometry_msgs/Pose2D.h"
#include "geometry_msgs/Pose.h"

#include <actionlib/server/simple_action_server.h>
#include "test_action/TestAction.h"
#include "test_action/MoveToAction.h"

class FibonacciAction
{
protected:

  ros::NodeHandle nh_;
  actionlib::SimpleActionServer<test_action::TestAction> as_; // NodeHandle instance must be created before this line. Otherwise strange error occurs.
  std::string action_name_;
  // create messages that are used to published feedback/result
  test_action::TestFeedback feedback_;
  test_action::TestResult result_;

public:

  FibonacciAction(std::string name) :
    as_(nh_, name, boost::bind(&FibonacciAction::executeCB, this, _1), false),
    action_name_(name)
  {
    as_.start();
  }

  ~FibonacciAction(void)
  {
  }

  void executeCB(const test_action::TestGoalConstPtr &goal)
  {
    // helper variables
    ros::Rate r(1);
    bool success = true;



    // publish info to the console for the user
    //ROS_INFO("%s: Executing, creating fibonacci sequence of order %i with seeds %i, %i", action_name_.c_str(), goal->pose_goal, feedback_.pose_current.pose.position.x, feedback_.sequence[1]);

    // start executing the action
    for(int i=1; i<10; i++)
    {
      // check that preempt has not been requested by the client
      if (as_.isPreemptRequested() || !ros::ok())
      {
        ROS_INFO("%s: Preempted", action_name_.c_str());
        // set the action state to preempted
        as_.setPreempted();
        success = false;
        break;
      }
      feedback_.pose_current.x = i;
      // publish the feedback
      //as_.publishFeedback(feedback_);
      // this sleep is not necessary, the sequence is computed at 1 Hz for demonstration purposes
      r.sleep();
    }

    if(success)
    {
      result_.pose_final.x = 2;
      ROS_INFO("%s: Succeeded", action_name_.c_str());
      // set the action state to succeeded
      as_.setSucceeded(result_);
    }
  }


};


class SubAction
{
protected:

  ros::NodeHandle nh_;
  actionlib::SimpleActionServer<test_action::MoveToAction> as_; // NodeHandle instance must be created before this line. Otherwise strange error occurs.
  std::string action_name_;

  geometry_msgs::Pose goal_;

  // create messages that are used to published feedback/result
  test_action::MoveToFeedback feedback_;
  test_action::MoveToResult result_;

  ros::Subscriber sub_;

public:

  SubAction(std::string name) :
    //as_(nh_, name, false),
    as_(nh_, name, boost::bind(&SubAction::goalCB, this, _1), false),
    action_name_(name)
  {
    //register the goal and feeback callbacks
    //as_.registerGoalCallback(boost::bind(&SubAction::goalCB, this));
    //as_.registerPreemptCallback(boost::bind(&SubAction::preemptCB, this));

    //subscribe to the data topic of interest
    sub_ = nh_.subscribe("/random_number", 1, &SubAction::subCB, this);

    as_.start();
  }

  ~SubAction(void)
  {
  }

  void goalCB(const test_action::MoveToGoalConstPtr &goal)
  {
    // accept the new goal
    //goal_ = as_.acceptNewGoal()->pose_goal;
    goal_ = goal->pose_goal;  

    ros::Rate r(1);
    for(int i = 0; i < 10; ++i)
    {
      ROS_INFO("current goal, %f", goal_.position.x);
        r.sleep();
    }
    as_.setSucceeded(result_);
  }

  void preemptCB(const test_action::MoveToGoalConstPtr &goal)
  {
    ROS_INFO("%s: Preempted", action_name_.c_str());
    // set the action state to preempted
    as_.setPreempted();
  }

  void subCB(const std_msgs::Float32::ConstPtr& msg)
  {
    ROS_INFO("read data from topic, %f", msg->data);
  }

};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "fibonacci");

  SubAction subAction("node");
  //FibonacciAction fibonacci("fibonacci");
  ros::spin();

  return 0;
}