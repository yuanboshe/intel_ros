#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/String.h>

// publishers
ros::Publisher pubServo;
ros::Publisher pubSpeaker;

const double panOffset = 0.5;
const double tiltOffset = 0.5;

std::vector<std::string> names;

double offsetX = 0;
double offsetY = 0;
ros::Time lastTime;

// rocking detect
double lastX = 0;
double lastY = 0;
ros::Time rockTime;
double rockDist = 0;

void faceBiasCallback(const geometry_msgs::PointPtr& msg)
{
  double x = msg->x;
  double y = msg->y;

  offsetX = 0.5 - x;
  offsetY = y - 0.5;

  // detect rocking
  if (ros::Time::now().sec - lastTime.sec > 2 || ros::Time::now().sec - rockTime.sec > 7)
  { //clear
    rockDist = 0;
    rockTime = ros::Time::now();
    ROS_INFO("rocking clear!");
  }
  else if (rockDist > 2)
  { //detected
    std::string stmStr("Stop, stop. You make me dizzy!");
    std_msgs::String msgStr;
    msgStr.data = stmStr;
    pubSpeaker.publish(msgStr);

    rockDist = 0;
    rockTime = ros::Time::now();
    ROS_INFO("rocking detected!");
  }
  else
  {
    rockDist += fabs(x - lastX) + fabs(y - lastY);
    ROS_INFO("rockDist = [%f]", rockDist);
  }

  lastX = x;
  lastY = y;

  // common update
  lastTime = ros::Time::now();
  ROS_INFO("offsetX = [%f], offsetY = [%f]", offsetX, offsetY);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "foucs_face");
  ros::NodeHandle nh;
  ros::NodeHandle ph("~");
  ros::Subscriber subFaceRec = ph.subscribe("/face_bias", 1, faceBiasCallback);
  pubServo = nh.advertise<sensor_msgs::JointState>("joint_states", 1);
  pubSpeaker = nh.advertise<std_msgs::String>("voice_synthesis", 1);

  lastTime = ros::Time::now();
  names.push_back("head_pan_joint");
  names.push_back("head_tilt_joint");

  int rate = 20;
  ros::Rate loopRate(rate);
  double speedRate = 1.2 / rate;

  double currentX = 0;
  double currentY = 0;

  while (ros::ok())
  {
    ros::Duration interval = ros::Time::now() - lastTime;
    if (interval.sec < 1)
    {
      currentX += offsetX * speedRate;
      currentY += offsetY * speedRate;

      sensor_msgs::JointState jointMsg;
      jointMsg.header.stamp = ros::Time::now();
      jointMsg.name = names;
      jointMsg.position.push_back(currentX);
      jointMsg.position.push_back(currentY);
      pubServo.publish(jointMsg);

      ROS_INFO("x: %f  y: %f", currentX, currentY);
    }

    ros::spinOnce();
    loopRate.sleep();
  }

  return 0;
}
