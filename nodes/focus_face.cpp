#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <sensor_msgs/JointState.h>

// publishers
ros::Publisher pubServo;
ros::Publisher pubSpeaker;

const double panOffset = 0.5;
const double tiltOffset = 0.5;

std::vector<std::string> names;

double offsetX = 0;
double offsetY = 0;
ros::Time lastTime;

void faceBiasCallback(const geometry_msgs::PointPtr& msg)
{
  double x = msg->x;
  double y = msg->y;

  offsetX = 0.5 - x;
  offsetY = y - 0.5;

  lastTime = ros::Time::now();
  ROS_INFO("offsetX = [%f], offsetY = [%f]", offsetX, offsetY);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "foucs_face");
  ros::NodeHandle nh;
  ros::NodeHandle ph("~");
  ros::Subscriber recogSub = ph.subscribe("/face_bias", 1, faceBiasCallback);
  pubServo = nh.advertise<sensor_msgs::JointState>("joint_states", 1);

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
