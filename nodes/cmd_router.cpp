#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/String.h>
#include <std_msgs/Int32.h>
#include "cmd_router.h"

// publishers
ros::Publisher pubServo;
ros::Publisher pubSpeaker;

const double panOffset = 0.5;
const double tiltOffset = 0.5;

std::vector<std::string> names;

double offsetX = 0;
double offsetY = 0;
ros::Time lastTime;

int numFaceBiasCallBack = 0;
const int faceOutTime = 10; //secs
const int cmdOutTime = 2;
ros::Time lastTimeFaceBias;
ros::Time lastTimeCmd;
ros::Time lastTimeGesture;
std::string lastCmd;
int cmdLevel = 1;

void faceBiasCallback(const geometry_msgs::PointConstPtr& msg)
{
  // Face interval bigger then 10 secs
  if (ros::Time::now().sec - lastTimeFaceBias.sec > faceOutTime)
  {
    std_msgs::String voiceOutStr;
    voiceOutStr.data = "Hello I am a robot!";
    pubSpeaker.publish(voiceOutStr);
  }

  lastTimeFaceBias = ros::Time::now();
}

int gAge = 0;
void ageCallback(const std_msgs::Int32ConstPtr& msg)
{
  gAge = msg->data;
}

std::string gEmotion;
void emotionCallback(const std_msgs::StringConstPtr& msg)
{
  gEmotion = msg->data;
}

std::string gGesture;
void gestureCallback(const std_msgs::StringConstPtr& msg)
{
  gGesture = msg->data;

  if (ros::Time::now().sec - lastTimeGesture.sec < cmdOutTime + 4)
    return;

  if (gGesture == "left hand: spreadfingers")
  {
    sensor_msgs::JointState jointMsg;
    jointMsg.header.stamp = ros::Time::now();
    jointMsg.name = names;
    jointMsg.position.push_back(0);
    jointMsg.position.push_back(0);
    pubServo.publish(jointMsg);

    std_msgs::String voiceOutStr;
    voiceOutStr.data = "Ok, reset head position!";
    pubSpeaker.publish(voiceOutStr);
  }
  else if (gGesture == "left hand: v_sign" || gGesture == "right hand: v_sign")
  {
    std::string stmStr;
    if (ros::Time::now().sec - lastTimeFaceBias.sec > faceOutTime)
    {
      stmStr.append("I can not see you. Please face me!");
    }
    else if (gAge > 5)
    {
      stmStr.append("I guess your age is between ");
      stmStr.append(boost::lexical_cast<std::string>(gAge - 5));
      stmStr.append(" and ");
      stmStr.append(boost::lexical_cast<std::string>(gAge + 10));
      stmStr.append(" years old. And your emotion is ");
      stmStr.append(gEmotion);
    }
    else
    {
      stmStr.append("Hello. I can guess your age and emotion. Please face to me and try again!");
    }
    std_msgs::String msgStr;
    msgStr.data = stmStr;
    pubSpeaker.publish(msgStr);
  }
  else
  {
    std::string stmStr("I do not know the meaning of your gesture ");
    stmStr.append(gGesture);
    std_msgs::String msgStr;
    msgStr.data = stmStr;
    pubSpeaker.publish(msgStr);
  }

  lastTimeGesture = ros::Time::now();
}

void recogCallback(const std_msgs::StringConstPtr& msg)
{

  if (ros::Time::now().sec - lastTimeCmd.sec < cmdOutTime)
    return;

  std::string cmd = msg->data;

  if (cmdLevel == 1)
  {
    if (cmd == "hello")
    {
      std::string stmStr;
      if (ros::Time::now().sec - lastTimeFaceBias.sec > faceOutTime)
      {
        stmStr.append("I can not see you. Please face me!");
      }
      else if (gAge > 5)
      {
        stmStr.append("I guess your age is between ");
        stmStr.append(boost::lexical_cast<std::string>(gAge - 5));
        stmStr.append(" and ");
        stmStr.append(boost::lexical_cast<std::string>(gAge + 10));
        stmStr.append(" years old. Is that right?");
        cmdLevel = 2;
        lastCmd = cmd;
      }
      else
      {
        stmStr.append("Hello. I can guess your age. Please wait a minute!");
      }
      std_msgs::String msgStr;
      msgStr.data = stmStr;
      pubSpeaker.publish(msgStr);
    }
  }

  if (cmdLevel == 2)
  {
    std::string stmStr;
    if (cmd == "yes" && lastCmd == "hello")
    {
      if (gEmotion == "joy")
      {
        stmStr.append("It seems to you are happy!");
      }
      else
      {
        stmStr.append("I think you are ");
        stmStr.append(gEmotion);
      }

      cmdLevel = 1;
    }
    if (cmd == "no" && lastCmd == "hello")
    {
      stmStr.append("Let me guess again. You are between ");
      stmStr.append(boost::lexical_cast<std::string>(gAge - 5));
      stmStr.append(" and ");
      stmStr.append(boost::lexical_cast<std::string>(gAge + 10));
      stmStr.append(" years old. Is that right?");
    }

    std_msgs::String msgStr;
    msgStr.data = stmStr;
    pubSpeaker.publish(msgStr);
  }

  lastTimeCmd = ros::Time::now();
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "cmd_router");
  ros::NodeHandle nh;
  ros::NodeHandle ph("~");
  ros::Subscriber subFaceBias = nh.subscribe("face_bias", 1, faceBiasCallback);
  ros::Subscriber subAge = nh.subscribe("age", 1, ageCallback);
  ros::Subscriber subEmotion = nh.subscribe("emotion", 1, emotionCallback);
  ros::Subscriber subGesture = nh.subscribe("gesture", 1, gestureCallback);
  ros::Subscriber subRecog = nh.subscribe("recognizer/output", 1, recogCallback);
  pubSpeaker = nh.advertise<std_msgs::String>("voice_synthesis", 1);
  pubServo = nh.advertise<sensor_msgs::JointState>("joint_states", 1);

  names.push_back("head_pan_joint");
  names.push_back("head_tilt_joint");

  int rate = 2;
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
