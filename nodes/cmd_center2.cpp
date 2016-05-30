#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <intel_ros/TwistWithMask.h>
#include "csvparser.h"
#include <iostream>
#include <fstream>

// global vars
bool paused;
double linearSpeed;
double angularSpeed;
double linearStep;
double angularStep;

// publishers
ros::Publisher authorPub;
ros::Publisher goalVelPub;
ros::Publisher goalVelMaskPub;
ros::Publisher speaker;
ros::Publisher simpleGoal;

void pubGoalVel(double lx, double ly, double az)
{
  geometry_msgs::Twist gVel;
  gVel.linear.x = lx;
  gVel.linear.y = ly;
  gVel.angular.z = az;
  goalVelPub.publish(gVel);
}

std::string author;
void pubAuthor(std::string authorName)
{
  std_msgs::String authorMsg;
  authorMsg.data = authorName;
  authorPub.publish(authorMsg);
  author = authorName;
}

// csv commands file parser
std::map<std::string, std::string> commands;
void loadCommandsFile(std::string path)
{
  ifstream infile(path.c_str());
  if (!infile)
  {
    ROS_ERROR("Can not open the commands file, please check the path [%s]!", path.c_str());
    exit(1);
  }

  string sLine;
  CSVParser parser;
  while (!infile.eof())
  {
    getline(infile, sLine); // Get a line
    if (sLine == "")
      continue;

    parser << sLine; // Feed the line to the parser

    // Now extract the columns from the line
    int id;
    std::string command, speech, response;
    parser >> id >> command >> speech >> response;

    // save cmd into cmds
    commands.insert(map<std::string, std::string>::value_type(command, response));
  }
  infile.close();

  ROS_INFO("Loaded [%d] commands.", commands.size());
}

int sec = 0;
std::string cmd;
// skip the duplicate command in 1 secs. check success will return true
bool checkCmd(std::string tcmd)
{
  int tsec = ros::Time::now().sec;
  if (tsec - sec < 1)
  {
    std_msgs::String response;
    response.data = "Reject commands " + tcmd;
    speaker.publish(response);
    return false;
  }
  cmd = tcmd;
  sec = tsec;
  std_msgs::String response;
  response.data = commands.find(tcmd)->second;
  speaker.publish(response);
  return true;
}

void recogCallback(const std_msgs::String::ConstPtr& msg)
{
  std::string cmd = msg->data;
  bool skipFlag = true;

  // program setting command
  if (cmd == "continue")
  {
    if (checkCmd("continue"))
      paused = false;
  }
  else if (cmd == "pause")
  {
    if (checkCmd("pause"))
    {
      paused = true;
      pubGoalVel(0, 0, 0);
      pubAuthor("center");
    }
  }
  else
    skipFlag = false;
  if (paused)
    return;

  // common commands without "center" author privilege
  if (skipFlag)
    return;
  else
    skipFlag = true;
  if (cmd == "stop")
  {
    if (checkCmd("stop"))
    {
      pubAuthor("center");
      pubGoalVel(0, 0, 0);
    }
  }
  else if (cmd == "follow me")
  {
    if (checkCmd("follow me"))
      pubGoalVel(0, 0, 0);
    pubAuthor("follower");
  }
  else if (cmd == "go your self")
  {
    if (checkCmd("go your self"))
      pubGoalVel(0, 0, 0);
    pubAuthor("avoidance");
  }
  else if (cmd == "slower")
  {
    if (checkCmd("slower"))
    {
      // empty
    }
  }
  else if (cmd == "faster")
  {
    if (checkCmd("faster"))
    {
      // empty
    }
  }
  else if (cmd == "turn left")
  {
    if (checkCmd("turn left"))
    {
      pubGoalVel(0, 0, angularSpeed);
    }
  }
  else if (cmd == "turn right")
  {
    if (checkCmd("turn right"))
    {
      pubGoalVel(0, 0, -angularSpeed);
    }
  }
  else if (cmd == "reset speed")
  {
    if (checkCmd("reset speed"))
    {
      // empty
    }
  }
  else if (cmd == "hello")
  {
    if (checkCmd("hello"))
    {
      std::string stmStr("What is your command?");
      std_msgs::String msgStr;
      msgStr.data = stmStr;
      speaker.publish(msgStr);
    }
  }
  else if (cmd == "gopanpan")
  {
    if (checkCmd("gopanpan"))
    {
      geometry_msgs::PoseStamped pose;
      pose.header.frame_id = "map";
      pose.pose.position.x = -6; //-1.96; //-7.69; //-7.2;
      pose.pose.position.y = 0.5;//-3.61; //-3.55; //-3.8;
      pose.pose.orientation.w = 1;
      simpleGoal.publish(pose);
    }
  }
  else if (cmd == "gofrontdoor")
  {
    if (checkCmd("gofrontdoor"))
    {
      geometry_msgs::PoseStamped pose;
      pose.header.frame_id = "map";
      pose.pose.position.x = 1.72;
      pose.pose.position.y = -7.52;
      pose.pose.orientation.z = -0.707;
      pose.pose.orientation.w = 0.707;
      simpleGoal.publish(pose);
    }
  }
  else if (cmd == "gobackdoor")
  {
    if (checkCmd("gobackdoor"))
    {
      geometry_msgs::PoseStamped pose;
      pose.header.frame_id = "map";
      pose.pose.position.x = -9.65;
      pose.pose.position.y = -7.19;
      pose.pose.orientation.z = -0.707;
      pose.pose.orientation.w = 0.707;
      simpleGoal.publish(pose);
    }
  }
  else if (cmd == "goyutou")
  {
    if (checkCmd("goyutou"))
    {
      geometry_msgs::PoseStamped pose;
      pose.header.frame_id = "map";
      pose.pose.position.x = -1.5;
      pose.pose.position.y = -3.6;
      pose.pose.orientation.w = 1;
      simpleGoal.publish(pose);
    }
  }
  else if (cmd == "goback")
  {
    if (checkCmd("goback"))
    {
      geometry_msgs::PoseStamped pose;
      pose.header.frame_id = "map";
      pose.pose.orientation.w = 1;
      simpleGoal.publish(pose);
    }
  }
  else if (cmd == "come here")
    {
      if (checkCmd("come here"))
      {
        geometry_msgs::PoseStamped pose;
        pose.header.frame_id = "map";
        pose.pose.orientation.z = 1;
        simpleGoal.publish(pose);
      }
    }
  else if (cmd == "gotobanana")
  {
    if (checkCmd("gotobanana"))
    {
      geometry_msgs::PoseStamped pose;
      pose.header.frame_id = "map";
      pose.pose.position.x = 4.5;
      pose.pose.position.y = 0.6;
      pose.pose.orientation.z = 0.0;
      pose.pose.orientation.w = 1.0;
      simpleGoal.publish(pose);
    }
  }
  else if (cmd == "gotopear")
  {
    if (checkCmd("gotopear"))
    {
      geometry_msgs::PoseStamped pose;
      pose.header.frame_id = "map";
      pose.pose.position.x = 5.4; //0.7;
      pose.pose.position.y = 0.0; //-3.0;
      pose.pose.orientation.z = 0.0; //0.707;
      pose.pose.orientation.w = 1.0; //0.707;
      simpleGoal.publish(pose);
    }
  }
  else if (cmd == "gotoapple")
    {
      if (checkCmd("gotoapple"))
      {
        geometry_msgs::PoseStamped pose;
        pose.header.frame_id = "map";
        pose.pose.position.x = 4.7;
        pose.pose.position.y = -3.3;
        pose.pose.orientation.z = 0.707;
        pose.pose.orientation.w = 0.707;
        simpleGoal.publish(pose);
      }
    }
  else
    skipFlag = false;

  // check controlling author
  if (author != "center")
    return;

  // "center" authored commands
  if (skipFlag)
    return;
  else
    skipFlag = true;
  if (cmd == "go straight")
  {
    if (checkCmd("go straight"))
    {
      pubGoalVel(linearSpeed, 0, 0);
      pubAuthor("center");
    }
  }
  else if (cmd == "backward")
  {
    if (checkCmd("backward"))
    {
      pubGoalVel(-linearSpeed, 0, 0);
      pubAuthor("center");
    }
  }
  else if (cmd == "rotate left")
  {
    if (checkCmd("rotate left"))
    {
      pubGoalVel(0, 0, angularSpeed);
      pubAuthor("center");
    }
  }
  else if (cmd == "rotate right")
  {
    if (checkCmd("rotate right"))
    {
      pubGoalVel(0, 0, -angularSpeed);
      pubAuthor("center");
    }
  }
  else
  {
    ROS_WARN("Unknown command [%s]", cmd.c_str());
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "cmd_center");
  ros::NodeHandle nh;
  ros::Subscriber recogSub = nh.subscribe("/recognizer/output", 1, recogCallback);
  authorPub = nh.advertise<std_msgs::String>("/cmd_center/author", 1);
  goalVelPub = nh.advertise<geometry_msgs::Twist>("/goal_vel", 1);
  goalVelMaskPub = nh.advertise<intel_ros::TwistWithMask>("/goal_vel_mask", 1);
  speaker = nh.advertise<std_msgs::String>("voice_synthesis", 1);
  simpleGoal = nh.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1);

  // Get params
  ros::NodeHandle ph("~");
  ph.param("paused", paused, false);
  ph.param("linearSpeed", linearSpeed, 0.2);
  ph.param("angularSpeed", angularSpeed, 0.4);
  ph.param("author", author, std::string("center"));
  std::string csvPath;
  ph.param("cmd_csv_path", csvPath, std::string("please/set/the/path.csv"));

  // load commands
  loadCommandsFile(csvPath);

  ros::spin();
  return 0;
}
