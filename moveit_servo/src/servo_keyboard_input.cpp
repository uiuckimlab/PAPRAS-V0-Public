#include <ros/ros.h>
#include <geometry_msgs/TwistStamped.h>
#include <control_msgs/JointJog.h>

#include <signal.h>
#include <stdio.h>
#include <termios.h>
#include <unistd.h>

// Define used keys
#define KEYCODE_RIGHT 0x43
#define KEYCODE_LEFT 0x44
#define KEYCODE_UP 0x41
#define KEYCODE_DOWN 0x42
#define KEYCODE_PERIOD 0x2E
#define KEYCODE_SEMICOLON 0x3B
#define KEYCODE_1 0x31
#define KEYCODE_2 0x32
#define KEYCODE_3 0x33
#define KEYCODE_4 0x34
#define KEYCODE_5 0x35
#define KEYCODE_6 0x36
#define KEYCODE_7 0x37
#define KEYCODE_Q 0x71
#define KEYCODE_W 0x77
#define KEYCODE_E 0x65
#define KEYCODE_R 0x72
#define KEYCODE_A 0x61
#define KEYCODE_Z 0x7a
#define KEYCODE_S 0x73
#define KEYCODE_X 0x78
#define KEYCODE_D 0x64
#define KEYCODE_C 0x63

// Some constants used in the Servo Teleop demo
const std::string TWIST_TOPIC = "/left_arm_servo_server/delta_twist_cmds";
const std::string JOINT_TOPIC = "/left_arm_servo_server/delta_joint_cmds";
const size_t ROS_QUEUE_SIZE = 100;
const std::string EEF_FRAME_ID = "robot1/end_effector_link";
const std::string BASE_FRAME_ID = "robot1/link1";
const float CMD_VEL = 0.5;

// A class for reading the key inputs from the terminal
class KeyboardReader
{
public:
  KeyboardReader() : kfd(0)
  {
    // get the console in raw mode
    tcgetattr(kfd, &cooked);
    struct termios raw;
    memcpy(&raw, &cooked, sizeof(struct termios));
    raw.c_lflag &= ~(ICANON | ECHO);
    // Setting a new line, then end of file
    raw.c_cc[VEOL] = 1;
    raw.c_cc[VEOF] = 2;
    tcsetattr(kfd, TCSANOW, &raw);
  }
  void readOne(char* c)
  {
    int rc = read(kfd, c, 1);
    if (rc < 0)
    {
      throw std::runtime_error("read failed");
    }
  }
  void shutdown()
  {
    tcsetattr(kfd, TCSANOW, &cooked);
  }

private:
  int kfd;
  struct termios cooked;
};

// Converts key-presses to Twist or Jog commands for Servo, in lieu of a controller
class KeyboardServo
{
public:
  KeyboardServo();
  int keyLoop();

private:
  void spin();

  ros::NodeHandle nh;

  ros::Publisher twist_pub_;
  ros::Publisher joint_pub_;

  std::string frame_to_publish_;
  double joint_vel_cmd_;
};

KeyboardServo::KeyboardServo() : frame_to_publish_(BASE_FRAME_ID), joint_vel_cmd_(1.0)
{
  twist_pub_= nh.advertise<geometry_msgs::TwistStamped>(TWIST_TOPIC, ROS_QUEUE_SIZE);
  joint_pub_= nh.advertise<control_msgs::JointJog>(JOINT_TOPIC, ROS_QUEUE_SIZE);
}

KeyboardReader input;

void quit(int sig)
{
  (void)sig;
  input.shutdown();
  ros::shutdown();
  exit(0);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "servo_keyboard_input");
  ros::AsyncSpinner spinner(0);
  spinner.start();

  KeyboardServo keyboard_servo;

  signal(SIGINT, quit);

  int rc = keyboard_servo.keyLoop();
  input.shutdown();
  ros::shutdown();

  return rc;
}

void KeyboardServo::spin()
{
  ros::waitForShutdown();
}

int KeyboardServo::keyLoop()
{
  char c;
  bool publish_twist = false;
  bool publish_joint = false;

  std::bind(&KeyboardServo::spin, this);

  puts("Reading from keyboard");
  puts("---------------------------");
  puts("Use arrow keys and the '.' and ';' keys to Cartesian jog");
  puts("Use 'W' to Cartesian jog in the world frame, and 'E' for the End-Effector frame");
  puts("Use 1|2|3|4|5|6|7 keys to joint jog. 'R' to reverse the direction of jogging.");
  puts("'Q' to quit.");

  for (;;)
  {
    // get the next event from the keyboard
    try
    {
      input.readOne(&c);
    }
    catch (const std::runtime_error&)
    {
      perror("read():");
      return -1;
    }

    // // Create the messages we might publish
    geometry_msgs::TwistStamped twist_msg;
    control_msgs::JointJog joint_msg;

    // Use read key-press
    switch (c)
    {
      case KEYCODE_LEFT:
        twist_msg.twist.linear.y = -CMD_VEL;
        publish_twist = true;
        break;
      case KEYCODE_RIGHT:
        twist_msg.twist.linear.y = CMD_VEL;
        publish_twist = true;
        break;
      case KEYCODE_UP:
        twist_msg.twist.linear.x = CMD_VEL;
        publish_twist = true;
        break;
      case KEYCODE_DOWN:
        twist_msg.twist.linear.x = -CMD_VEL;
        publish_twist = true;
        break;
      case KEYCODE_PERIOD:
        twist_msg.twist.linear.z = -CMD_VEL;
        publish_twist = true;
        break;
      case KEYCODE_SEMICOLON:
        twist_msg.twist.linear.z = CMD_VEL;
        publish_twist = true;
        break;
      case KEYCODE_A:
        twist_msg.twist.angular.y = -CMD_VEL;
        publish_twist = true;
        break;
      case KEYCODE_Z:
        twist_msg.twist.angular.y = CMD_VEL;
        publish_twist = true;
        break;
      case KEYCODE_S:
        twist_msg.twist.angular.x = CMD_VEL;
        publish_twist = true;
        break;
      case KEYCODE_X:
        twist_msg.twist.angular.x = -CMD_VEL;
        publish_twist = true;
        break;
      case KEYCODE_D:
        twist_msg.twist.angular.z = -CMD_VEL;
        publish_twist = true;
        break;
      case KEYCODE_C:
        twist_msg.twist.angular.z = CMD_VEL;
        publish_twist = true;
        break;
      case KEYCODE_E:
        frame_to_publish_ = EEF_FRAME_ID;
        break;
      case KEYCODE_W:
        frame_to_publish_ = BASE_FRAME_ID;
        break;
      case KEYCODE_1:
        joint_msg.joint_names.push_back("robot1/joint1");
        joint_msg.velocities.push_back(joint_vel_cmd_);
        publish_joint = true;
        break;
      case KEYCODE_2:
        joint_msg.joint_names.push_back("robot1/joint2");
        joint_msg.velocities.push_back(joint_vel_cmd_);
        publish_joint = true;
        break;
      case KEYCODE_3:
        joint_msg.joint_names.push_back("robot1/joint3");
        joint_msg.velocities.push_back(joint_vel_cmd_);
        publish_joint = true;
        break;
      case KEYCODE_4:
        joint_msg.joint_names.push_back("robot1/joint4");
        joint_msg.velocities.push_back(joint_vel_cmd_);
        publish_joint = true;
        break;
      case KEYCODE_5:
        joint_msg.joint_names.push_back("robot1/joint5");
        joint_msg.velocities.push_back(joint_vel_cmd_);
        publish_joint = true;
        break;
      case KEYCODE_6:
        joint_msg.joint_names.push_back("robot1/joint6");
        joint_msg.velocities.push_back(joint_vel_cmd_);
        publish_joint = true;
        break;
      case KEYCODE_R:
        joint_vel_cmd_ *= -1;
        break;
      case KEYCODE_Q:
        return 0;
    }

    // If a key requiring a publish was pressed, publish the message now
    if (publish_twist)
    {
      twist_msg.header.stamp = ros::Time::now();
      twist_msg.header.frame_id = frame_to_publish_;
      twist_pub_.publish(twist_msg);
      publish_twist = false;
    }
    else if (publish_joint)
    {
      joint_msg.header.stamp = ros::Time::now();
      joint_msg.header.frame_id = BASE_FRAME_ID;
      joint_pub_.publish(joint_msg);
      publish_joint = false;
    }
  }

  return 0;
}