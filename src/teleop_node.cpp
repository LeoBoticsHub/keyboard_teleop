#include <ros/ros.h>
#include <geometry_msgs/TwistStamped.h>

#include <unistd.h>
#include <fcntl.h>
#include <stdio.h>
#include <termios.h>

// For non-blocking keyboard inputs
int getch(void)
{
  fcntl(1, F_SETFL, fcntl(0, F_GETFL) | O_NONBLOCK);

  static struct termios oldt, newt;
  tcgetattr( STDIN_FILENO, &oldt);           // save old settings

  int c = getchar();  // read character (non-blocking)
  int discard;
  while ((discard = getchar()) != EOF);


  newt = oldt;
  newt.c_lflag &= ~(ICANON);                 // disable buffering
  tcsetattr( STDIN_FILENO, TCSANOW, &newt);  // apply new settings

  tcsetattr( STDIN_FILENO, TCSANOW, &oldt);  // restore old settings
  return c;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "teleop_node");

  std::string ns = "keyboard_teleop";
  ros::NodeHandle n(ns);

  // variable init
  unsigned int seq{0};
  double vx{0}, vy{0}, w{0};

  // Ros parameters loading
  std::string cmd_vel_topic;
  n.getParam("cmd_vel_topic_name", cmd_vel_topic);

  std::string cmd_vel_father_frame_id;
  n.getParam("cmd_vel_father_frame_id", cmd_vel_father_frame_id);

  double max_v, max_w;
  n.getParam("max_lin_vel", max_v);
  n.getParam("max_ang_vel", max_w);

  double lin_speed{0.5}, ang_speed{0.5};
  constexpr double zero_speed{0.0};

  // create a publisher to car_frame/velocity_reference to send the velocity command to the robot
  ros::Publisher pub = n.advertise<geometry_msgs::TwistStamped>(cmd_vel_topic, 100);


  printf("-----------------------------------------\n"
         "-           Keyboard Teleop             -\n"
         "-----------------------------------------\n"
         "\n\nTo use the teleop use the following usage:\n"
         "w: move forward\ns: move backward\nd: turn right\na: turn left\n"
         "e: move right\nq: move left\n"
         "Using a composition of non opposite keys "
         "(e.g., w+d, NOT w+s) the robot will compose the velocity commands.");

  int key_cmd{' '};
  geometry_msgs::TwistStamped cmd_vel_msg;
  ros::Rate loop_rate(10);
  while (ros::ok())
  {

    key_cmd = getch();

    printf("command: %c\n", key_cmd);

    switch (key_cmd) {

    // move forward
    case 'w':
      vx = lin_speed;
      break;

      // move backward
    case 's':
      vx = -lin_speed;
      break;

      // stop robot
    case 'x':
      vx = zero_speed;
      vy = zero_speed;
      w = zero_speed;
      break;

      // turn right
    case 'd':
      w = -ang_speed;
      break;

      // turn left
    case 'a':
      w = ang_speed;
      break;

      // move left
    case 'q':
      vy = -lin_speed;
      break;

      // move right
    case 'e':
      vy = lin_speed;
      break;

    }

    if(key_cmd == 3)
      break;

    // writing commands on ros message
    cmd_vel_msg.header.frame_id = cmd_vel_father_frame_id;
    cmd_vel_msg.header.seq      = seq;
    cmd_vel_msg.header.stamp    = ros::Time::now();
    cmd_vel_msg.twist.linear.x  = vx;
    cmd_vel_msg.twist.linear.y  = vy;
    cmd_vel_msg.twist.angular.z = w;

    // publish message on topic
    pub.publish(cmd_vel_msg);

    vx = 0;
    vy = 0;
    w = 0;

    seq++;
    key_cmd = ' ';
    ros::spinOnce();

    loop_rate.sleep();
  }

  return 0;
}
