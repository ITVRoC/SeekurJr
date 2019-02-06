#include <termios.h>
#include <signal.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#include <ros/ros.h>
#include <p2os_msgs/PTZState.h>

// PTZ
#define KEYCODE_I 0x69
#define KEYCODE_K 0x6B
#define KEYCODE_J 0x6A
#define KEYCODE_L 0x6C
#define KEYCODE_U 0x75
#define KEYCODE_O 0x6F
#define KEYCODE_M 0x6D

#define KEYCODE_I_CAP 0x49
#define KEYCODE_K_CAP 0x4B
#define KEYCODE_J_CAP 0x4A
#define KEYCODE_L_CAP 0x4C
#define KEYCODE_U_CAP 0x55
#define KEYCODE_O_CAP 0x4F
#define KEYCODE_M_CAP 0x4D

class TeleopPioneerKeyboard
{
 private:
  double pan_rate_, tilt_rate_, zoom_rate_;
  double fast_pan_rate_, fast_tilt_rate_, fast_zoom_rate_;
  p2os_msgs::PTZState ptz_cmd_;

  ros::NodeHandle n_;
  ros::Publisher ptz_pub_; 

 public:
  void init()
  {
    ptz_cmd_.pan = ptz_cmd_.tilt = ptz_cmd_.zoom = 0;
    ptz_cmd_.relative = false;
    ptz_pub_ = n_.advertise<p2os_msgs::PTZState>("ptz_control", 1000);

  ros::NodeHandle n_private("~");
    n_private.param("pan_rate", pan_rate_, 5.0);
    n_private.param("tilt_rate", tilt_rate_, 5.0);
    n_private.param("zoom_rate", zoom_rate_, 196.0);
    n_private.param("fast_pan_rate", fast_pan_rate_, 20.0);
    n_private.param("fast_tilt_rate", fast_tilt_rate_, 20.0);
    n_private.param("fast_zoom_rate", fast_zoom_rate_, 392.0);
  }

  ~TeleopPioneerKeyboard()   { }
  void keyboardLoop();

};

// Nao sei do que se trata *******************
int kfd = 0;
struct termios cooked, raw;

void quit(int sig)
{
  tcsetattr(kfd, TCSANOW, &cooked);
  exit(0);
}
//  *********************************


int main(int argc, char** argv)
{
  ros::init(argc, argv, "pr2_base_keyboard");

  TeleopPioneerKeyboard tpk;
  tpk.init();

  signal(SIGINT,quit);

  tpk.keyboardLoop();

  return(0);
}


void TeleopPioneerKeyboard::keyboardLoop()
{
  char c;
  bool ptz_dirty = false;

  // get the console in raw mode
  tcgetattr(kfd, &cooked);
  memcpy(&raw, &cooked, sizeof(struct termios));
  raw.c_lflag &=~ (ICANON | ECHO);
  // Setting a new line, then end of file
  raw.c_cc[VEOL] = 1;
  raw.c_cc[VEOF] = 2;
  tcsetattr(kfd, TCSANOW, &raw);

  puts("Reading from keyboard");
  puts("---------------------------");
  puts("Use JL to pan");
  puts("Use KI to tilt");
  puts("Use UO to zoom");
  puts("Use CAPS LOOK to go faster");


  for(;;)
  {
    // get the next event from the keyboard
    if(read(kfd, &c, 1) < 0)
    {
      perror("read():");
      exit(-1);
    }

    // Clear the previous commands
    ptz_cmd_.pan = ptz_cmd_.tilt = ptz_cmd_.zoom = 0;
    ptz_dirty = false;

    switch(c)
    {
        // PTZ
      case KEYCODE_I:
        ptz_cmd_.tilt = tilt_rate_;
        ptz_cmd_.relative = true;
        ptz_dirty = true;
        break;
      case KEYCODE_K:
        ptz_cmd_.tilt = -tilt_rate_;
        ptz_cmd_.relative = true;
        ptz_dirty = true;
        break;
      case KEYCODE_J:
        ptz_cmd_.pan = -pan_rate_;
        ptz_cmd_.relative = true;
        ptz_dirty = true;
        break;
      case KEYCODE_L:
        ptz_cmd_.pan = pan_rate_;
        ptz_cmd_.relative = true;
        ptz_dirty = true;
        break;
      case KEYCODE_U:
        ptz_cmd_.zoom = -zoom_rate_;
        ptz_cmd_.relative = true;
        ptz_dirty = true;
        break;
      case KEYCODE_O:
        ptz_cmd_.zoom = zoom_rate_;
        ptz_cmd_.relative = true;
        ptz_dirty = true;
        break;
      case KEYCODE_I_CAP:
        ptz_cmd_.tilt = fast_tilt_rate_;
        ptz_cmd_.relative = true;
        ptz_dirty = true;
        break;
      case KEYCODE_K_CAP:
        ptz_cmd_.tilt = -fast_tilt_rate_;
        ptz_cmd_.relative = true;
        ptz_dirty = true;
        break;
      case KEYCODE_J_CAP:
        ptz_cmd_.pan = -fast_pan_rate_;
        ptz_cmd_.relative = true;
        ptz_dirty = true;
        break;
      case KEYCODE_L_CAP:
        ptz_cmd_.pan = fast_pan_rate_;
        ptz_cmd_.relative = true;
        ptz_dirty = true;
        break;
      case KEYCODE_U_CAP:
        ptz_cmd_.zoom = -fast_zoom_rate_;
        ptz_cmd_.relative = true;
        ptz_dirty = true;
        break;
      case KEYCODE_O_CAP:
        ptz_cmd_.zoom = fast_zoom_rate_;
        ptz_cmd_.relative = true;
        ptz_dirty = true;
        break;
      case KEYCODE_M:
        ptz_cmd_.pan = 0;
        ptz_cmd_.tilt = 0;
        ptz_cmd_.zoom = 0;
        ptz_cmd_.relative = false;
        ptz_dirty = true;
        break;
      default:
        break;
    }


    if (ptz_dirty)
    {
      ptz_pub_.publish(ptz_cmd_);
    }

  }
}

