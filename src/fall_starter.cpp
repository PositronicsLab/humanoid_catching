#include <ros/ros.h>
#include <std_msgs/Header.h>
#include <signal.h>
#include <termios.h>
#include <stdio.h>


namespace
{

using namespace std;
using namespace ros;

int kfd = 0;
struct termios cooked, raw;
static const char KEYCODE_ENTER = 10;

class FallStarter
{
private:

    //! Node handle
    ros::NodeHandle nh;

    //! Private nh
    ros::NodeHandle pnh;

    //! Catching controller notifier
    ros::Publisher catchNotifier;

public:
    FallStarter() :
        pnh("~")
    {
        catchNotifier = nh.advertise<std_msgs::Header>("/human/fall", 1, true);
    }

    void fall()
    {
        ROS_DEBUG("Notifying fall catcher");
        std_msgs::Header header;
        header.stamp = ros::Time::now();
        catchNotifier.publish(header);
        ROS_DEBUG("Fall initiation complete");
    }

    void keyLoop()
    {
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
        puts("Press Enter to initiate the fall");

        for(;;)
        {
            char c;

            // get the next event from the keyboard
            if(read(kfd, &c, 1) < 0)
            {
                perror("read():");
                exit(-1);
            }

            switch(c)
            {
                ROS_INFO("Character %c", c);
            case KEYCODE_ENTER:
                ROS_DEBUG("Enter key pressed");
                fall();
                tcsetattr(kfd, TCSANOW, &cooked);
                return;
            default:
                ROS_DEBUG("Unknown key pressed");
                break;
            }
        }
    }
};
}

void quit(int sig)
{
    tcsetattr(kfd, TCSANOW, &cooked);
    exit(0);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "fall_starter");
    FallStarter fs;
    signal(SIGINT, quit);
    fs.keyLoop();
    ros::Duration(3.0).sleep();
    return 0;
}
