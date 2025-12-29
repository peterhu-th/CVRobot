include<ros / ros.h>
#include <std_msgs/Int32MultiArray.h>
#include <geometry_msgs/Twist.h>
    geometry_msgs::Twist vel_cmd;
static int arADVal[15];
void AD_Callback(const std_msgs::Int32MultiArray msg)
{
    if (msg.data.size() < 15)
        return;
    for (int i = 0; i < 15; i++)
    {
        arADVal[i] = msg.data[i];
    }
}
int main(int argc, char **argv)
{
    ros::init(argc, argv, " avoid_obstacles");
    ros::NodeHandle n;
    ros::Subscriber sub_ad = n.subscribe("/wpb_cv/ad", 100, AD_Callback);
    ros::Publisher vel_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
    ros::Rate loop_rate(2);
    int nCountToStop = 0;
    while (ros::ok())
    {
        switch (state)
        {
        case 0:
            vel_cmd.linear.x = 0;
            vel_cmd.linear.y = 0;
            vel_cmd.linear.z = 0;
            vel_cmd.angular.x = 0;
            vel_cmd.angular.y = 0;
            vel_cmd.angular.z = 0;
            vel_pub.publish(vel_cmd);
            printf("[robot] initialized\n");
            state++;
            break;
        case 1:
            vel_cmd.linear.x = 0.1;
            vel_pub.publish(vel_cmd);
            printf("[robot] moving foward\n");
            if (arADVal[0] > 700 || arADVal[1] > 700)
            {
                vel_cmd.linear.x = 0;
                state++;
                break;
            }
        case 2:
            printf("[robot] turn right and begin circling\n")
            vel_cmd.angular.z = -0.6;
            int cnt = 0;
            int round_cnt = 0;
            if (arADVal[0] > 700)
            {
                vel_cmd.linear.x = 0;
                vel_cmd.angular.z = -0.3;
                vel_cmd.linear.x = 0.1;
                cnt++;
                round_cnt--;
                printf("[robot] obstacle detected for the %d time\n", cnt);
            }else{
                vel_cmd.angular.z = -0.3;
                round_cnt++;
            }
            if (round_cnt == 10 || round_cnt > 10)
            {
                state++;
            }
            break;
        case 3:
            vel_cmd.angular.z = 0.6;
            vel_cmd.linear.x = 0.1;
            if (clock)
            {
               state++;
            }
            ("[robot] mission completed, moving back\n");

        case 4:
             vel_cmd.linear.x = 0;
            vel_cmd.linear.y = 0;
            vel_cmd.linear.z = 0;
            vel_cmd.angular.x = 0;
            vel_cmd.angular.y = 0;
            vel_cmd.angular.z = 0;
            vel_pub.publish(vel_cmd);
            printf("[robot] stop\n");
            state++;
            break;
        }

        if (nCountToStop > 600)
        {
            vel_cmd.linear.x = 0;
            vel_cmd.angular.z = 0;
            ROS_WARN("TIME_OUT!");
        }
        vel_pub.publish(vel_cmd);
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}