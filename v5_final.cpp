#include <ros/ros.h>
#include <std_msgs/Int32MultiArray.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Imu.h>
#include <cmath>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>

geometry_msgs::Twist vel_cmd;

class ObstacleAvoider
{

private:
    // 状态枚举
    enum State // 定义任务的不同阶段
    {
        INIT = 1, // 初始化
        APPROACH, // 接近障碍物
        CIRCLE,   // 逆时针环绕障碍物
        RETURN,   // 返程
        STOP,     // 停止
        ERROR     // 错误
    };
    State current_state; // 状态机变量，用于切换状态
    State last_state;    // 上一个状态，用于ERROR报告

    // ros相关
    ros::NodeHandle n; // 节点处理对象
    ros::Subscriber imu_sub;
    ros::Subscriber ad_sub; // 订阅对象
    ros::Publisher vel_pub; // 广播对象
    ros::Rate loop_rate;    // 控制频率

    tf::TransformBroadcaster odom_broadcaster;
    tf::Transform transform;

    // 时间相关
    double mission_start_time;
    double mission_end_time;
    double total_duration; // 任务用时
    double state_start_time;
    double last_state_start_time;
    double state_duration; // 阶段用时
    double current_time;
    double last_time;
    double loop_time;          // 单位时间
    double target_return_time; // 返回阶段目标用时，由APPROACH决定
    double state_elapsed;      // 现阶段用时，不断更新

    bool imu_data_available;
    double current_yaw;       // 当前角度
    double last_yaw;          // 上次角度
    double initial_yaw;       // 初始角度
    double delta_angle;       // 相邻两次IMU读数差
    double accumulated_angle; // 累计角度

    // 常量
    const int OBSTACLE_THRESHOD_FRONT = 700; // 障碍物检测阈值，值越大，距离阈值越小
    const int OBSTACLE_THRESHODE_SIDE_MIN = 850;
    const int OBSTACLE_THRESHODE_SIDE_MAX = 950;
    const double LINEAR_SPEED = 0.1;  // 线速度,m/s
    const double ANGULAR_SPEED = 0.2; // 角速度,rad/s
    const double PI = 3.14159265358979323846;
    const double TARGET_CONSEQUENSE_ANGLE = 1.5 * PI; // 360度
    const double TARGET_TURN_ANGLE = 1.0 / 3.0 * PI;  // 60度

    // 阶段限时
    const double INIT_MAXTIME = 1.0;
    const double APPROACH_MAXTIME = 15.0;
    const double TURN_MAXTIME = 10.0;
    const double CIRCLE_MAXTIME = 120.0;
    const double RETURN_MAXTIME = 15.0;

    // 传感器数据
    int left_sensor;      // 前端左侧
    int right_sensor;     // 前端右侧
    int left_side_sensor; // 左侧面

    // 传感器回调函数，仅前两位有意义
    void AD_Callback(const std_msgs::Int32MultiArray::ConstPtr &msg)
    {
        if (msg->data.size() != 15)
        {
            return; // 筛选有效信息
        }

        left_sensor = msg->data[0];
        right_sensor = msg->data[1];
        left_side_sensor = msg->data[2];
    }

    // IMU传感器回调函数
    void imuCallback(const sensor_msgs::Imu::ConstPtr &msg)
    {
        transform.setOrigin(tf::Vector3(0, 0, 0));
        tf::Quaternion q(msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w);
        last_yaw = current_yaw;
        current_yaw = tf::getYaw(q);
        angleCalculator();
        transform.setRotation(q);
        odom_broadcaster.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "imu_base", "base_footprint"));
        imu_data_available = true;
    }

    void angleCalculator()
    {
        delta_angle = current_yaw - last_yaw;
        if (abs(delta_angle) < 0.1)
        {
            accumulated_angle += delta_angle;
        }
    }

    // 发布命令到控制器
    void publish(double linear_x, double angular_z)
    {
        geometry_msgs::Twist vel_cmd;
        vel_cmd.linear.x = linear_x;
        vel_cmd.linear.y = 0;
        vel_cmd.linear.z = 0;
        vel_cmd.angular.x = 0;
        vel_cmd.angular.y = 0;
        vel_cmd.angular.z = angular_z;
        vel_pub.publish(vel_cmd);
    }

    // 判断是否检测到障碍物
    bool obstacleDetected()
    {
        return (left_sensor > OBSTACLE_THRESHOD_FRONT || right_sensor > OBSTACLE_THRESHOD_FRONT);
    }

    void obstacleDealer()
    {
        if (obstacleDetected())
        {
            if (left_sensor > OBSTACLE_THRESHODE_FRONT && right_sensor > OBSTACLE_THRESHODE_FRONT) // 前端左右都有障碍，后退右转
            {
                publish(-2.0 * LINEAR_SPEED, -ANGULAR_SPEED);
                ROS_INFO("ObstacleDealer Controlling.");
            }
            else if (left_sensor > OBSTACLE_THRESHODE_FRONT && right_sensor <= OBSTACLE_THRESHODE_FRONT) // 前端左侧有障碍，右侧无障碍
            {
                if (left_side_sensor > OBSTACLE_THRESHODE_SIDE_MAX) // 左侧有障碍，前进右转
                {
                    publish(2.0 * LINEAR_SPEED, -2.0 * ANGULAR_SPEED);
                    ROS_INFO("ObstacleDealer Controlling.");
                }
                else if (left_side_sensor < OBSTACLE_THRESHODE_SIDE_MIN) // 左侧无障碍，后退右转
                {
                    publish(-4.0 * LINEAR_SPEED, -4.0 * ANGULAR_SPEED);
                    ROS_INFO("ObstacleDealer Controlling.");
                }
                else // 左侧障碍物距离适中，前进右转
                {
                    publish(2.0 * LINEAR_SPEED, -2.0 * ANGULAR_SPEED);
                    ROS_INFO("ObstacleDealer Controlling.");
                }
            }
            else if (right_sensor > OBSTACLE_THRESHODE_FRONT && left_sensor <= OBSTACLE_THRESHODE_FRONT) // 前端右侧有障碍，左侧无障碍，原地右转
            {
                publish(0, -ANGULAR_SPEED);
                ROS_INFO("ObstacleDealer Controlling.");
            }
            if (current_state == APPROACH)
            {
                updateState(CIRCLE);
            }
            ROS_INFO("Current Accumulated Angle Is %f", accumulated_angle);
        }
    }

    // 主要控制逻辑的实现函数
    void normalController()
    {
        state_elapsed = ros::Time::now().toSec() - state_start_time;
        switch (current_state)
        {
            /*
                case结构：
                    报告当前状态；
                    执行驱动任务；
                    记录必要参数；
                    更新状态；
                    超时跳转；

                理想的执行逻辑：
                    INIT——初始化；
                    APPROACH——前进直到检测到障碍物；
                    TURN_RIGHT_1——右转；
                    CIRCLE——逆时针环绕障碍物；
                    TURN_RIGHT_2——右转；
                    RETURN——返回原点；
                    STOP——停止
                若产生ERROR，则停下并报告产生ERROR的阶段
            */

        case INIT:
            ROS_INFO("Initializing...\n");
            stopRobot();

            if (loop_time > 1 || loop_time == 0)
            {
                current_time = ros::Time::now().toSec();
                loop_time = current_time - last_time;
                last_time = current_time;
            }

            if (mission_start_time == 0)
            {
                mission_start_time = state_start_time =
                    ros::Time::now().toSec(); // 由于INIT之前没有状态，需要额外添加以便计算时间
            }

            if (imu_data_available == true && mission_start_time != 0 && loop_time < 1 && loop_time != 0)
            {
                updateState(APPROACH);
            }
            else if (mission_start_time == 0)
            {
                ROS_INFO("Waiting for Loop Timer...");
            }
            else if (!imu_data_available)
            {
                ROS_INFO("Waiting for IMU data...");
            }
            else if (loop_time > 1 || loop_time == 0)
            {
                ROS_INFO("Waiting for Timer Initialized...");
            }

            if (state_elapsed > INIT_MAXTIME)
            {
                updateState(ERROR);
            }

            break;

        case APPROACH:
            ROS_INFO("Moving Forward...\n");
            publish(2.0 * LINEAR_SPEED, 0);
            ROS_INFO("Ros Controlling.");

            if (state_elapsed > APPROACH_MAXTIME)
            {
                ROS_WARN("Approach Timeout, No Obstacle Detected!\n");
                updateState(ERROR);
            }

            break;

        case CIRCLE:
            ROS_INFO("Start Circling...\n");

            if (left_side_sensor > OBSTACLE_THRESHODE_SIDE_MAX) // 左侧有障碍，前进右转
            {
                publish(2.0 * LINEAR_SPEED, -ANGULAR_SPEED);
                ROS_INFO("NormalController Controlling.");
            }
            else if (left_side_sensor < OBSTACLE_THRESHOD_SIDE_MIN) // 左侧无障碍，前进左转
            {
                publish(2.0 * LINEAR_SPEED, ANGULAR_SPEED);
                ROS_INFO("NormalController Controlling.");
            }
            else // 左侧障碍物距离适中，前进左转
            {
                publish(2.0 * LINEAR_SPEED, 4.0 * ANGULAR_SPEED);
                ROS_INFO("NormalController Controlling.");
            }

            if (accumulated_angle >= TARGET_CONSEQUENSE_ANGLE)
            {
                ROS_INFO("Circle Completed!\n");
                updateState(RETURN);
            }

            if (state_elapsed > CIRCLE_MAXTIME)
            {
                ROS_WARN("Cricle Timeout!\n");
                updateState(ERROR);
            }

            ROS_INFO("Current Accumulated Angle Is %f\n", accumulated_angle);

            break;

        case RETURN:
            ROS_INFO("Returning...\n");
            publish(2.0 * LINEAR_SPEED, 0);
            ROS_INFO("Ros Controlling.");

            if (state_elapsed >= target_return_time) // target_return_time即为APPROACH阶段前进时长
            {
                updateState(STOP);
            }

            if (state_elapsed > RETURN_MAXTIME)
            {
                ROS_WARN("Return Timeout!\n");
                updateState(ERROR);
            }

            break;

        case STOP:
            ROS_INFO("Mission Completed Successfully!\n");
            mission_end_time = ros::Time::now().toSec();
            total_duration = mission_end_time - mission_start_time;
            stopRobot();
            ROS_INFO("Total Duration: %f seconds\n", total_duration);

            if (state_elapsed > 5.0)
            {
                ROS_INFO("Shutting Down...");
                ros::shutdown();
            }
            
            break;

        case ERROR:
            ROS_INFO("Error Occurred at state %d\n!", last_state);
            stopRobot();

            if (state_elapsed > 5.0)
            {
                ros::shutdown();
            }

            break;
        }
    }

    // 更新状态，用于任务推进和ERROR处理
    void updateState(State new_state)
    {
        
        last_state_start_time = state_start_time;                  // 把上次读取的time赋值给last
        state_start_time = ros::Time::now().toSec();               // 更新当前time
        state_duration = state_start_time - last_state_start_time; // 用两次state起始时间差作为持续时间
        last_state = current_state;
        current_state = new_state;
        if (current_state == CIRCLE) // 此时记录的其实是APPROACH的用时
        {
            if (current_state == RETURN || current_state == STOP)
            {
                return;
            }
            target_return_time = state_duration; // APPOARCH需要记录时间作为RETURN的参考
            accumulated_angle = 0;
            ROS_INFO("Starting Circle with Accumulated Angle 0.");
        }
        ROS_INFO("State %d Lasted for %f\n", last_state, state_duration); // 输出state持续时间
        ROS_INFO("State Changed to %d", new_state);                       // 报告state更新状态
    }

    void stopRobot()
    {
        publish(0, 0);
        ROS_INFO("Stop Controlling.");
    }

public:
    // 构造函数，用于初始化机器人
    ObstacleAvoider() : loop_rate(10),
                        current_state(INIT),
                        last_state(INIT),
                        mission_start_time(0.0),
                        mission_end_time(0.0),
                        total_duration(0.0),
                        state_start_time(0.0),
                        last_state_start_time(0.0),
                        state_duration(0.0),
                        current_time(0.0),
                        last_time(0.0),
                        loop_time(0.0),
                        state_elapsed(0.0),
                        left_sensor(0),
                        right_sensor(0),
                        side_sensor(0),
                        current_yaw(0.0),
                        initial_yaw(0.0),
                        delta_angle(0.0),
                        last_yaw(0.0),
                        delta_angle(0.0),
                        imu_data_available(false)
    {

        // 创建advertise和subscribe
        vel_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
        ad_sub = n.subscribe("/wpb_cv/ad", 100, &ObstacleAvoider::AD_Callback, this); // 添加参数this
        imu_sub = n.subscribe("/imu/data_raw", 100, &ObstacleAvoider::imuCallback, this);

        stopRobot();
        ROS_INFO("ObstacleAvoider Initialized");
        state_start_time = mission_start_time = ros::Time::now().toSec();
    }

    void controllerManager()
    {
        if (obstacleDetected())
        {
            obstacleDealer();
            ROS_INFO("Obstacle Detected!");
        }
        else
        {
            normalController();
        }
        accumulated_angle = current_yaw - initial_yaw;
    }

    void spin()
    {
        while (ros::ok())
        {
            ros::spinOnce();
            controllerManager();
            loop_rate.sleep();
        }
    }

    ~ObstacleAvoider()
    {
        stopRobot();
        ROS_INFO("ObstacleAvoider Shutting Down.");
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "avoid_obstacles");
    printf("Programm Launch.");

    try
    {
        ObstacleAvoider avoider;
        avoider.spin();
    }

    catch (const std::exception &e)
    {
        ROS_ERROR("Exception: %s", e.what());
        return 1;
    }

    return 0;
}
