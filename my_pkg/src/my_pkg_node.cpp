#include <ros/ros.h>
#include <geometry_msgs/TwistStamped.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>

ros::Subscriber subSpeed;
ros::Publisher pubOldTwisr;
ros::Subscriber subImu;
ros::Publisher pubImu;
ros::Subscriber subPoints;
ros::Publisher pubPoints;
bool time_init_flag = false;
ros::Time d_time;
//float limit_l_max=0.5;
//float limit_a_max=0.4;
//float limit_l_min=0.2;
//float limit_a_min=0.2;
void speedHandler(const geometry_msgs::TwistStamped::ConstPtr &speedIn)
{
    geometry_msgs::Twist twist;
    twist.linear.x = speedIn->twist.linear.x*1;
    twist.linear.y = speedIn->twist.linear.y*0;
    twist.linear.z = speedIn->twist.linear.z*0;

    twist.angular.x = speedIn->twist.angular.x*0;
    twist.angular.y = speedIn->twist.angular.y*0;
    twist.angular.z = speedIn->twist.angular.z*1;
    
    //if(twist.linear.x >= limit_l_max) twist.linear.x = limit_l_max;
    //if(twist.linear.x <= -limit_l_max) twist.linear.x = -limit_l_max;
    //if(twist.angular.z >= limit_a_max) twist.angular.z = limit_a_max;
    //if(twist.angular.z <= -limit_a_max) twist.angular.z = -limit_a_max;
    
    //if((twist.linear.x <= limit_l_min) && (twist.linear.x > 0)) twist.linear.x = limit_l_min;
    //if((twist.linear.x >= -limit_l_min) && (twist.linear.x < 0)) twist.linear.x = -limit_l_min;
    //if((twist.angular.z <= limit_a_min) && (twist.angular.z > 0)) twist.angular.z = limit_a_min;
    //if((twist.angular.z >= -limit_a_min) && (twist.angular.z < 0)) twist.angular.z = -limit_a_min;
    
    pubOldTwisr.publish(twist);
    

    double l = 1;
    double vehicleSpeed = speedIn->twist.linear.x;
    double vehicleYawRate = speedIn->twist.angular.z;

    double vr = vehicleSpeed + l * vehicleYawRate / 2;
    double vl = vehicleSpeed - l * vehicleYawRate / 2;
}
void ImuCallback(const sensor_msgs::Imu::ConstPtr &imuIn)
{

    sensor_msgs::Imu imu;
    imu = *imuIn;
    if (!time_init_flag)
    {
        time_init_flag = true;
        ros::Time time_now = ros::Time::now();

        if (time_now.nsec < imu.header.stamp.nsec)
        {
            d_time.sec = time_now.sec - imu.header.stamp.sec - 1;
            d_time.nsec = 1000000000 + time_now.nsec - imu.header.stamp.nsec;
        }
        else
        {
            d_time.sec = time_now.sec - imu.header.stamp.sec;
            d_time.nsec = time_now.nsec - imu.header.stamp.nsec;
        }
    }
    if (imu.header.stamp.nsec + d_time.nsec > 999999999)
    {
        imu.header.stamp.sec += d_time.sec + 1;
        imu.header.stamp.nsec += d_time.nsec - 1000000000;
    }
    else
    {
        imu.header.stamp.sec += d_time.sec;
        imu.header.stamp.nsec += d_time.nsec;
    }

    pubImu.publish(imu);
}

// void PointsCallback(const sensor_msgs::PointCloud2::ConstPtr &pointIn)
// {

    // sensor_msgs::PointCloud2 points;

//    if (!time_init_flag)
//     {
//         time_init_flag = true;
//         ros::Time time_now = ros::Time::now();

//         if (time_now.nsec < points.header.stamp.nsec)
//         {
//             d_time.sec = time_now.sec - points.header.stamp.sec - 1;
//             d_time.nsec = 1000000000 + time_now.nsec - points.header.stamp.nsec;
//         }
//         else
//         {
//             d_time.sec = time_now.sec - points.header.stamp.sec;
//             d_time.nsec = time_now.nsec - points.header.stamp.nsec;
//         }
//     }

//     if (points.header.stamp.nsec + d_time.nsec > 999999999)
//     {
//         points.header.stamp.sec += d_time.sec + 1;
//         points.header.stamp.nsec += d_time.nsec - 1000000000;
//     }
//     else
//     {
//         points.header.stamp.sec += d_time.sec;
//         points.header.stamp.nsec += d_time.nsec;
//     }

//     pubPoints.publish(pointIn);
// }

int main(int argc, char **argv)
{
    ros::init(argc, argv, "vehicleControl");
    ros::NodeHandle nh;
    //ros::NodeHandle nhPrivate;
    
    //nhPrivate.getParam("limit_l_max", limit_l_max);
    //nhPrivate.getParam("limit_a_max", limit_a_max);
    //nhPrivate.getParam("limit_l_min", limit_l_min);
    //nhPrivate.getParam("limit_a_min", limit_a_min);

    subSpeed = nh.subscribe<geometry_msgs::TwistStamped>("/cmd_vel", 5, speedHandler);
    pubOldTwisr = nh.advertise<geometry_msgs::Twist>("/cmd_ve", 5);
    subImu = nh.subscribe<sensor_msgs::Imu>("/os_cloud_node/imu", 5, ImuCallback);
    pubImu = nh.advertise<sensor_msgs::Imu>("/imu", 5);
    // subPoints = nh.subscribe<sensor_msgs::PointCloud2>("/os_cloud_node/points", 2, PointsCallback);
    // pubPoints = nh.advertise<sensor_msgs::PointCloud2>("/points", 2);

    ros::Rate rate(10);
    bool status = ros::ok();
    while (status)
    {
        ros::spinOnce();
        status = ros::ok();
        rate.sleep();
    }
    
}
