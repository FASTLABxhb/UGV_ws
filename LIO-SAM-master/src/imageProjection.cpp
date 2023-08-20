#include "utility.h"
#include "lio_sam/cloud_info.h"

// wxx

#define D(x) std::cout<<#x<<"="<<x<<std::endl;

// Velodyne
struct PointXYZIRT
{
    PCL_ADD_POINT4D
    PCL_ADD_INTENSITY;
    uint16_t ring;
    float time;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT (PointXYZIRT,  
    (float, x, x) (float, y, y) (float, z, z) (float, intensity, intensity)
    (uint16_t, ring, ring) (float, time, time)
)

// Ouster
// struct PointXYZIRT {
//     PCL_ADD_POINT4D;
//     float intensity;
//     uint32_t t;
//     uint16_t reflectivity;
//     uint8_t ring;
//     uint16_t noise;
//     uint32_t range;
//     EIGEN_MAKE_ALIGNED_OPERATOR_NEW
// }EIGEN_ALIGN16;

// POINT_CLOUD_REGISTER_POINT_STRUCT(PointXYZIRT,
//     (float, x, x) (float, y, y) (float, z, z) (float, intensity, intensity)
//     (uint32_t, t, t) (uint16_t, reflectivity, reflectivity)
//     (uint8_t, ring, ring) (uint16_t, noise, noise) (uint32_t, range, range)
// )

void removeNaNFromPointCloud(pcl::PointCloud<PointXYZIRT>::Ptr laserCloudIn, pcl::PointCloud<PointXYZIRT>::Ptr laserCloudOut)
{
    // D(laserCloudIn.size());
    int count = 0;
    for( int i = 0; i < (int)laserCloudIn->size(); i++ )
    {
        if( laserCloudIn->points[i].ring != 100 )
        {
            laserCloudOut->points[count] = laserCloudIn->points[i];
            count++;
        }
    }
    laserCloudOut->points.resize(count);
    // D(count);
}

const int queueLength = 500;

class ImageProjection : public ParamServer
{
private:

    std::mutex imuLock;
    std::mutex odoLock;

    ros::Subscriber subLaserCloud;
    ros::Publisher  pubLaserCloud;
    
    ros::Publisher pubExtractedCloud;
    ros::Publisher pubRawSensorCloud;
    ros::Publisher pubLaserCloudInfo;

    ros::Subscriber subImu;
    std::deque<sensor_msgs::Imu> imuQueue;

    ros::Subscriber subOdom;
    std::deque<nav_msgs::Odometry> odomQueue;

    std::deque<sensor_msgs::PointCloud2> cloudQueue;
    sensor_msgs::PointCloud2 currentCloudMsg;
    
    double *imuTime = new double[queueLength];
    double *imuRotX = new double[queueLength];
    double *imuRotY = new double[queueLength];
    double *imuRotZ = new double[queueLength];
    double *imuPoseX = new double[queueLength];
    double *imuPoseY = new double[queueLength];
    double *imuPoseZ = new double[queueLength];
    double imuPoseXInit = 0;
    double imuPoseYInit = 0;
    double imuPoseZInit = 0;
    std::vector<Eigen::Quaterniond> imuRot_q;
    Eigen::Quaterniond init_q_inverse;

    int imuPointerCur;
    bool firstPointFlag;
    Eigen::Affine3f transStartInverse;
    Eigen::Quaterniond q_start;
    Eigen::Quaterniond q_start_inverse;
    Eigen::Vector3d t_start;

    pcl::PointCloud<PointXYZIRT>::Ptr laserCloudIn;
    // pcl::PointCloud<PointType>::Ptr laserCloudIn;
    pcl::PointCloud<PointType>::Ptr   fullCloud;
    pcl::PointCloud<PointType>::Ptr   extractedCloud;
    pcl::PointCloud<PointType>::Ptr   sensorRawCloud;

    int deskewFlag;
    cv::Mat rangeMat;

    bool odomDeskewFlag;
    float odomIncreX;
    float odomIncreY;
    float odomIncreZ;

    lio_sam::cloud_info cloudInfo;
    double timeScanCur;
    double timeScanNext;
    std_msgs::Header cloudHeader;


public:
    ImageProjection():
    deskewFlag(0)
    {
        imuRot_q.resize(queueLength);
        subImu        = nh.subscribe<sensor_msgs::Imu>("lio_sam/new_imu", 2000, &ImageProjection::imuHandler, this, ros::TransportHints().tcpNoDelay());
        // subImu        = nh.subscribe<sensor_msgs::Imu>(imuTopic, 2000, &ImageProjection::imuHandler, this, ros::TransportHints().tcpNoDelay());
        subOdom       = nh.subscribe<nav_msgs::Odometry>(odomTopic, 2000, &ImageProjection::odometryHandler, this, ros::TransportHints().tcpNoDelay());
        subLaserCloud = nh.subscribe<sensor_msgs::PointCloud2>(pointCloudTopic, 5, &ImageProjection::cloudHandler, this, ros::TransportHints().tcpNoDelay());

        pubExtractedCloud = nh.advertise<sensor_msgs::PointCloud2> ("lio_sam/deskew/cloud_deskewed", 1);
        pubRawSensorCloud = nh.advertise<sensor_msgs::PointCloud2> ("lio_sam/sensor_cloud", 1);

        pubLaserCloudInfo = nh.advertise<lio_sam::cloud_info> ("lio_sam/deskew/cloud_info", 10);

        allocateMemory();
        resetParameters();

        pcl::console::setVerbosityLevel(pcl::console::L_ERROR);
    }

    void allocateMemory()
    {
        laserCloudIn.reset(new pcl::PointCloud<PointXYZIRT>());
        // laserCloudIn.reset(new pcl::PointCloud<PointType>());
        fullCloud.reset(new pcl::PointCloud<PointType>());
        extractedCloud.reset(new pcl::PointCloud<PointType>());

        sensorRawCloud.reset(new pcl::PointCloud<PointType>());
        sensorRawCloud->points.resize(N_SCAN*Horizon_SCAN);

        fullCloud->points.resize(N_SCAN*Horizon_SCAN);

        cloudInfo.startRingIndex.assign(N_SCAN, 0);
        cloudInfo.endRingIndex.assign(N_SCAN, 0);

        cloudInfo.pointColInd.assign(N_SCAN*Horizon_SCAN, 0);
        cloudInfo.pointRange.assign(N_SCAN*Horizon_SCAN, 0);

        resetParameters();
    }

    void resetParameters()
    {
        laserCloudIn->clear();
        extractedCloud->clear();
        // reset range matrix for range image projection
        rangeMat = cv::Mat(N_SCAN, Horizon_SCAN, CV_32F, cv::Scalar::all(FLT_MAX));

        imuPointerCur = 0;
        firstPointFlag = true;
        odomDeskewFlag = false;

        imuPoseXInit = 0;
        imuPoseYInit = 0;
        imuPoseZInit = 0;

        Eigen::Quaterniond qtemp(1, 0, 0, 0);
        for (int i = 0; i < queueLength; ++i)
        {
            imuTime[i] = 0;
            imuRotX[i] = 0;
            imuRotY[i] = 0;
            imuRotZ[i] = 0;
            imuPoseX[i] = 0;
            imuPoseY[i] = 0;
            imuPoseZ[i] = 0;
            imuRot_q[i] = qtemp;
        }
    }

    ~ImageProjection(){}

    void imuHandler(const sensor_msgs::Imu::ConstPtr& imuMsg)
    {
        sensor_msgs::Imu thisImu = (*imuMsg);
        // sensor_msgs::Imu thisImu = imuConverter(*imuMsg);

        std::lock_guard<std::mutex> lock1(imuLock);
        imuQueue.push_back(thisImu);

        // debug IMU data
        // cout << std::setprecision(6);
        // cout << "IMU acc: " << endl;
        // cout << "x: " << thisImu.linear_acceleration.x << 
        //       ", y: " << thisImu.linear_acceleration.y << 
        //       ", z: " << thisImu.linear_acceleration.z << endl;
        // cout << "IMU gyro: " << endl;
        // cout << "x: " << thisImu.angular_velocity.x << 
        //       ", y: " << thisImu.angular_velocity.y << 
        //       ", z: " << thisImu.angular_velocity.z << endl;
        // double imuRoll, imuPitch, imuYaw;
        // tf::Quaternion orientation;
        // tf::quaternionMsgToTF(thisImu.orientation, orientation);
        // tf::Matrix3x3(orientation).getRPY(imuRoll, imuPitch, imuYaw);
        // cout << "IMU roll pitch yaw: " << endl;
        // cout << "roll: " << imuRoll << ", pitch: " << imuPitch << ", yaw: " << imuYaw << endl << endl;
    }

    void odometryHandler(const nav_msgs::Odometry::ConstPtr& odometryMsg)
    {
        std::lock_guard<std::mutex> lock2(odoLock);
        odomQueue.push_back(*odometryMsg);
    }

    void cloudHandler(const sensor_msgs::PointCloud2ConstPtr& laserCloudMsg)
    {
        if (!cachePointCloud(laserCloudMsg))
            return;

        if (!deskewInfo())
            return;

        projectPointCloud();

        cloudExtraction();

        publishClouds();

        resetParameters();
    }

    bool cachePointCloud(const sensor_msgs::PointCloud2ConstPtr& laserCloudMsgIn)
    {
        // cache point cloud
        sensor_msgs::PointCloud2 laserCloudMsg;
        laserCloudMsg = *laserCloudMsgIn;
	nh.param<double>("lio_sam/topicTimeOffset",topicTimeOffset,0.0);
        laserCloudMsg.header.stamp = ros::Time(laserCloudMsg.header.stamp.toSec() + topicTimeOffset);
laserCloudMsg.header.stamp= ros::Time::now();
        cloudQueue.push_back(laserCloudMsg);

        if (cloudQueue.size() <= 2)
            return false;
        else
        {
            currentCloudMsg = cloudQueue.front();
            cloudQueue.pop_front();

            cloudHeader = currentCloudMsg.header;
            timeScanCur = cloudHeader.stamp.toSec();
            timeScanNext = cloudQueue.front().header.stamp.toSec();
        }

        // convert cloud
        pcl::fromROSMsg(currentCloudMsg, *laserCloudIn);

        removeNaNFromPointCloud(laserCloudIn, laserCloudIn);

        // check dense flag
        // if (laserCloudIn->is_dense == false)
        // {
        //     ROS_ERROR("Point cloud is not in dense format, please remove NaN points first!");
        //     ros::shutdown();
        // }

        return true;
    }

    bool deskewInfo()
    {
        std::lock_guard<std::mutex> lock1(imuLock);
        std::lock_guard<std::mutex> lock2(odoLock);

        // make sure IMU data available for the scan
        if (imuQueue.empty() || imuQueue.front().header.stamp.toSec() > timeScanCur || imuQueue.back().header.stamp.toSec() < timeScanNext)
        {
            ROS_DEBUG("Waiting for IMU data ...");
            return false;
        }

        imuDeskewInfo();

        odomDeskewInfo();

        return true;
    }

    void imuDeskewInfo()
    {
        cloudInfo.imuAvailable = false;

        while (!imuQueue.empty())
        {
            if (imuQueue.front().header.stamp.toSec() < timeScanCur - 0.005)
                imuQueue.pop_front();
            else
                break;
        }

        if (imuQueue.empty())
            return;

        // imuPointerCur = 0;

        // for (int i = 0; i < (int)imuQueue.size(); ++i)
        // {
        //     sensor_msgs::Imu thisImuMsg = imuQueue[i];
        //     double currentImuTime = thisImuMsg.header.stamp.toSec();

        //     // get roll, pitch, and yaw estimation for this scan
        //     if (currentImuTime <= timeScanCur)
        //         imuRPY2rosRPY(&thisImuMsg, &cloudInfo.imuRollInit, &cloudInfo.imuPitchInit, &cloudInfo.imuYawInit);

        //     if (currentImuTime > timeScanNext + 0.01)
        //         break;

        //     if (imuPointerCur == 0){
        //         imuRotX[0] = 0;
        //         imuRotY[0] = 0;
        //         imuRotZ[0] = 0;
        //         imuTime[0] = currentImuTime;
        //         ++imuPointerCur;
        //         continue;
        //     }

        //     // get angular velocity
        //     double angular_x, angular_y, angular_z;
        //     imuAngular2rosAngular(&thisImuMsg, &angular_x, &angular_y, &angular_z);

        //     // integrate rotation
        //     double timeDiff = currentImuTime - imuTime[imuPointerCur-1];
        //     imuRotX[imuPointerCur] = imuRotX[imuPointerCur-1] + angular_x * timeDiff;
        //     imuRotY[imuPointerCur] = imuRotY[imuPointerCur-1] + angular_y * timeDiff;
        //     imuRotZ[imuPointerCur] = imuRotZ[imuPointerCur-1] + angular_z * timeDiff;
        //     imuTime[imuPointerCur] = currentImuTime;
        //     ++imuPointerCur;
        // }

        // --imuPointerCur;

        // if (imuPointerCur <= 0)
        //     return;

        cloudInfo.imuAvailable = true;
    }

    void odomDeskewInfo()
    {
        cloudInfo.odomAvailable = false;

        while (!odomQueue.empty())
        {
            if (odomQueue.front().header.stamp.toSec() < timeScanCur - 0.005)
                odomQueue.pop_front();
            else
                break;
        }

        if (odomQueue.empty())
            return;

        if (odomQueue.front().header.stamp.toSec() > timeScanCur)
            return;

        /*************************************************************************************/
        imuPointerCur = 0;

        for (int i = 0; i < (int)odomQueue.size(); ++i)
        {
            nav_msgs::Odometry thisOdomMsg = odomQueue[i];
            double currentImuTime = thisOdomMsg.header.stamp.toSec();

            // get roll, pitch, and yaw estimation for this scan
            if (currentImuTime <= timeScanCur)
                odomRPY2rosRPY(&thisOdomMsg, &cloudInfo.imuRollInit, &cloudInfo.imuPitchInit, &cloudInfo.imuYawInit);

            if (currentImuTime > timeScanNext + 0.01)
                break;

            Eigen::Quaterniond q(thisOdomMsg.pose.pose.orientation.w, thisOdomMsg.pose.pose.orientation.x, thisOdomMsg.pose.pose.orientation.y, thisOdomMsg.pose.pose.orientation.z);

            if (imuPointerCur == 0){
                imuPoseX[0] = 0;
                imuPoseY[0] = 0;
                imuPoseZ[0] = 0;
                imuPoseXInit = thisOdomMsg.pose.pose.position.x;
                imuPoseYInit = thisOdomMsg.pose.pose.position.y;
                imuPoseZInit = thisOdomMsg.pose.pose.position.z;

                imuRotX[0] = 0;
                imuRotY[0] = 0;
                imuRotZ[0] = 0;
                imuRot_q[0] = q;
                init_q_inverse = imuRot_q[0].inverse();
                imuTime[0] = currentImuTime;
                ++imuPointerCur;
                continue;
            }

            // pose
            imuPoseX[imuPointerCur] = thisOdomMsg.pose.pose.position.x - imuPoseXInit;
            imuPoseY[imuPointerCur] = thisOdomMsg.pose.pose.position.y - imuPoseYInit;
            imuPoseZ[imuPointerCur] = thisOdomMsg.pose.pose.position.z - imuPoseZInit;

            //rot
            imuRot_q[imuPointerCur] = init_q_inverse * q;
            imuTime[imuPointerCur] = currentImuTime;
            ++imuPointerCur;

            // double angular_x, angular_y, angular_z;
            // sensor_msgs::Imu thisImuMsg = imuQueue[i];
            // imuAngular2rosAngular(&thisImuMsg, &angular_x, &angular_y, &angular_z);
            // integrate rotation
            // double timeDiff = currentImuTime - imuTime[imuPointerCur-1];
            // imuRotX[imuPointerCur] = imuRotX[imuPointerCur-1] + angular_x * timeDiff;
            // imuRotY[imuPointerCur] = imuRotY[imuPointerCur-1] + angular_y * timeDiff;
            // imuRotZ[imuPointerCur] = imuRotZ[imuPointerCur-1] + angular_z * timeDiff;
            // D(imuRotZ[imuPointerCur]);
            // imuTime[imuPointerCur] = currentImuTime;
            // ++imuPointerCur;

            // Eigen::Vector3d eulerAngle = imuRot_q[imuPointerCur].matrix().eulerAngles(2,1,0);
            // imuRotX[imuPointerCur] = (eulerAngle[2]);
            // imuRotY[imuPointerCur] = (eulerAngle[1]);
            // imuRotZ[imuPointerCur] = (eulerAngle[0]);

        }

        --imuPointerCur;

        if (imuPointerCur <= 0)
            return;

        /*************************************************************************************/

        // get start odometry at the beinning of the scan
        nav_msgs::Odometry startOdomMsg;

        for (int i = 0; i < (int)odomQueue.size(); ++i)
        {
            startOdomMsg = odomQueue[i];

            if (ROS_TIME(&startOdomMsg) < timeScanCur)
                continue;
            else
                break;
        }

        tf::Quaternion orientation;
        tf::quaternionMsgToTF(startOdomMsg.pose.pose.orientation, orientation);

        double roll, pitch, yaw;
        tf::Matrix3x3(orientation).getRPY(roll, pitch, yaw);

        // Initial guess used in mapOptimization
        cloudInfo.initialGuessX = startOdomMsg.pose.pose.position.x;
        cloudInfo.initialGuessY = startOdomMsg.pose.pose.position.y;
        cloudInfo.initialGuessZ = startOdomMsg.pose.pose.position.z;
        cloudInfo.initialGuessRoll  = roll;
        cloudInfo.initialGuessPitch = pitch;
        cloudInfo.initialGuessYaw   = yaw;
        cloudInfo.imuPreintegrationResetId = round(startOdomMsg.pose.covariance[0]);

        cloudInfo.odomAvailable = true;

        // get end odometry at the end of the scan
        odomDeskewFlag = false;

        if (odomQueue.back().header.stamp.toSec() < timeScanNext)
            return;

        nav_msgs::Odometry endOdomMsg;

        for (int i = 0; i < (int)odomQueue.size(); ++i)
        {
            endOdomMsg = odomQueue[i];

            if (ROS_TIME(&endOdomMsg) < timeScanNext)
                continue;
            else
                break;
        }

        if (int(round(startOdomMsg.pose.covariance[0])) != int(round(endOdomMsg.pose.covariance[0])))
            return;

        Eigen::Affine3f transBegin = pcl::getTransformation(startOdomMsg.pose.pose.position.x, startOdomMsg.pose.pose.position.y, startOdomMsg.pose.pose.position.z, roll, pitch, yaw);

        tf::quaternionMsgToTF(endOdomMsg.pose.pose.orientation, orientation);
        tf::Matrix3x3(orientation).getRPY(roll, pitch, yaw);
        Eigen::Affine3f transEnd = pcl::getTransformation(endOdomMsg.pose.pose.position.x, endOdomMsg.pose.pose.position.y, endOdomMsg.pose.pose.position.z, roll, pitch, yaw);

        Eigen::Affine3f transBt = transBegin.inverse() * transEnd;

        float rollIncre, pitchIncre, yawIncre;
        pcl::getTranslationAndEulerAngles(transBt, odomIncreX, odomIncreY, odomIncreZ, rollIncre, pitchIncre, yawIncre);

        odomDeskewFlag = true;
    }

    void findRotation(double pointTime, Eigen::Quaterniond *rotqCur)
    {

        int imuPointerFront = 0;
        while (imuPointerFront < imuPointerCur)
        {
            if (pointTime < imuTime[imuPointerFront])
                break;
            ++imuPointerFront;
        }

        if (pointTime > imuTime[imuPointerFront] || imuPointerFront == 0)
        {
            *rotqCur = imuRot_q[imuPointerFront];
        } else {
            int imuPointerBack = imuPointerFront - 1;
            // double ratioFront = (pointTime - imuTime[imuPointerBack]) / (imuTime[imuPointerFront] - imuTime[imuPointerBack]);
            // double ratioBack = (imuTime[imuPointerFront] - pointTime) / (imuTime[imuPointerFront] - imuTime[imuPointerBack]);
            // *rotXCur = imuRotX[imuPointerFront] * ratioFront + imuRotX[imuPointerBack] * ratioBack;
            // *rotYCur = imuRotY[imuPointerFront] * ratioFront + imuRotY[imuPointerBack] * ratioBack;
            // *rotZCur = imuRotZ[imuPointerFront] * ratioFront + imuRotZ[imuPointerBack] * ratioBack;

            // Eigen::Quaterniond q_temp = imuRot_q[imuPointerBack].inverse() * imuRot_q[imuPointerFront];
            // q_temp = Eigen::Quaterniond::Identity().slerp( ratioFront, q_temp );
            // *rotqCur = imuRot_q[imuPointerBack] * q_temp;
            // if ( ratioFront < 0.5 )
            //     *rotqCur = imuRot_q[imuPointerBack];
            // else
            //     *rotqCur = imuRot_q[imuPointerFront];

            *rotqCur = imuRot_q[imuPointerFront];
        }
    }

    void findPosition(double pointTime, float *posXCur, float *posYCur, float *posZCur)
    {
        *posXCur = 0; *posYCur = 0; *posZCur = 0;

        // return; 
        // If the sensor moves relatively slow, like walking speed, positional deskew seems to have little benefits. Thus code below is commented.

        if (cloudInfo.odomAvailable == false || odomDeskewFlag == false)
            return;

        int odomPointerFront = 0;
        while (odomPointerFront < imuPointerCur)
        {
            if (pointTime < imuTime[odomPointerFront])
                break;
            ++odomPointerFront;
        }

        if (pointTime > imuTime[odomPointerFront] || odomPointerFront == 0)
        {
            *posXCur = imuPoseX[odomPointerFront];
            *posYCur = imuPoseY[odomPointerFront];
            *posZCur = imuPoseZ[odomPointerFront];
        } else {
            int odomPointerBack = odomPointerFront - 1;
            double ratioFront = (pointTime - imuTime[odomPointerBack]) / (imuTime[odomPointerFront] - imuTime[odomPointerBack]);
            double ratioBack = (imuTime[odomPointerFront] - pointTime) / (imuTime[odomPointerFront] - imuTime[odomPointerBack]);
            *posXCur = imuPoseX[odomPointerFront] * ratioFront + imuPoseX[odomPointerBack] * ratioBack;
            *posYCur = imuPoseY[odomPointerFront] * ratioFront + imuPoseY[odomPointerBack] * ratioBack;
            *posZCur = imuPoseZ[odomPointerFront] * ratioFront + imuPoseZ[odomPointerBack] * ratioBack;
            // *posXCur = imuPoseX[odomPointerFront];
            // *posYCur = imuPoseY[odomPointerFront];
            // *posZCur = imuPoseZ[odomPointerFront];
        }
    }

    PointType deskewPoint(PointType *point, double relTime)
    {
        if (deskewFlag == -1 || cloudInfo.imuAvailable == false)
            return *point;

        double pointTime = timeScanCur + relTime;

        Eigen::Quaterniond rotqCur;
        findRotation(pointTime, &rotqCur);

        float posXCur, posYCur, posZCur;
        findPosition(pointTime, &posXCur, &posYCur, &posZCur);

        if (firstPointFlag == true)
        {
            // transStartInverse = (pcl::getTransformation(posXCur, posYCur, posZCur, rotXCur, rotYCur, rotZCur)).inverse();
            q_start = rotqCur;
            q_start_inverse = rotqCur.inverse();
            Eigen::Vector3d t_tempp(posXCur, posYCur, posZCur);
            t_start = t_tempp;
            firstPointFlag = false;
        }

        // transform points to start
        // Eigen::Affine3f transFinal = pcl::getTransformation(posXCur, posYCur, posZCur, rotXCur, rotYCur, rotZCur);
        // Eigen::Affine3f transBt = transStartInverse * transFinal;

        // PointType newPoint;
        // newPoint.x = transBt(0,0) * point->x + transBt(0,1) * point->y + transBt(0,2) * point->z + transBt(0,3);
        // newPoint.y = transBt(1,0) * point->x + transBt(1,1) * point->y + transBt(1,2) * point->z + transBt(1,3);
        // newPoint.z = transBt(2,0) * point->x + transBt(2,1) * point->y + transBt(2,2) * point->z + transBt(2,3);
        // newPoint.intensity = point->intensity;
        
        //注意这里的q t 都是绝对量
        Eigen::Vector3d t_temp(posXCur, posYCur, posZCur);
        Eigen::Vector3d t_diff_to_start = t_temp - t_start;
        Eigen::Quaterniond q_diff_to_start = q_start_inverse * rotqCur;
        Eigen::Vector3d new_point_t(point->x, point->y, point->z);
        new_point_t = q_start * t_diff_to_start + q_diff_to_start * new_point_t;

        PointType newPoint;
        newPoint.x = new_point_t[0];
        newPoint.y = new_point_t[1];
        newPoint.z = new_point_t[2];
        newPoint.intensity = point->intensity;

        return newPoint;
    }

    void projectPointCloud()
    {
        int cloudSize = laserCloudIn->points.size();
        // range image projection
        for (int i = 0; i < cloudSize; ++i)
        {
            PointType thisPoint;
            thisPoint.x = laserCloudIn->points[i].x;
            thisPoint.y = laserCloudIn->points[i].y;
            thisPoint.z = laserCloudIn->points[i].z;
            thisPoint.intensity = laserCloudIn->points[i].intensity;

            float range = pointDistance(thisPoint);
            
            //if (range > 1.1)
            //    sensorRawCloud->points[i] = thisPoint;
            float limit_x = 0.53;
            float limit_y = 0.40;
            //float limit_z = -0.625;
            //if(((thisPoint.x >= limit_x) || (thisPoint.x <= -limit_x)) || ((thisPoint.y >= limit_y) || (thisPoint.y <= -limit_y)) || ((thisPoint.z >= 0) || (thisPoint.z <= limit_z)))
            //if((((thisPoint.x >= limit_x) || (thisPoint.x <= -limit_x)) || ((thisPoint.y >= limit_y) || (thisPoint.y <= -limit_y)) || (thisPoint.z >= 0)) && (thisPoint.z >= limit_z))
            if(((thisPoint.x >= limit_x) || (thisPoint.x <= -limit_x)) || ((thisPoint.y >= limit_y) || (thisPoint.y <= -limit_y)) || (thisPoint.z >= 0))
            {
                sensorRawCloud->points[i] = thisPoint;
            }

            // int rowIdn = laserCloudIn->points[i].ring;
            float angle_for_rowIdn = atan(thisPoint.z / sqrt(thisPoint.x * thisPoint.x + thisPoint.y * thisPoint.y)) * 180 / M_PI;
            int rowIdn = int((angle_for_rowIdn + 15) / 2 + 0.5);

            if (rowIdn < 0 || rowIdn >= N_SCAN)
                continue;

            if (rowIdn % downsampleRate != 0)
                continue;

            float horizonAngle = atan2(thisPoint.y, thisPoint.x) * 180 / M_PI + 180;

            // std::cout << horizonAngle << std::endl;

            static float ang_res_x = 360.0/float(Horizon_SCAN);
            int columnIdn = round(horizonAngle/ang_res_x);
            // int columnIdn = -round((horizonAngle-90.0)/ang_res_x) + Horizon_SCAN/2;
            if (columnIdn >= Horizon_SCAN)
                columnIdn -= Horizon_SCAN;

            if (columnIdn < 0 || columnIdn >= Horizon_SCAN)
                continue;
            
            //if (range < 1.0)
            //    continue;
            //if(((thisPoint.x < limit_x) && (thisPoint.x > -limit_x)) && ((thisPoint.y < limit_y) && ((thisPoint.z > 0) && (thisPoint.z > limit_z)))
            //if((((thisPoint.x < limit_x) && (thisPoint.x > -limit_x)) && ((thisPoint.y < limit_y) && (thisPoint.y > -limit_y)) && (thisPoint.z < 0)) || (thisPoint.z < limit_z))
            if(((thisPoint.x < limit_x) && (thisPoint.x > -limit_x)) && ((thisPoint.y < limit_y) && (thisPoint.y > -limit_y)) && (thisPoint.z < 0))
                continue;

            if (rangeMat.at<float>(rowIdn, columnIdn) != FLT_MAX)
                continue;

            // for the amsterdam dataset
            // if (range < 6.0 && rowIdn <= 7 && (columnIdn >= 1600 || columnIdn <= 200))
            //     continue;
            // if (thisPoint.z < -2.0)
            //     continue;

            rangeMat.at<float>(rowIdn, columnIdn) = range;

            // double time_for_points = ( 1 - 1.0 * columnIdn / Horizon_SCAN ) * (timeScanNext - timeScanCur);

            // thisPoint = deskewPoint(&thisPoint, time_for_points); // Velodyne
            thisPoint = deskewPoint(&thisPoint, laserCloudIn->points[i].time); // Velodyne
            // thisPoint = deskewPoint(&thisPoint, (float)laserCloudIn->points[i].t / 1000000000.0); // Ouster

            int index = columnIdn  + rowIdn * Horizon_SCAN;
            fullCloud->points[index] = thisPoint;
        }

    }

    void cloudExtraction()
    {
        int count = 0;
        // extract segmented cloud for lidar odometry
        for (int i = 0; i < N_SCAN; ++i)
        {
            cloudInfo.startRingIndex[i] = count - 1 + 5;

            for (int j = 0; j < Horizon_SCAN; ++j)
            {
                if (rangeMat.at<float>(i,j) != FLT_MAX)
                {
                    // mark the points' column index for marking occlusion later
                    cloudInfo.pointColInd[count] = j;
                    // save range info
                    cloudInfo.pointRange[count] = rangeMat.at<float>(i,j);
                    // save extracted cloud
                    extractedCloud->push_back(fullCloud->points[j + i*Horizon_SCAN]);
                    // size of extracted cloud
                    ++count;
                }
            }
            cloudInfo.endRingIndex[i] = count -1 - 5;
        }
    }
    
    void publishClouds()
    {
        cloudInfo.header = cloudHeader;
        cloudInfo.cloud_deskewed  = publishCloud(&pubExtractedCloud, extractedCloud, cloudHeader.stamp, "point_base_link");
	//cloudInfo.cloud_deskewed  = publishCloud(&pubExtractedCloud, extractedCloud, cloudHeader.stamp, "vehicle");
        cloudInfo.raw_sensor_cloud = publishCloud(&pubRawSensorCloud, sensorRawCloud, cloudHeader.stamp, "point_base_link");
        pubLaserCloudInfo.publish(cloudInfo);
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "lio_sam");

    ImageProjection IP;
    
    ROS_INFO("\033[1;32m----> Image Projection Started.\033[0m");

    ros::MultiThreadedSpinner spinner(3);
    spinner.spin();
    
    return 0;
}
