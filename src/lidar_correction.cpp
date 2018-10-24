#include <cmath>
#include <vector>

#include <common.h>
#include <opencv/cv.h>
#include <nav_msgs/Odometry.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

using std::atan2;
using std::cos;
using std::sin;

int scanIDInd[72] = {0,
                     0, 0, 0, 1, 1, 1, 2, 2, 2, 3,
                     3, 3, 4, 4, 4, 5, 5, 5, 6, 6,
                     6, 7, 7, 7, 8, 8, 8, 9, 9, 9,
                     10, 10, 11, 12, 13, 14, 15, 16, 17, 18,
                     19, 20, 21, 22, 23, 24, 25, 26, 27, 28,
                     29, 30, 31, 32, 33, 34, 34, 35, 35, 35,
                     36, 36, 36, 37, 37, 37, 38, 38, 38, 39,
                     39};

//扫描周期, velodyne频率10Hz，周期0.1s
const double scanPeriod = 0.1;

//初始化控制变量
const int systemDelay = 0; //弃用前20帧初始数据
int systemInitCount = 0;
bool systemInited = false;

//激光雷达线数
const int N_SCANS = 40;

//imu时间戳大于当前点云时间戳的位置
int imuPointerFront = 0;
//imu最新收到的点在数组中的位置
int imuPointerLast = -1;
//imu循环队列长度
const int imuQueLength = 200;

//点云数据开始第一个点的位移/速度/欧拉角
float imuRollStart = 0, imuPitchStart = 0, imuYawStart = 0;
float imuRollCur = 0, imuPitchCur = 0, imuYawCur = 0;

float imuVeloXStart = 0, imuVeloYStart = 0, imuVeloZStart = 0;
float imuShiftXStart = 0, imuShiftYStart = 0, imuShiftZStart = 0;

//当前点的速度，位移信息
float imuVeloXCur = 0, imuVeloYCur = 0, imuVeloZCur = 0;
float imuShiftXCur = 0, imuShiftYCur = 0, imuShiftZCur = 0;

//每次点云数据当前点相对于开始第一个点的畸变位移，速度
float imuShiftFromStartXCur = 0, imuShiftFromStartYCur = 0, imuShiftFromStartZCur = 0;
float imuVeloFromStartXCur = 0, imuVeloFromStartYCur = 0, imuVeloFromStartZCur = 0;

//IMU信息
double imuTime[imuQueLength] = {0};
float imuRoll[imuQueLength] = {0};
float imuPitch[imuQueLength] = {0};
float imuYaw[imuQueLength] = {0};

float imuAccX[imuQueLength] = {0};
float imuAccY[imuQueLength] = {0};
float imuAccZ[imuQueLength] = {0};

float imuVeloX[imuQueLength] = {0};
float imuVeloY[imuQueLength] = {0};
float imuVeloZ[imuQueLength] = {0};

float imuShiftX[imuQueLength] = {0};
float imuShiftY[imuQueLength] = {0};
float imuShiftZ[imuQueLength] = {0};

ros::Publisher pubLaserCloud;
ros::Publisher pubImuTrans;

//计算局部坐标系下点云中的点相对第一个开始点的由于加减速运动产生的位移畸变
// void ShiftToStartIMU(float pointTime)
// {
//     //计算相对于第一个点由于加减速产生的畸变位移(全局坐标系下畸变位移量delta_Tg)
//     //imuShiftFromStartCur = imuShiftCur - (imuShiftStart + imuVeloStart * pointTime)
//     imuShiftFromStartXCur = imuShiftXCur - imuShiftXStart - imuVeloXStart * pointTime;
//     imuShiftFromStartYCur = imuShiftYCur - imuShiftYStart - imuVeloYStart * pointTime;
//     imuShiftFromStartZCur = imuShiftZCur - imuShiftZStart - imuVeloZStart * pointTime;

//     /********************************************************************************
//   Rz(pitch).inverse * Rx(pitch).inverse * Ry(yaw).inverse * delta_Tg
//   transfrom from the global frame to the local frame
//   *********************************************************************************/

//     //绕y轴旋转(-imuYawStart)，即Ry(yaw).inverse
//     float x1 = cos(imuYawStart) * imuShiftFromStartXCur - sin(imuYawStart) * imuShiftFromStartZCur;
//     float y1 = imuShiftFromStartYCur;
//     float z1 = sin(imuYawStart) * imuShiftFromStartXCur + cos(imuYawStart) * imuShiftFromStartZCur;

//     //绕x轴旋转(-imuPitchStart)，即Rx(pitch).inverse
//     float x2 = x1;
//     float y2 = cos(imuPitchStart) * y1 + sin(imuPitchStart) * z1;
//     float z2 = -sin(imuPitchStart) * y1 + cos(imuPitchStart) * z1;

//     //绕z轴旋转(-imuRollStart)，即Rz(pitch).inverse
//     imuShiftFromStartXCur = cos(imuRollStart) * x2 + sin(imuRollStart) * y2;
//     imuShiftFromStartYCur = -sin(imuRollStart) * x2 + cos(imuRollStart) * y2;
//     imuShiftFromStartZCur = z2;
// }

// //计算局部坐标系下点云中的点相对第一个开始点由于加减速产生的的速度畸变（增量）
// void VeloToStartIMU()
// {
//     //计算相对于第一个点由于加减速产生的畸变速度(全局坐标系下畸变速度增量delta_Vg)
//     imuVeloFromStartXCur = imuVeloXCur - imuVeloXStart;
//     imuVeloFromStartYCur = imuVeloYCur - imuVeloYStart;
//     imuVeloFromStartZCur = imuVeloZCur - imuVeloZStart;

//     /********************************************************************************
//     Rz(pitch).inverse * Rx(pitch).inverse * Ry(yaw).inverse * delta_Vg
//     transfrom from the global frame to the local frame
//   *********************************************************************************/

//     //绕y轴旋转(-imuYawStart)，即Ry(yaw).inverse
//     float x1 = cos(imuYawStart) * imuVeloFromStartXCur - sin(imuYawStart) * imuVeloFromStartZCur;
//     float y1 = imuVeloFromStartYCur;
//     float z1 = sin(imuYawStart) * imuVeloFromStartXCur + cos(imuYawStart) * imuVeloFromStartZCur;

//     //绕x轴旋转(-imuPitchStart)，即Rx(pitch).inverse
//     float x2 = x1;
//     float y2 = cos(imuPitchStart) * y1 + sin(imuPitchStart) * z1;
//     float z2 = -sin(imuPitchStart) * y1 + cos(imuPitchStart) * z1;

//     //绕z轴旋转(-imuRollStart)，即Rz(pitch).inverse
//     imuVeloFromStartXCur = cos(imuRollStart) * x2 + sin(imuRollStart) * y2;
//     imuVeloFromStartYCur = -sin(imuRollStart) * x2 + cos(imuRollStart) * y2;
//     imuVeloFromStartZCur = z2;
// }

//去除点云加减速产生的位移畸变
void TransformToStartIMU(PointType *p)
{
    /********************************************************************************
    Ry*Rx*Rz*Pl, transform point to the global frame
  *********************************************************************************/
    // //绕z轴旋转(imuRollCur)
    // float x1 = cos(imuRollCur) * p->x - sin(imuRollCur) * p->y;
    // float y1 = sin(imuRollCur) * p->x + cos(imuRollCur) * p->y;
    // float z1 = p->z;

    // //绕x轴旋转(imuPitchCur)
    // float x2 = x1;
    // float y2 = cos(imuPitchCur) * y1 - sin(imuPitchCur) * z1;
    // float z2 = sin(imuPitchCur) * y1 + cos(imuPitchCur) * z1;

    // //绕y轴旋转(imuYawCur)
    // float x3 = cos(imuYawCur) * x2 + sin(imuYawCur) * z2;
    // float y3 = y2;
    // float z3 = -sin(imuYawCur) * x2 + cos(imuYawCur) * z2;

    Eigen::AngleAxisd rotation_vector1(imuYawCur, Eigen::Vector3d::UnitZ()); //沿 Z 轴旋转 xx 度
    rotation_vector1 = rotation_vector1 * Eigen::AngleAxisd(imuPitchCur, Eigen::Vector3d::UnitY());
    rotation_vector1 = rotation_vector1 * Eigen::AngleAxisd(imuRollCur, Eigen::Vector3d::UnitX());

    Eigen::Vector3d v1(p->x, p->y, p->z);
    Eigen::Vector3d v_rotated1 = rotation_vector1 * v1;

    float tmpX = v_rotated1(0) + imuShiftXCur;
    float tmpY = v_rotated1(1) + imuShiftYCur;
    float tmpZ = v_rotated1(2) + imuShiftZCur;

    /********************************************************************************
    Rz(pitch).inverse * Rx(pitch).inverse * Ry(yaw).inverse * Pg
    transfrom global points to the local frame
  *********************************************************************************/

    // //绕y轴旋转(-imuYawStart)
    // float x4 = cos(imuYawStart) * x3 - sin(imuYawStart) * z3;
    // float y4 = y3;
    // float z4 = sin(imuYawStart) * x3 + cos(imuYawStart) * z3;

    // //绕x轴旋转(-imuPitchStart)
    // float x5 = x4;
    // float y5 = cos(imuPitchStart) * y4 + sin(imuPitchStart) * z4;
    // float z5 = -sin(imuPitchStart) * y4 + cos(imuPitchStart) * z4;

    // //绕z轴旋转(-imuRollStart)，然后叠加平移量
    // p->x = cos(imuRollStart) * x5 + sin(imuRollStart) * y5 + imuShiftFromStartXCur;
    // p->y = -sin(imuRollStart) * x5 + cos(imuRollStart) * y5 + imuShiftFromStartYCur;
    // p->z = z5 + imuShiftFromStartZCur;

    Eigen::AngleAxisd rotation_vector2(-imuRollStart, Eigen::Vector3d::UnitX()); //沿 X 轴旋转 xx 度
    rotation_vector2 = rotation_vector2 * Eigen::AngleAxisd(-imuPitchStart, Eigen::Vector3d::UnitY());
    rotation_vector2 = rotation_vector2 * Eigen::AngleAxisd(-imuYawStart, Eigen::Vector3d::UnitZ());

    Eigen::Vector3d v2(-imuShiftXStart, -imuShiftYStart, -imuShiftZStart);
    Eigen::Vector3d v3(tmpX, tmpY, tmpZ);
    Eigen::Vector3d v_rotated2 = rotation_vector2 * v2;
    Eigen::Vector3d v_rotated3 = rotation_vector2 * v3;

    p->x = v_rotated2(0) + v_rotated3(0);
    p->y = v_rotated2(1) + v_rotated3(1);
    p->z = v_rotated2(2) + v_rotated3(2);
}

//积分速度与位移
void AccumulateIMUShift()
{
    float roll = imuRoll[imuPointerLast];
    float pitch = imuPitch[imuPointerLast];
    float yaw = imuYaw[imuPointerLast];
    float accX = imuAccX[imuPointerLast];
    float accY = imuAccY[imuPointerLast];
    float accZ = imuAccZ[imuPointerLast];

    // //将当前时刻的加速度值绕交换过的ZXY固定轴（原XYZ）分别旋转(roll, pitch, yaw)角，转换得到世界坐标系下的加速度值(right hand rule)
    // //绕z轴旋转(roll)
    // float x1 = cos(roll) * accX - sin(roll) * accY;
    // float y1 = sin(roll) * accX + cos(roll) * accY;
    // float z1 = accZ;
    // //绕x轴旋转(pitch)
    // float x2 = x1;
    // float y2 = cos(pitch) * y1 - sin(pitch) * z1;
    // float z2 = sin(pitch) * y1 + cos(pitch) * z1;
    // //绕y轴旋转(yaw)
    // accX = cos(yaw) * x2 + sin(yaw) * z2;
    // accY = y2;
    // accZ = -sin(yaw) * x2 + cos(yaw) * z2;

    Eigen::AngleAxisd rotation_vector(yaw, Eigen::Vector3d::UnitZ()); //沿 Z 轴旋转 xx 度
    rotation_vector = rotation_vector * Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY());
    rotation_vector = rotation_vector * Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX());

    Eigen::Vector3d v(accX, accY, accZ);
    Eigen::Vector3d v_rotated = rotation_vector * v;

    accX = v_rotated(0);
    accY = v_rotated(1);
    accZ = v_rotated(2);

    //上一个imu点
    int imuPointerBack = (imuPointerLast + imuQueLength - 1) % imuQueLength;
    //上一个点到当前点所经历的时间，即计算imu测量周期
    double timeDiff = imuTime[imuPointerLast] - imuTime[imuPointerBack];
    //要求imu的频率至少比lidar高，这样的imu信息才使用，后面校正也才有意义
    if (timeDiff < scanPeriod)
    { //（隐含从静止开始运动）
        //求每个imu时间点的位移与速度,两点之间视为匀加速直线运动
        imuShiftX[imuPointerLast] = imuShiftX[imuPointerBack] + imuVeloX[imuPointerBack] * timeDiff + accX * timeDiff * timeDiff / 2;
        imuShiftY[imuPointerLast] = imuShiftY[imuPointerBack] + imuVeloY[imuPointerBack] * timeDiff + accY * timeDiff * timeDiff / 2;
        imuShiftZ[imuPointerLast] = imuShiftZ[imuPointerBack] + imuVeloZ[imuPointerBack] * timeDiff + accZ * timeDiff * timeDiff / 2;

        imuVeloX[imuPointerLast] = imuVeloX[imuPointerBack] + accX * timeDiff;
        imuVeloY[imuPointerLast] = imuVeloY[imuPointerBack] + accY * timeDiff;
        imuVeloZ[imuPointerLast] = imuVeloZ[imuPointerBack] + accZ * timeDiff;
    }
}

//接收点云数据，velodyne雷达坐标系安装为x轴向前，y轴向左，z轴向上的右手坐标系
void laserCloudHandler(const sensor_msgs::PointCloud2ConstPtr &laserCloudMsg)
{
    if (!systemInited)
    { //丢弃前20个点云数据
        systemInitCount++;
        if (systemInitCount >= systemDelay)
        {
            systemInited = true;
        }
        return;
    }

    //记录每个scan有曲率的点的开始和结束索引
    std::vector<int> scanStartInd(N_SCANS, 0);
    std::vector<int> scanEndInd(N_SCANS, 0);

    //当前点云时间
    double timeScanCur = laserCloudMsg->header.stamp.toSec();
    pcl::PointCloud<pcl::PointXYZ> laserCloudIn;
    //消息转换成pcl数据存放
    pcl::fromROSMsg(*laserCloudMsg, laserCloudIn);
    std::vector<int> indices;
    //移除空点
    pcl::removeNaNFromPointCloud(laserCloudIn, laserCloudIn, indices);
    //点云点的数量
    int cloudSize = laserCloudIn.points.size();
    //lidar scan开始点的旋转角,atan2范围[-pi,+pi],计算旋转角时取负号是因为velodyne是顺时针旋转
    float startOri = -atan2(laserCloudIn.points[0].y, laserCloudIn.points[0].x);
    //lidar scan结束点的旋转角，加2*pi使点云旋转周期为2*pi
    float endOri = -atan2(laserCloudIn.points[cloudSize - 1].y,
                          laserCloudIn.points[cloudSize - 1].x) +
                   2 * M_PI;

    //结束方位角与开始方位角差值控制在(PI,3*PI)范围，允许lidar不是一个圆周扫描
    //正常情况下在这个范围内：pi < endOri - startOri < 3*pi，异常则修正
    if (endOri - startOri > 3 * M_PI)
    {
        endOri -= 2 * M_PI;
    }
    else if (endOri - startOri < M_PI)
    {
        endOri += 2 * M_PI;
    }
    //lidar扫描线是否旋转过半
    bool halfPassed = false;
    int count = cloudSize;
    PointType point;
    std::vector<pcl::PointCloud<PointType> > laserCloudScans(N_SCANS);
    for (int i = 0; i < cloudSize; i++)
    {
        //坐标轴交换，velodyne lidar的坐标系也转换到z轴向前，x轴向左的右手坐标系
        point.x = laserCloudIn.points[i].x;
        point.y = laserCloudIn.points[i].y;
        point.z = laserCloudIn.points[i].z;

        //计算点的仰角(根据lidar文档垂直角计算公式),根据仰角排列激光线号，velodyne每两个scan之间间隔2度
        float angle = atan(point.z / sqrt(point.x * point.x + point.y * point.y)) * 180 / M_PI;
        int scanID;
        // //仰角四舍五入(加减0.5截断效果等于四舍五入)
        // int roundedAngle = int(angle + (angle < 0.0 ? -0.5 : +0.5));
        // if (roundedAngle > 0)
        // {
        //     scanID = roundedAngle;
        // }
        // else
        // {
        //     scanID = roundedAngle + (N_SCANS - 1);
        // }
        // //过滤点，只挑选[-15度，+15度]范围内的点,scanID属于[0,15]
        // if (scanID > (N_SCANS - 1) || scanID < 0)
        // {
        //     count--;
        //     continue;
        // }

        int roundedAngle = int(angle * 3 + 50);

        if (roundedAngle < 0 || roundedAngle > 71)
        {
            count--;
            continue;
        }

        scanID = scanIDInd[roundedAngle];

        //该点的旋转角
        float ori = -atan2(point.y, point.x);
        if (!halfPassed)
        { //根据扫描线是否旋转过半选择与起始位置还是终止位置进行差值计算，从而进行补偿
            //确保-pi/2 < ori - startOri < 3*pi/2
            if (ori < startOri - M_PI / 2)
            {
                ori += 2 * M_PI;
            }
            else if (ori > startOri + M_PI * 3 / 2)
            {
                ori -= 2 * M_PI;
            }

            if (ori - startOri > M_PI)
            {
                halfPassed = true;
            }
        }
        else
        {
            ori += 2 * M_PI;

            //确保-3*pi/2 < ori - endOri < pi/2
            if (ori < endOri - M_PI * 3 / 2)
            {
                ori += 2 * M_PI;
            }
            else if (ori > endOri + M_PI / 2)
            {
                ori -= 2 * M_PI;
            }
        }

        //-0.5 < relTime < 1.5（点旋转的角度与整个周期旋转角度的比率, 即点云中点的相对时间）
        float relTime = (ori - startOri) / (endOri - startOri);
        //点强度=线号+点相对时间（即一个整数+一个小数，整数部分是线号，小数部分是该点的相对时间）,匀速扫描：根据当前扫描的角度和扫描周期计算相对扫描起始位置的时间
        point.intensity = scanID + scanPeriod * relTime;

        //点时间=点云时间+周期时间
        if (imuPointerLast >= 0)
        {                                           //如果收到IMU数据,使用IMU矫正点云畸变
            float pointTime = relTime * scanPeriod; //计算点的周期时间
            //寻找是否有点云的时间戳小于IMU的时间戳的IMU位置:imuPointerFront
            while (imuPointerFront != imuPointerLast)
            {
                if (timeScanCur + pointTime < imuTime[imuPointerFront])
                {
                    break;
                }
                imuPointerFront = (imuPointerFront + 1) % imuQueLength;
            }

            if (timeScanCur + pointTime > imuTime[imuPointerFront])
            { //没找到,此时imuPointerFront==imtPointerLast,只能以当前收到的最新的IMU的速度，位移，欧拉角作为当前点的速度，位移，欧拉角使用
                imuRollCur = imuRoll[imuPointerFront];
                imuPitchCur = imuPitch[imuPointerFront];
                imuYawCur = imuYaw[imuPointerFront];

                imuVeloXCur = imuVeloX[imuPointerFront];
                imuVeloYCur = imuVeloY[imuPointerFront];
                imuVeloZCur = imuVeloZ[imuPointerFront];

                imuShiftXCur = imuShiftX[imuPointerFront];
                imuShiftYCur = imuShiftY[imuPointerFront];
                imuShiftZCur = imuShiftZ[imuPointerFront];
            }
            else
            { //找到了点云时间戳小于IMU时间戳的IMU位置,则该点必处于imuPointerBack和imuPointerFront之间，据此线性插值，计算点云点的速度，位移和欧拉角
                int imuPointerBack = (imuPointerFront + imuQueLength - 1) % imuQueLength;
                //按时间距离计算权重分配比率,也即线性插值
                float ratioFront = (timeScanCur + pointTime - imuTime[imuPointerBack]) / (imuTime[imuPointerFront] - imuTime[imuPointerBack]);
                float ratioBack = (imuTime[imuPointerFront] - timeScanCur - pointTime) / (imuTime[imuPointerFront] - imuTime[imuPointerBack]);

                imuRollCur = imuRoll[imuPointerFront] * ratioFront + imuRoll[imuPointerBack] * ratioBack;
                imuPitchCur = imuPitch[imuPointerFront] * ratioFront + imuPitch[imuPointerBack] * ratioBack;
                if (imuYaw[imuPointerFront] - imuYaw[imuPointerBack] > M_PI)
                {
                    imuYawCur = imuYaw[imuPointerFront] * ratioFront + (imuYaw[imuPointerBack] + 2 * M_PI) * ratioBack;
                }
                else if (imuYaw[imuPointerFront] - imuYaw[imuPointerBack] < -M_PI)
                {
                    imuYawCur = imuYaw[imuPointerFront] * ratioFront + (imuYaw[imuPointerBack] - 2 * M_PI) * ratioBack;
                }
                else
                {
                    imuYawCur = imuYaw[imuPointerFront] * ratioFront + imuYaw[imuPointerBack] * ratioBack;
                }

                //本质:imuVeloXCur = imuVeloX[imuPointerback] + (imuVelX[imuPointerFront]-imuVelX[imuPoniterBack])*ratioFront
                imuVeloXCur = imuVeloX[imuPointerFront] * ratioFront + imuVeloX[imuPointerBack] * ratioBack;
                imuVeloYCur = imuVeloY[imuPointerFront] * ratioFront + imuVeloY[imuPointerBack] * ratioBack;
                imuVeloZCur = imuVeloZ[imuPointerFront] * ratioFront + imuVeloZ[imuPointerBack] * ratioBack;

                imuShiftXCur = imuShiftX[imuPointerFront] * ratioFront + imuShiftX[imuPointerBack] * ratioBack;
                imuShiftYCur = imuShiftY[imuPointerFront] * ratioFront + imuShiftY[imuPointerBack] * ratioBack;
                imuShiftZCur = imuShiftZ[imuPointerFront] * ratioFront + imuShiftZ[imuPointerBack] * ratioBack;
            }

            if (i == 0)
            { //如果是第一个点,记住点云起始位置的速度，位移，欧拉角
                imuRollStart = imuRollCur;
                imuPitchStart = imuPitchCur;
                imuYawStart = imuYawCur;

                imuVeloXStart = imuVeloXCur;
                imuVeloYStart = imuVeloYCur;
                imuVeloZStart = imuVeloZCur;

                imuShiftXStart = imuShiftXCur;
                imuShiftYStart = imuShiftYCur;
                imuShiftZStart = imuShiftZCur;
            }
            else
            { //计算之后每个点相对于第一个点的由于加减速非匀速运动产生的位移速度畸变，并对点云中的每个点位置信息重新补偿矫正
                // ShiftToStartIMU(pointTime);
                // VeloToStartIMU();
                TransformToStartIMU(&point);
            }
        }
        laserCloudScans[scanID].push_back(point); //将每个补偿矫正的点放入对应线号的容器
    }

    //获得有效范围内的点的数量
    cloudSize = count;

    pcl::PointCloud<PointType>::Ptr laserCloud(new pcl::PointCloud<PointType>());
    for (int i = 0; i < N_SCANS; i++)
    { //将所有的点按照线号从小到大放入一个容器
        *laserCloud += laserCloudScans[i];
    }

    //publich消除非匀速运动畸变后的所有的点
    sensor_msgs::PointCloud2 laserCloudOutMsg;
    pcl::toROSMsg(*laserCloud, laserCloudOutMsg);
    laserCloudOutMsg.header.stamp = laserCloudMsg->header.stamp;
    laserCloudOutMsg.header.frame_id = "/pandar";
    pubLaserCloud.publish(laserCloudOutMsg);

    //publich IMU消息,由于循环到了最后，因此是Cur都是代表最后一个点，即最后一个点的欧拉角，畸变位移及一个点云周期增加的速度
    pcl::PointCloud<pcl::PointXYZ> imuTrans(4, 1);
    //起始点欧拉角
    imuTrans.points[0].x = imuPitchStart;
    imuTrans.points[0].y = imuYawStart;
    imuTrans.points[0].z = imuRollStart;

    //最后一个点的欧拉角
    imuTrans.points[1].x = imuPitchCur;
    imuTrans.points[1].y = imuYawCur;
    imuTrans.points[1].z = imuRollCur;

    //最后一个点相对于第一个点的畸变位移和速度
    imuTrans.points[2].x = imuShiftFromStartXCur;
    imuTrans.points[2].y = imuShiftFromStartYCur;
    imuTrans.points[2].z = imuShiftFromStartZCur;

    imuTrans.points[3].x = imuVeloFromStartXCur;
    imuTrans.points[3].y = imuVeloFromStartYCur;
    imuTrans.points[3].z = imuVeloFromStartZCur;

    sensor_msgs::PointCloud2 imuTransMsg;
    pcl::toROSMsg(imuTrans, imuTransMsg);
    imuTransMsg.header.stamp = laserCloudMsg->header.stamp;
    imuTransMsg.header.frame_id = "/pandar";
    pubImuTrans.publish(imuTransMsg);
}

//接收imu消息，imu坐标系为x轴向前，y轴向右，z轴向上的右手坐标系
void imuHandler(const sensor_msgs::Imu::ConstPtr &imuIn)
{
    double roll, pitch, yaw;
    tf::Quaternion orientation;
    //convert Quaternion msg to Quaternion
    tf::quaternionMsgToTF(imuIn->orientation, orientation);
    //This will get the roll pitch and yaw from the matrix about fixed axes X, Y, Z respectively. That's R = Rz(yaw)*Ry(pitch)*Rx(roll).
    //Here roll pitch yaw is in the global frame
    tf::Matrix3x3(orientation).getRPY(roll, pitch, yaw);

    // //减去重力的影响,求出xyz方向的加速度实际值，并进行坐标轴交换，统一到z轴向前,x轴向左的右手坐标系, 交换过后RPY对应fixed axes ZXY(RPY---ZXY)。Now R = Ry(yaw)*Rx(pitch)*Rz(roll).
    // float accX = imuIn->linear_acceleration.y - sin(roll) * cos(pitch) * 9.81;
    // float accY = imuIn->linear_acceleration.z - cos(roll) * cos(pitch) * 9.81;
    // float accZ = imuIn->linear_acceleration.x + sin(pitch) * 9.81;

    Eigen::AngleAxisd rotation_vector(-roll, Eigen::Vector3d::UnitX()); //沿 X 轴旋转 xx 度
    rotation_vector = rotation_vector * Eigen::AngleAxisd(-pitch, Eigen::Vector3d::UnitY());
    rotation_vector = rotation_vector * Eigen::AngleAxisd(-yaw, Eigen::Vector3d::UnitZ());

    Eigen::Vector3d g(0, 0, -9.81);
    Eigen::Vector3d v_rotated = rotation_vector * g;

    float accX = imuIn->linear_acceleration.x + v_rotated(0);
    float accY = imuIn->linear_acceleration.y + v_rotated(1);
    float accZ = imuIn->linear_acceleration.z + v_rotated(2);

    //循环移位效果，形成环形数组
    imuPointerLast = (imuPointerLast + 1) % imuQueLength;

    imuTime[imuPointerLast] = imuIn->header.stamp.toSec();
    imuRoll[imuPointerLast] = roll;
    imuPitch[imuPointerLast] = pitch;
    imuYaw[imuPointerLast] = yaw;
    imuAccX[imuPointerLast] = accX;
    imuAccY[imuPointerLast] = accY;
    imuAccZ[imuPointerLast] = accZ;

    AccumulateIMUShift();
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "lidarCorrection");
    ros::NodeHandle nh;

    ros::Subscriber subLaserCloud = nh.subscribe<sensor_msgs::PointCloud2>("/pandar_points", 2, laserCloudHandler);

    ros::Subscriber subImu = nh.subscribe<sensor_msgs::Imu>("/Inertial/imu/data", 100, imuHandler);

    pubLaserCloud = nh.advertise<sensor_msgs::PointCloud2>("/velodyne_cloud_2", 2);

    pubImuTrans = nh.advertise<sensor_msgs::PointCloud2>("/imu_trans", 5);

    ros::spin();

    return 0;
}