/**
* This file is part of ORB-SLAM3
*
* Copyright (C) 2017-2021 Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
* Copyright (C) 2014-2016 Raúl Mur-Artal, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
*
* ORB-SLAM3 is free software: you can redistribute it and/or modify it under the terms of the GNU General Public
* License as published by the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM3 is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even
* the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License along with ORB-SLAM3.
* If not, see <http://www.gnu.org/licenses/>.
*/


#ifndef TRACKING_H
#define TRACKING_H

#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>

#include "Viewer.h"
#include "FrameDrawer.h"
#include "Atlas.h"
#include "LocalMapping.h"
#include "LoopClosing.h"
#include "Frame.h"
#include "ORBVocabulary.h"
#include "KeyFrameDatabase.h"
#include "ORBextractor.h"
#include "MapDrawer.h"
#include "System.h"
#include "ImuTypes.h"
#include "Settings.h"

#include "GeometricCamera.h"

#include <mutex>
#include <unordered_set>


#include "../yolov8/include/yolo.h"
#include "../yolov8/include/yolov8.h"
#include "../yolov8/include/yolov8_seg.h"
#include "../line_descriptor/include/line_descriptor.hpp"

namespace ORB_SLAM3
{

class Viewer;
class FrameDrawer;
class Atlas;
class LocalMapping;
class LoopClosing;
class System;
class Settings;

class Tracking
{  

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    /**
     * @brief 构造函数
     * 
     * @param[in] pSys              系统实例 
     * @param[in] pVoc              字典指针
     * @param[in] pFrameDrawer      帧绘制器
     * @param[in] pMapDrawer        地图绘制器
     * @param[in] pMap              地图句柄
     * @param[in] pKFDB             关键帧数据库句柄
     * @param[in] strSettingPath    配置文件路径
     * @param[in] sensor            传感器类型
     */
    Tracking(System* pSys, ORBVocabulary* pVoc, FrameDrawer* pFrameDrawer, MapDrawer* pMapDrawer, Atlas* pAtlas,
             KeyFrameDatabase* pKFDB, const string &strSettingPath, const int sensor, Settings* settings,
             YOLOv8Seg* yoloseg, cv::Ptr<LineDescriptor::BinaryDescriptor> LineExtracter, const string &_nameSeq=std::string());

    ~Tracking();

    // Parse the config file
    bool ParseCamParamFile(cv::FileStorage &fSettings);
    bool ParseORBParamFile(cv::FileStorage &fSettings);
    bool ParseIMUParamFile(cv::FileStorage &fSettings);

    // Preprocess the input and call Track(). Extract features and performs stereo matching.
     // 下面的函数都是对不同的传感器输入的图像进行处理(转换成为灰度图像),并且调用Tracking线程
    /**
     * @brief 处理双目输入
     * 
     * @param[in] imRectLeft    左目图像
     * @param[in] imRectRight   右目图像
     * @param[in] timestamp     时间戳
     * @return cv::Mat          世界坐标系到该帧相机坐标系的变换矩阵
     */
    Sophus::SE3f GrabImageStereo(const cv::Mat &imRectLeft,const cv::Mat &imRectRight, const double &timestamp, string filename);
    /**
     * @brief 处理RGBD输入的图像
     * 
     * @param[in] imRGB         彩色图像
     * @param[in] imD           深度图像
     * @param[in] timestamp     时间戳
     * @return cv::Mat          世界坐标系到该帧相机坐标系的变换矩阵
     */
    Sophus::SE3f GrabImageRGBD(const cv::Mat &imRGB,const cv::Mat &imD, const double &timestamp, string filename);
    /**
     * @brief 处理单目输入图像
     * 
     * @param[in] im            图像
     * @param[in] timestamp     时间戳
     * @return cv::Mat          世界坐标系到该帧相机坐标系的变换矩阵
     */
    Sophus::SE3f GrabImageMonocular(const cv::Mat &im, const double &timestamp, string filename);

    void GrabImuData(const IMU::Point &imuMeasurement);

    void SetLocalMapper(LocalMapping* pLocalMapper);
    void SetLoopClosing(LoopClosing* pLoopClosing);
    void SetViewer(Viewer* pViewer);
    void SetStepByStep(bool bSet);
    bool GetStepByStep();

    // Load new settings
    // The focal lenght should be similar or scale prediction will fail when projecting points
    void ChangeCalibration(const string &strSettingPath);

    // Use this function if you have deactivated local mapping and you only want to localize the camera.
    void InformOnlyTracking(const bool &flag);

    void UpdateFrameIMU(const float s, const IMU::Bias &b, KeyFrame* pCurrentKeyFrame);
    KeyFrame* GetLastKeyFrame()
    {
        return mpLastKeyFrame;
    }

    void CreateMapInAtlas();
    //std::mutex mMutexTracks;

    //--
    void NewDataset();
    int GetNumberDataset();
    int GetMatchesInliers();

    //DEBUG
    void SaveSubTrajectory(string strNameFile_frames, string strNameFile_kf, string strFolder="");
    void SaveSubTrajectory(string strNameFile_frames, string strNameFile_kf, Map* pMap);

    float GetImageScale();

#ifdef REGISTER_LOOP
    void RequestStop();
    bool isStopped();
    void Release();
    bool stopRequested();
#endif

public:

    // Tracking states
    enum eTrackingState{
        SYSTEM_NOT_READY=-1,//系统没有准备好的状态,一般就是在启动后加载配置文件和词典文件时候的状态
        NO_IMAGES_YET=0, //当前无图像
        NOT_INITIALIZED=1,//有图像但是没有完成初始化
        OK=2,//正常时候的工作状态
        RECENTLY_LOST=3,// 新增了一个状态RECENTLY_LOST，主要是结合IMU看看能不能拽回来
        LOST=4, //系统已经跟丢了的状态
        OK_KLT=5
    };

    eTrackingState mState;//跟踪状态
    eTrackingState mLastProcessedState;//上一帧的跟踪状态.这个变量在绘制当前帧的时候会被使用到
    int mSensor;// Input sensor

    Frame mCurrentFrame;
    Frame mLastFrame;
    cv::Mat mImGray,mImSegLine,mImLast,mImLastRGB;//当前帧的灰度图像 //? 提问,那么在双目输入和在RGBD输入的时候呢? 在双目输入和在RGBD输入时，为左侧图像的灰度图
    cv::Mat mInstance,mInstanLast;
    vector<KeyLine> mvKeyLine;
    vector<util::Box> mvObjects;
    vector<bool> mvbDyanmic;

    // Initialization Variables (Monocular)
    std::vector<int> mvIniLastMatches;// 初始化时前两帧相关变量，之前的匹配
    std::vector<int> mvIniMatches;//初始化阶段中,当前帧中的特征点和参考帧中的特征点的匹配关系，跟踪初始化时前两帧之间的匹配
    std::vector<cv::Point2f> mvbPrevMatched;//在初始化的过程中,保存参考帧中的特征点//记录"上一帧"所有特征点
    std::vector<cv::Point3f> mvIniP3D;//初始化过程中匹配后进行三角化得到的空间点
    Frame mInitialFrame;//单目初始化的第一帧，初始化过程中的参考帧

    // Lists used to recover the full camera trajectory at the end of the execution.
    // Basically we store the reference keyframe for each frame and its relative transformation
    list<Sophus::SE3f> mlRelativeFramePoses;//所有帧的参考关键帧到这个帧的相对位姿
    list<KeyFrame*> mlpReferences;//所有参考关键帧
    list<double> mlFrameTimes;//所有帧的时间戳 
    list<bool> mlbLost;//所有帧是否跟丢的标志

    vector<double> mSeg,mLineExtration,mSpatil,mPLoptimle;
    // frames with estimated pose
    int mTrackedFr;
    bool mbStep;//逐帧运行模式下是否成功获取下一帧

    // True if local mapping is deactivated and we are performing only localization
    bool mbOnlyTracking;

    void Reset(bool bLocMap = false);
    void ResetActiveMap(bool bLocMap = false);

    void SetInstance(cv::Mat &img)
    {
        mInstance=img.clone();
    }
    
    float mMeanTrack;
    bool mbInitWith3KFs;
    double t0; // time-stamp of first read frame
    double t0vis; // time-stamp of first inserted keyframe
    double t0IMU; // time-stamp of IMU initialization
    bool mFastInit = false;


    vector<MapPoint*> GetLocalMapMPS();

    bool mbWriteStats;

#ifdef REGISTER_TIMES
    void LocalMapStats2File();
    void TrackStats2File();
    void PrintTimeStats();

    vector<double> vdRectStereo_ms;
    vector<double> vdResizeImage_ms;//跟踪线程中重定义图幅大小的时间统计
    vector<double> vdORBExtract_ms;//创建图像帧时提取图像ORB特征的时间统计
    vector<double> vdStereoMatch_ms;//创建图像帧时双目鱼眼相机左右图像匹配的时间统计
    vector<double> vdIMUInteg_ms;//track()中PreintegrateIMU()跟踪线程IMU预计分时间统计
    vector<double> vdPosePred_ms;//跟踪线程中跟踪图像帧(跟踪参考帧、恒速模型、重定位)预测位姿的时间统计
    vector<double> vdLMTrack_ms;//跟踪线程中局部建图跟踪得到更多的匹配优化当前帧位姿的时间统计
    vector<double> vdNewKF_ms;//跟踪线程判断并插入关键帧的时间统计
    vector<double> vdTrackTotal_ms;//跟踪线程中总的跟踪的时间统计
#endif

protected:

    // Main tracking function. It is independent of the input sensor.
    void Track();

    // Map initialization for stereo and RGB-D
    void StereoInitialization();

    // Map initialization for monocular
    void MonocularInitialization();
    //void CreateNewMapPoints();
    void CreateInitialMapMonocular();

    void CheckReplacedInLastFrame();
    bool TrackReferenceKeyFrameWithLine();
    bool TrackReferenceKeyFrame();
    void UpdateLastFrame();
    bool TrackWithMotionModelWithLine();
    bool TrackWithMotionModel();
    bool PredictStateIMU();
    //重定位过程
    bool Relocalization();

    void UpdateLocalMap();
    void UpdateLocalPoints();
    void UpdateLocalKeyFrames();

    bool TrackLocalMap();
    void SearchLocalPointsWithLine();
    void SearchLocalPoints();

    bool NeedNewKeyFrame();
    void CreateNewKeyFrame();

    // Perform preintegration from last frame
    void PreintegrateIMU();

    // Reset IMU biases and compute frame velocity
    void ResetFrameIMU();



    void SegmentAndLineExteract()
    {
        mvKeyLine.clear();
        if(mImSegLine.channels()!=3) cvtColor(mImSegLine,mImSegLine,cv::COLOR_GRAY2BGR);
        vector<cv::Mat> imgbatch(1,mImSegLine);
        myoloseg->run(imgbatch);
        myoloseg->getInstanceMask(mInstance,mvObjects);
        mLineExtracter->detect(mImSegLine,mvKeyLine);
        // for(int i=0;i<mvKeyLine.size();i++)cout<<mvKeyLine[i].ps<<" "<<mvKeyLine[i].pe<<endl;
    }
    void PreTrackWithKL(Frame &curtF, const Frame *lastF,const cv::Mat &imDepth, const cv::Mat &imInstance);


void static fit2dPlaneRANSAC(const std::vector<cv::Point2f> &points, cv::Point3f &bestPlane, cv::Point2f &pt, int maxIterations = 50, double distanceThreshold = 0.001) 
{
    // cout<<"5.1"<<endl;
    int bestInlierCount = 0, n=points.size()*0.9;
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<> dis(0, points.size() - 1);
// cout<<"5.2"<<endl;
    for (int iter = 0; iter < maxIterations; ++iter) 
    {
        int idx1 = dis(gen);
        int idx2 = dis(gen);

        if (idx1 == idx2)  continue; // 避免选到相同的点 
        const cv::Point2f  p1 = points[idx1];
        const cv::Point2f  p2 = points[idx2];

        // 计算直线参数 Ax + By + C = 0
        double A = p2.y - p1.y;      // A = y2 - y1
        double B = p1.x - p2.x;      // B = x1 - x2
        double C = p2.x * p1.y - p1.x * p2.y; // C = x2*y1 - x1*y2
        double D = 1.0/sqrt(pow(A,2)+pow(B,2));
        cv::Point3f normal=cv::Point3f(A*D,B*D,C*D);

        int inlierCount = 0; 
        for(int i=0,iend=points.size(); i<iend; i++) 
        {
            cv::Point3f point(points[i].x, points[i].y, 1);
            double distance =abs(normal.dot(point));
            // double distance =points[i].z-Eigen::Vector3f(plane[0],plane[1],plane[3]).dot(Eigen::Vector3f(points[i].x,points[i].y,1))/plane[2];
            if (distance < distanceThreshold) inlierCount++;
        }
        // 更新最佳平面
        if(inlierCount > bestInlierCount) {
            bestInlierCount = inlierCount;
            bestPlane = normal;
            pt=p1;
        }
        if(inlierCount>n) break;
    }
}


    bool mbMapUpdated;//更新地图， 判断地图id是否更新了

    
    IMU::Preintegrated *mpImuPreintegratedFromLastKF;  // Imu preintegration from last KeyFrame

    // Queue of IMU measurements between frames
    std::list<IMU::Point> mlQueueImuData;

    // Vector of IMU measurements from previous to current frame (to be filled by PreintegrateIMU)
    std::vector<IMU::Point> mvImuFromLastFrame;
    std::mutex mMutexImuQueue;

    // Imu calibration parameters
    IMU::Calib *mpImuCalib;

    // Last Bias Estimation (at keyframe creation)
    IMU::Bias mLastBias;

    // In case of performing only localization, this flag is true when there are no matches to
    // points in the map. Still tracking will continue if there are enough matches with temporal points.
    // In that case we are doing visual odometry. The system will try to do relocalization to recover
    // "zero-drift" localization to the map.
    bool mbVO;//当进行纯定位时才会有的一个变量,为false表示该帧匹配了很多的地图点，跟踪正常;如果少于10个则为true表示快要跟踪失败

    //Other Thread Pointers
    LocalMapping* mpLocalMapper;//局部建图器句柄
    LoopClosing* mpLoopClosing;//回环检测器句柄

    // orb特征提取器，不管单目还是双目，mpORBextractorLeft都要用到
    // 如果是双目，则要用到mpORBextractorRight
    // NOTICE 如果是单目，在初始化的时候使用mpIniORBextractor而不是mpORBextractorLeft，
    // mpIniORBextractor属性中提取的特征点个数是mpORBextractorLeft的两倍
    ORBextractor* mpORBextractorLeft, *mpORBextractorRight;
    ORBextractor* mpIniORBextractor;//在初始化的时候使用的特征点提取器,其提取到的特征点个数会更多

    //BoW
    ORBVocabulary* mpORBVocabulary;
    KeyFrameDatabase* mpKeyFrameDB;//当前系统运行的时候,关键帧所产生的数据库

    // Initalization (only for monocular)
    bool mbReadyToInitializate;
    bool mbSetInit;

    //Local Map
    KeyFrame* mpReferenceKF;//用于普通帧跟踪的参考关键帧
    std::vector<KeyFrame*> mvpLocalKeyFrames;//局部关键帧集合
    std::vector<MapPoint*> mvpLocalMapPoints;//局部地图点的集合
    std::vector<MapLine *> mvpLocalMapLines;
    
    YOLOv8Seg* myoloseg;
    BinaryDescriptor* mLineExtracter;
    // System
    System* mpSystem;
    
    //Drawers
    Viewer* mpViewer;
    FrameDrawer* mpFrameDrawer;
    MapDrawer* mpMapDrawer;
    bool bStepByStep;//从菜单栏中返回的是否逐帧运行

    //Atlas
    Atlas* mpAtlas;

    //Calibration matrix
    cv::Mat mK;//相机的内参数矩阵
    Eigen::Matrix3f mK_;
    cv::Mat mDistCoef;//相机的去畸变参数
    float mbf;//相机的基线长度 * 相机的焦距
    float mImageScale;//图像缩放因子

    float mImuFreq;
    double mImuPer;//imu起始数据会比当前帧的前一帧时间戳早的差值，读取参数时设置为0.001
    bool mInsertKFsLost;//跟踪丢失时是否插入关键帧

    //New KeyFrame rules (according to fps)
    int mMinFrames,mMaxFrames;// 新建关键帧和重定位中用来判断最小最大时间间隔，和帧率有关

    int mnFirstImuFrameId;
    int mnFramesToResetIMU;//表示经过多少帧后可以重置IMU，一般设置为和帧率相同，对应的时间是1s

    // Threshold close/far points
    // Points seen as close by the stereo/RGBD sensor are considered reliable
    // and inserted from just one frame. Far points requiere a match in two keyframes.
    float mThDepth;//用于区分远点和近点的阈值. 近点认为可信度比较高;远点则要求在两个关键帧中得到匹配

    // For RGB-D inputs only. For some datasets (e.g. TUM) the depthmap values are scaled.
    float mDepthMapFactor;//深度缩放因子,链接深度值和具体深度值的参数.只对RGBD输入有效

    //Current matches in frame
    int mnMatchesInliers;//当前帧中的进行匹配的内点,将会被不同的函数反复使用

    //Last Frame, KeyFrame and Relocalisation Info
    KeyFrame* mpLastKeyFrame;// 上一关键帧
    unsigned int mnLastKeyFrameId;
    unsigned int mnLastRelocFrameId;// 上一次重定位的那一帧的ID
    double mTimeStampLost;// 记录跟踪丢失的时间
    double time_recently_lost;  //当前帧距离跟丢帧的时间，默认5s

    unsigned int mnFirstFrameId;//所有地图的第一帧ID
    unsigned int mnInitialFrameId;//当前地图创建时第1个关键帧的id，它是在上一个地图最大关键帧id的基础上增加1
    unsigned int mnLastInitFrameId;

    bool mbCreatedMap;//是否创建了地图,track()函数的CreateMapInAtlas()里面设置为true

   
    bool mbVelocity{false};//恒速模型追踪时是否有速度
    Sophus::SE3f mVelocity;//恒速模型追踪时用的速度，即上一帧到当前帧的位姿变换

    //Color order (true RGB, false BGR, ignored if grayscale)
    bool mbRGB;

    list<MapPoint*> mlpTemporalPoints;//追踪时添加的临时MapPoint，之后在CreateNewKeyFrame之前会全部删除
    list<MapLine*> mlpTemporalLines;

    //int nMapChangeIndex;

    int mnNumDataset;

    ofstream f_track_stats;

    ofstream f_track_times;
    double mTime_PreIntIMU;
    double mTime_PosePred;
    double mTime_LocalMapTrack;
    double mTime_NewKF_Dec;

    GeometricCamera* mpCamera, *mpCamera2;//KB8模型的双目才有mpCamera2，见tracking类构造函数从setting类或配置文件加载参数

    int initID, lastID;

    Sophus::SE3f mTlr;

    void newParameterLoader(Settings* settings);

#ifdef REGISTER_LOOP
    bool Stop();

    bool mbStopped;
    bool mbStopRequested;
    bool mbNotStop;
    std::mutex mMutexStop;
#endif

public:
    cv::Mat mImRight;
};

} //namespace ORB_SLAM

#endif // TRACKING_H
