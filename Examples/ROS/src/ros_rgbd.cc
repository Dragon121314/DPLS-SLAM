/**
* This file is part of ORB-SLAM3
*
* Copyright (C) 2017-2020 Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
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


#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>

#include<ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include<opencv2/core/core.hpp>
#include"../../../include/System.h"
using namespace std;

class ImageGrabber
{
public:
    ImageGrabber(ORB_SLAM3::System* pSLAM):mpSLAM(pSLAM)
    {
        tim=0.0;
        num=0;
    }

    void GrabRGBD(const sensor_msgs::ImageConstPtr& msgRGB,const sensor_msgs::ImageConstPtr& msgD,const sensor_msgs::ImageConstPtr& Instance);

    ORB_SLAM3::System* mpSLAM;
    double tim; //运行时间
    int num;  //处理数量

    void onShutdown(double t)
    {
        cout<<"c++: 数量： "<<num<<" all time: "<<tim<<endl;
        cout<<"平均运行时间"<<1000*tim/num<<" ms"<<endl;
        cout<<"总平均运行时间"<<1000*t/num<<" ms"<<endl;
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "RGBD");
    ros::start();
    ros::NodeHandle nh("~");
    // if(argc != 3)
    // {
    //     cerr << endl << "Usage: rosrun ORB_SLAM3 RGBD path_to_vocabulary path_to_settings" << endl;        
    //     ros::shutdown();
    //     return 1;
    // }    
    std::string voc_file, config_file;
    nh.param<std::string>("voc_file", voc_file, " ");
    nh.param<std::string>("config_file", config_file, " ");//~ 前缀，表示参数在当前节点的私有命名空间中。
        // 检查参数是否被正确设置
    if (voc_file.empty() || config_file.empty()) {
        ROS_ERROR("Failed to read voc_file or config_file parameters.");
        ros::shutdown();
        return 1;
    }

    // 打印参数值以确认
    ROS_INFO("voc_file: %s", voc_file.c_str());
    ROS_INFO("config_file: %s", config_file.c_str());
    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM3::System SLAM(voc_file,config_file,ORB_SLAM3::System::RGBD,true);

    ImageGrabber igb(&SLAM);

    ros::Time start_time=ros::Time::now();
    // message_filters::Subscriber<sensor_msgs::Image> rgb_sub(nh, "/camera/rgb/image_raw", 100);
    // message_filters::Subscriber<sensor_msgs::Image> depth_sub(nh, "/camera/depth_registered/image_raw", 100);
    // 以下为tum rgbd 数据集的topic
    message_filters::Subscriber<sensor_msgs::Image> rgb_sub(nh, "/camera/color/image_raw", 10000);
    message_filters::Subscriber<sensor_msgs::Image> depth_sub(nh, "/camera/aligned_depth_to_color/image_raw", 10000);
    message_filters::Subscriber<sensor_msgs::Image> instance_sub(nh, "/yolo/mask", 10000);
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::Image> sync_pol;
    message_filters::Synchronizer<sync_pol> sync(sync_pol(1000), rgb_sub,depth_sub,instance_sub);
    sync.registerCallback(boost::bind(&ImageGrabber::GrabRGBD,&igb,_1,_2,_3));

    // // 注册关闭回调
    // ros::Timer shutdown_timer = nh.createTimer(ros::Duration(1.0), boost::bind(&ImageGrabber::onShutdown, &igb, _1), true);
    ros::Rate rate(100);
    while(ros::ok())
    {
        ros::spinOnce();
        rate.sleep();
    }
    ros::Duration duration=ros::Time::now()-start_time ;
    double t=duration.toSec();
    igb.onShutdown(t);
    // Stop all threads
    SLAM.Shutdown();

    // Save camera trajectory
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

    ros::shutdown();

    return 0;
}

void ImageGrabber::GrabRGBD(const sensor_msgs::ImageConstPtr& msgRGB,const sensor_msgs::ImageConstPtr& msgD,const sensor_msgs::ImageConstPtr& Instance)
{
    // Copy the ros image message to cv::Mat.
    ros::Time start_time=ros::Time::now();
    cv_bridge::CvImageConstPtr cv_ptrRGB;
    try
    {
        cv_ptrRGB = cv_bridge::toCvShare(msgRGB);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv_bridge::CvImageConstPtr cv_ptrD;
    try
    {
        cv_ptrD = cv_bridge::toCvShare(msgD);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    cv_bridge::CvImageConstPtr cv_ptrI;
    try
    {
        cv_ptrI = cv_bridge::toCvShare(Instance);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    mpSLAM->SetInstance(cv_ptrI->image);
    

    mpSLAM->TrackRGBD(cv_ptrRGB->image,cv_ptrD->image,cv_ptrRGB->header.stamp.toSec());

    ros::Time end_time=ros::Time::now();
    ros::Duration duration = end_time - start_time;
    tim+=duration.toSec();
    num++;
}


