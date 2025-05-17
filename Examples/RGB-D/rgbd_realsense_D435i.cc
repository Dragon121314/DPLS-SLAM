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

#include <signal.h>
#include <stdlib.h>
#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>
#include <ctime>
#include <sstream>
#include <condition_variable>
#include <opencv2/core/core.hpp>
#include <librealsense2/rs.hpp>
#include "librealsense2/rsutil.h"
#include <System.h>

using namespace std;
bool b_continue_session;

void exit_loop_handler(int s)
{
    cout << "Finishing session" << endl;
    b_continue_session = false;
}

rs2_stream find_stream_to_align(const std::vector<rs2::stream_profile>& streams);
bool profile_changed(const std::vector<rs2::stream_profile>& current, const std::vector<rs2::stream_profile>& prev);
void interpolateData(const std::vector<double> &vBase_times,std::vector<rs2_vector> &vInterp_data, std::vector<double> &vInterp_times,
                     const rs2_vector &prev_data, const double &prev_time);

rs2_vector interpolateMeasure(const double target_time,const rs2_vector current_data, const double current_time,
                              const rs2_vector prev_data, const double prev_time);

//显示传感器可支持的属性选项，在change_sensor_option()函数中可以改变选项的值
static rs2_option get_sensor_option(const rs2::sensor& sensor)
{
    //传感器通常有几个选项来控制其属性，如曝光、亮度等。
    std::cout << "Sensor supports the following options:\n" << std::endl;
    // Starting from 0 until RS2_OPTION_COUNT (exclusive)
    for (int i = 0; i < static_cast<int>(RS2_OPTION_COUNT); i++)//以下循环显示了如何迭代所有可用选项
    {
        rs2_option option_type = static_cast<rs2_option>(i);
        //SDK enum types can be streamed to get a string that represents them
        std::cout << "  " << i << ": " << option_type;
        // To control an option, use the following api:
        // First, verify that the sensor actually supports this option
        if (sensor.supports(option_type))
        {
            std::cout << std::endl;
            // Get a human readable description of the option
            const char* description = sensor.get_option_description(option_type);
            std::cout << "       Description   : " << description << std::endl;
            // Get the current value of the option
            float current_value = sensor.get_option(option_type);
            std::cout << "       Current Value : " << current_value << std::endl;
            //To change the value of an option, please follow the change_sensor_option() function
        }
        else std::cout << " is not supported" << std::endl;
    }
    uint32_t selected_sensor_option = 0;
    return static_cast<rs2_option>(selected_sensor_option);
}

int main(int argc, char **argv) 
{
    string strVocabulary="../Vocabulary/ORBvoc.txt";
    string strSettings="../Examples/RGB-D/RealSense_D435i.yaml";
    string file_name=""; //trajectory_file_name,应该时加载以前的轨迹即之前建立好的地图
    bool bFileName = false;//file_name有则为ture
    struct sigaction sigIntHandler;//sigaction 类型的结构体变量，用于设置信号处理程序
    sigIntHandler.sa_handler = exit_loop_handler;//将信号处理程序 exit_loop_handler 赋值给 sigaction 结构体中的 sa_handler 字段，用于处理 SIGINT 信号。
    sigemptyset(&sigIntHandler.sa_mask);//初始化 sa_mask 字段为一个空的信号集，即在处理 SIGINT 信号时，不阻塞其他信号。
    sigIntHandler.sa_flags = 0;//信号处理标志设置为 0，表示使用默认行为（不设置特殊选项）。
    sigaction(SIGINT, &sigIntHandler, NULL);//将前面定义的 sigIntHandler 作为 SIGINT 信号的处理程序，并将处理结果应用到当前进程。
    b_continue_session = true;
    double offset = 0; // ms

    rs2::context ctx;//创建一个 rs2::context 对象，用于管理和访问 RealSense 硬件和相关功能。
    rs2::device_list devices = ctx.query_devices();//使用ctx.query_devices()查询连接到计算机上的所有RealSense设备，并将结果存储在devices对象中。
    rs2::device selected_device;//定义一个rs2::device对象selected_device，可以用来引用特定的RealSense设备
    if (devices.size() == 0)
    {
        std::cerr << "No device connected, please connect a RealSense device" << std::endl;
        return 0;
    }
    else selected_device = devices[0];

    std::vector<rs2::sensor> sensors = selected_device.query_sensors();//获取selected_device上的所有传感器，并将它们存储在sensors向量中
    int index = 0;
    for (rs2::sensor sensor : sensors) // We can now iterate the sensors and print their names
        if (sensor.supports(RS2_CAMERA_INFO_NAME)) //检查传感器是否支持获取相机名称的信息（通常用于检查传感器是否具备某些功能）。
        {
            ++index;
            if (index == 1) 
            {
                sensor.set_option(RS2_OPTION_ENABLE_AUTO_EXPOSURE, 1);//启用自动曝光功能，将其设置为 1（启用）
                sensor.set_option(RS2_OPTION_AUTO_EXPOSURE_LIMIT,50000);//设置自动曝光的限制时间为50,000微秒（50毫秒）
                sensor.set_option(RS2_OPTION_EMITTER_ENABLED, 1); //启用传感器的发射器功能，将其设置为 1（启用）。 打开发射器获取深度信息
            }
            get_sensor_option(sensor);
            if (index == 2)
            {
                // RGB camera
                sensor.set_option(RS2_OPTION_EXPOSURE,80.f);//将传感器的曝光时间设置为80毫秒。

            }
            if (index == 3)
            {
                sensor.set_option(RS2_OPTION_ENABLE_MOTION_CORRECTION,0);//禁用运动校正功能，将其设置为0（禁用）
            }

        }
    rs2::pipeline pipe;// Declare RealSense pipeline, 封装实际设备和传感器，用于管理和处理数据流
    rs2::config cfg;// Create a configuration for configuring the pipeline with a non default profile
    // RGB stream
    cfg.enable_stream(RS2_STREAM_COLOR,640, 480, RS2_FORMAT_RGB8, 30);//配置管道以启用彩色流，分辨率为640x480，格式为RGB8，帧率为30帧每秒。
    // Depth stream
    // cfg.enable_stream(RS2_STREAM_INFRARED, 1, 640, 480, RS2_FORMAT_Y8, 30);
    cfg.enable_stream(RS2_STREAM_DEPTH,640, 480, RS2_FORMAT_Z16, 30);//配置管道以启用深度流，分辨率为640x480，格式为Z16（16位深度图），帧率为30帧每秒
    // IMU stream
    cfg.enable_stream(RS2_STREAM_ACCEL, RS2_FORMAT_MOTION_XYZ32F); //, 250);// 63 //配置管道以启用加速度计流，格式为 XYZ32F（32位浮点数的加速度数据）
    cfg.enable_stream(RS2_STREAM_GYRO, RS2_FORMAT_MOTION_XYZ32F); //, 400);  //配置管道以启用陀螺仪流，格式为 XYZ32F（32位浮点数的角速度数据）
    std::mutex imu_mutex;
    std::condition_variable cond_image_rec;//是C++11标准库中提供的一种线程同步机制，用于线程间的等待和通知。它可以与std::mutex一起使用以协调多个线程之间的执行顺序
    vector<double> v_accel_timestamp;//加速度计时间戳
    vector<rs2_vector> v_accel_data;//加速度计数据
    vector<double> v_gyro_timestamp;//陀螺仪时间戳
    vector<rs2_vector> v_gyro_data;//陀螺仪数据

    double prev_accel_timestamp = 0;//前一个加速度计时间戳
    rs2_vector prev_accel_data;//前一个加速度数据
    double current_accel_timestamp = 0;//当前加速度计时间戳
    rs2_vector current_accel_data;//当前的加速度数据
    vector<double> v_accel_timestamp_sync;//异步加速度时间戳，即加速度时间戳少于陀螺仪的数据，则将当前加速度数据加入直到两个数据相等
    vector<rs2_vector> v_accel_data_sync;//异步加速度数据，即加速度时间戳少于陀螺仪的数据时，则将当前加速度数据加入直到两个数据相等

    cv::Mat imCV, depthCV;//OpenCV彩色图和深度图
    int width_img, height_img;//图像的宽高
    double timestamp_image = -1.0;
    bool image_ready = false;//ture指示图像数据已经准备好
    int count_im_buffer = 0; // count dropped frames  计数掉帧

    // start and stop just to get necessary profile
    rs2::pipeline_profile pipe_profile = pipe.start(cfg);//启动RealSense管道pipe并使用配置cfg，返回的pipe_profile包含当前管道的配置和流信息
    pipe.stop();//停止管道pipe，关闭数据流

    // Align depth and RGB frames。。。。管道可以选择没有颜色流的设备，如果没有颜色流则选择将深度与另一个流对齐
    rs2_stream align_to = find_stream_to_align(pipe_profile.get_streams());//从当前管道配置中获取流信息，找到需要对齐的流（例如深度流与RGB流）
    rs2::align align(align_to); //rs2::align允许我们将深度帧与其他帧对齐，align_to是我们指定的对齐深度帧的流类型 
    rs2::frameset fsSLAM;//定义一个frameset对象fsSLAM，用于存储来自RealSense相机的图像和深度帧。
    //定义一个IMU数据回调函数imu_callback，这个函数将在IMU数据到达时被调用。[&]表示这个回调函数可以访问外部变量
    auto imu_callback = [&](const rs2::frame& frame) 
    {
        std::unique_lock<std::mutex> lock(imu_mutex);
        if(rs2::frameset fs = frame.as<rs2::frameset>())//将传入的frame对象转换为frameset对象fs，如果转换成功则进入条件语句内部。
        {
            count_im_buffer++;
            double new_timestamp_image = fs.get_timestamp()*1e-3;//获取当前frameset的时间戳（单位：毫秒），并转换为秒（时间戳乘以1e-3）
            if(abs(timestamp_image-new_timestamp_image)<0.001)//当前帧与上一帧时间戳差异小于1毫秒，认为数据重复不处理
            {
                count_im_buffer--;
                return;
            }
            if(profile_changed(pipe.get_active_profile().get_streams(), pipe_profile.get_streams()))//检查当前管道配置是否与之前的配置pipe_profile不同
            {
                //如果配置发生了变化，更新pipe_profile，重新确定需要对齐的流，并重新创建rs2::align对象align。
                pipe_profile = pipe.get_active_profile();
                align_to = find_stream_to_align(pipe_profile.get_streams());
                align = rs2::align(align_to);
            }
            //对齐深度和rgb需要很长时间，将其移出中断状态以避免丢失IMU测量值
            fsSLAM = fs;

            /*
            //Get processed aligned frame
            auto processed = align.process(fs);

            // Trying to get both other and aligned depth frames
            rs2::video_frame color_frame = processed.first(align_to);
            rs2::depth_frame depth_frame = processed.get_depth_frame();
            //If one of them is unavailable, continue iteration
            if (!depth_frame || !color_frame) {
                cout << "Not synchronized depth and image\n";
                return;
            }
            imCV = cv::Mat(cv::Size(width_img, height_img), CV_8UC3, (void*)(color_frame.get_data()), cv::Mat::AUTO_STEP);
            depthCV = cv::Mat(cv::Size(width_img, height_img), CV_16U, (void*)(depth_frame.get_data()), cv::Mat::AUTO_STEP);
            cv::Mat depthCV_8U;
            depthCV.convertTo(depthCV_8U,CV_8U,0.01);
            cv::imshow("depth image", depthCV_8U);*/

            timestamp_image = fs.get_timestamp()*1e-3;
            image_ready = true;
            while(v_gyro_timestamp.size() > v_accel_timestamp_sync.size())//检查陀螺仪时间戳的数量是否多于加速度计时间戳的数量，即对齐陀螺仪与加速度计数据
            {
                int index = v_accel_timestamp_sync.size();
                double target_time = v_gyro_timestamp[index];
                v_accel_data_sync.push_back(current_accel_data);
                v_accel_timestamp_sync.push_back(target_time);
            }
            lock.unlock();
            cond_image_rec.notify_all();//调用cond_image_rec的notify_all方法，用于通知所有等待该条件变量的线程某个事件已发生
        }
    };
    pipe_profile = pipe.start(cfg, imu_callback);//启动摄像头流，配置为cfg并指定IMU回调函数imu_callback
    rs2::stream_profile cam_stream = pipe_profile.get_stream(RS2_STREAM_COLOR);//获取颜色流的配置资料
    rs2_intrinsics intrinsics_cam = cam_stream.as<rs2::video_stream_profile>().get_intrinsics();//获取摄像头的内在参数（如焦距和畸变系数）。
    width_img = intrinsics_cam.width;
    height_img = intrinsics_cam.height;
    std::cout << " fx = " << intrinsics_cam.fx << std::endl;
    std::cout << " fy = " << intrinsics_cam.fy << std::endl;
    std::cout << " cx = " << intrinsics_cam.ppx << std::endl;
    std::cout << " cy = " << intrinsics_cam.ppy << std::endl;
    std::cout << " height = " << intrinsics_cam.height << std::endl;
    std::cout << " width = " << intrinsics_cam.width << std::endl;
    std::cout << " Coeff = " << intrinsics_cam.coeffs[0] << ", " << intrinsics_cam.coeffs[1] << ", " <<
    intrinsics_cam.coeffs[2] << ", " << intrinsics_cam.coeffs[3] << ", " << intrinsics_cam.coeffs[4] << ", " << std::endl;
    std::cout << " Model = " << intrinsics_cam.model << std::endl;

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM3::System SLAM(strVocabulary,strSettings,ORB_SLAM3::System::RGBD, true, 0, file_name);
    float imageScale = SLAM.GetImageScale();//图像缩放因子
    double timestamp;
    cv::Mat im, depth;
    double t_resize = 0.f;
    double t_track = 0.f;
    rs2::frameset fs;
    while (!SLAM.isShutDown())
    {
        {
            std::unique_lock<std::mutex> lk(imu_mutex);
            if(!image_ready) cond_image_rec.wait(lk);//线程会在此调用上等待，直到条件变量被通知或超时。
            std::chrono::steady_clock::time_point time_Start_Process = std::chrono::steady_clock::now();
            fs = fsSLAM;
            if(count_im_buffer>1) cout << count_im_buffer-1 << " dropped frs\n";
            count_im_buffer = 0;
            timestamp = timestamp_image;
            im = imCV.clone();
            depth = depthCV.clone();
            image_ready = false;
        }
        auto processed = align.process(fs);//对捕获的图像帧fs进行对齐处理，返回处理后的结果
        rs2::video_frame color_frame = processed.first(align_to);//提取对齐后的彩色图
        rs2::depth_frame depth_frame = processed.get_depth_frame();//提取对其后的深度图
        im=cv::Mat(cv::Size(width_img, height_img), CV_8UC3, (void*)(color_frame.get_data()), cv::Mat::AUTO_STEP);//将颜色帧数据转换为OpenCV的cv::Mat格式。
        depth=cv::Mat(cv::Size(width_img, height_img), CV_16U, (void*)(depth_frame.get_data()), cv::Mat::AUTO_STEP);//将深度帧数据转换为OpenCV的cv::Mat格式
        /*cv::Mat depthCV_8U;
        depthCV.convertTo(depthCV_8U,CV_8U,0.01);
        cv::imshow("depth image", depthCV_8U);*/
        if(imageScale != 1.f)
        {
#ifdef REGISTER_TIMES
            std::chrono::steady_clock::time_point t_Start_Resize = std::chrono::steady_clock::now();
#endif
            int width = im.cols * imageScale;
            int height = im.rows * imageScale;
            cv::resize(im, im, cv::Size(width, height));
            cv::resize(depth, depth, cv::Size(width, height));
#ifdef REGISTER_TIMES
            std::chrono::steady_clock::time_point t_End_Resize = std::chrono::steady_clock::now();
            t_resize = std::chrono::duration_cast<std::chrono::duration<double,std::milli> >(t_End_Resize - t_Start_Resize).count();
            SLAM.InsertResizeTime(t_resize);
#endif
        }

#ifdef REGISTER_TIMES
        std::chrono::steady_clock::time_point t_Start_Track = std::chrono::steady_clock::now();
#endif
        // Pass the image to the SLAM system
        SLAM.TrackRGBD(im, depth, timestamp); //, vImuMeas); depthCV
#ifdef REGISTER_TIMES
        std::chrono::steady_clock::time_point t_End_Track = std::chrono::steady_clock::now();
        t_track = t_resize + std::chrono::duration_cast<std::chrono::duration<double,std::milli> >(t_End_Track - t_Start_Track).count();
        SLAM.InsertTrackTime(t_track);
#endif
    }
    cout << "System shutdown!\n";
}

//给定一个流向量，将深度流与另一个流对其，优先使用颜色流(RGB图)，没有则采用另一个流
rs2_stream find_stream_to_align(const std::vector<rs2::stream_profile>& streams)
{
    rs2_stream align_to = RS2_STREAM_ANY;//指定要对齐的流类型，RS2_STREAM_ANY表示不特定的流
    bool depth_stream_found = false;
    bool color_stream_found = false;
    for (rs2::stream_profile sp : streams)//码遍历streams容器中的每一个 rs2::stream_profile 对象
    {
        rs2_stream profile_stream = sp.stream_type();//提取流的类型
        if (profile_stream != RS2_STREAM_DEPTH)
        {
            if (!color_stream_found) align_to = profile_stream;
            if (profile_stream == RS2_STREAM_COLOR) color_stream_found = true;
        }
        else depth_stream_found = true;
    }
    if(!depth_stream_found) throw std::runtime_error("No Depth stream available");
    //检查align_to是否等于RS2_STREAM_ANY，如果是则抛出一个运行时异常，说明没有找到要对齐的流。
    if (align_to == RS2_STREAM_ANY) throw std::runtime_error("No stream found to align with Depth");
    return align_to;
}

//检查当前的流配置是否与之前的配置相同，返回ture表示配置发生改变
bool profile_changed(const std::vector<rs2::stream_profile>& current, const std::vector<rs2::stream_profile>& prev)
{
    for (auto&& sp : prev)
    {
        //If previous profile is in current (maybe just added another)   在current向量中查找与sp具有相同唯一标识符的流配置。itr是查找结果的迭代器
        auto itr = std::find_if(std::begin(current), std::end(current), [&sp](const rs2::stream_profile& current_sp){return sp.unique_id()==current_sp.unique_id();});
        if (itr == std::end(current)) //检查itr是否等于current向量的末尾迭代器，表示未找到相应的流配置。
        {
            return true;
        }
    }
    return false;
}