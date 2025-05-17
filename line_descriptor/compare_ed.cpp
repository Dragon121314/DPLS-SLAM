/*M///////////////////////////////////////////////////////////////////////////////////////
 //
 //  IMPORTANT: READ BEFORE DOWNLOADING, COPYING, INSTALLING OR USING.
 //
 //  By downloading, copying, installing or using the software you agree to this license.
 //  If you do not agree to this license, do not download, install,
 //  copy or use the software.
 //
 //
 //                           License Agreement
 //                For Open Source Computer Vision Library
 //
 // Copyright (C) 2014, Biagio Montesano, all rights reserved.
 // Third party copyrights are property of their respective owners.
 //
 // Redistribution and use in source and binary forms, with or without modification,
 // are permitted provided that the following conditions are met:
 //
 //   * Redistribution's of source code must retain the above copyright notice,
 //     this list of conditions and the following disclaimer.
 //
 //   * Redistribution's in binary form must reproduce the above copyright notice,
 //     this list of conditions and the following disclaimer in the documentation
 //     and/or other materials provided with the distribution.
 //
 //   * The name of the copyright holders may not be used to endorse or promote products
 //     derived from this software without specific prior written permission.
 //
 // This software is provided by the copyright holders and contributors "as is" and
 // any express or implied warranties, including, but not limited to, the implied
 // warranties of merchantability and fitness for a particular purpose are disclaimed.
 // In no event shall the Intel Corporation or contributors be liable for any direct,
 // indirect, incidental, special, exemplary, or consequential damages
 // (including, but not limited to, procurement of substitute goods or services;
 // loss of use, data, or profits; or business interruption) however caused
 // and on any theory of liability, whether in contract, strict liability,
 // or tort (including negligence or otherwise) arising in any way out of
 // the use of this software, even if advised of the possibility of such damage.
 //
 //M*/

#include <iostream>
#include <opencv2/opencv_modules.hpp>

#include <fstream>
#include <opencv2/line_descriptor.hpp>
#include <opencv2/core/utility.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/highgui.hpp>
#include "include/line_descriptor.hpp"
using namespace cv;
using namespace line_descriptor;
using namespace std;
void printMatType(const cv::Mat& mat) 
{
    // 获取 Mat 的类型
    int type = mat.type();

    // 通道数
    int channels = CV_MAT_CN(type);
    // 基本数据类型
    int depth = CV_MAT_DEPTH(type);

    // 输出通道数和数据类型
    std::cout << "Channels: " << channels << std::endl;
    
    // 根据深度输出数据类型
    switch (depth) {
        case CV_8U:  std::cout << "Depth: 8-bit unsigned integer" << std::endl; break;
        case CV_8S:  std::cout << "Depth: 8-bit signed integer" << std::endl; break;
        case CV_16U: std::cout << "Depth: 16-bit unsigned integer" << std::endl; break;
        case CV_16S: std::cout << "Depth: 16-bit signed integer" << std::endl; break;
        case CV_32S: std::cout << "Depth: 32-bit signed integer" << std::endl; break;
        case CV_32F: std::cout << "Depth: 32-bit floating point" << std::endl; break;
        case CV_64F: std::cout << "Depth: 64-bit floating point" << std::endl; break;
        default:     std::cout << "Unknown depth" << std::endl; break;
    }
}
void LoadImages(const string &strPathLeft, const string &strPathRight, const string &strPathTimes,
                vector<string> &vstrImageLeft, vector<string> &vstrImageRight, vector<double> &vTimeStamps)
{
    ifstream fTimes;
    fTimes.open(strPathTimes.c_str());
    vTimeStamps.reserve(5000);
    vstrImageLeft.reserve(5000);
    vstrImageRight.reserve(5000);
    while(!fTimes.eof())
    {
        string s;
        getline(fTimes,s);
        if(!s.empty())
        {
            stringstream ss;
            ss << s;
            vstrImageLeft.push_back(strPathLeft + "/" + ss.str() + ".png");
            vstrImageRight.push_back(strPathRight + "/" + ss.str() + ".png");
            double t;
            ss >> t;
            vTimeStamps.push_back(t/1e9);

        }
    }
}

void LoadImagesRGB(const string &strAssociationFilename, vector<string> &vstrImageFilenamesRGB, vector<string> &vstrImageFilenamesD, vector<double> &vTimestamps)
{
    ifstream fAssociation;
    fAssociation.open(strAssociationFilename.c_str());
    while(!fAssociation.eof())
    {
        string s;
        getline(fAssociation,s);
        if(!s.empty())
        {
            stringstream ss;
            ss << s;
            double t;
            string sRGB, sD;
            ss >> t;
            vTimestamps.push_back(t);
            ss >> sRGB;
            vstrImageFilenamesRGB.push_back(sRGB);
            ss >> t;
            ss >> sD;
            vstrImageFilenamesD.push_back(sD);
        }
    }
}
int main( int argc, char** argv )
{
  // string pathSeq="/home/robot/Datasets/TUM/rgbd_dataset_freiburg3_walking_rpy/";
  string pathSeq="/home/robot/Datasets/TUM/HighDynamic/";
  // string pathSeq="/home/robot/Datasets/EuRoC/V1_02_medium";
  string pathCam0 = pathSeq + "/mav0/cam0/data";
  string pathCam1 = pathSeq + "/mav0/cam1/data";
  string pathTimeStamps="/home/robot/ORB_SLAM/ORB-LINE-SLAM/Examples/Stereo-Line/EuRoC_TimeStamps/V102.txt";
  
  vector<string>  vstrImageLeft;
  vector<string>  vstrImageRight;
  vector<double> vTimestampsCam;
  // string strAssociationFilename ="/home/robot/Datasets/TUM/rgbd_dataset_freiburg3_walking_rpy/associate.txt";
  string strAssociationFilename ="/home/robot/Datasets/TUM/HighDynamic/associate.txt";
  LoadImagesRGB(strAssociationFilename, vstrImageLeft, vstrImageRight, vTimestampsCam);
// 
  // LoadImages(pathCam0, pathCam1, pathTimeStamps, vstrImageLeft, vstrImageRight, vTimestampsCam);
  /* get parameters from comand line */
  /* create a pointer to a BinaryDescriptor object with deafult parameters */
  BinaryDescriptor::Params param;
  param.numOfOctave_ = 2;
  param.widthOfBand_ = 7;
  param.reductionRatio = 2;
  param.ksize_ = 5;
  Ptr<BinaryDescriptor> opencv_ed = BinaryDescriptor::createBinaryDescriptor();
  // Ptr<BinaryDescriptor> opencv_ed = BinaryDescriptor::createBinaryDescriptor(param);
  Ptr<LineDescriptor::BinaryDescriptor> my_ed = LineDescriptor::BinaryDescriptor::createBinaryDescriptor();
  // cout<<opencv_ed->params.widthOfBand_<<
  /* create a structure to store extracted lines */
  vector<KeyLine> opencv_edlines;
  vector<LineDescriptor::KeyLine> my_edlines;

for(size_t ii=0;ii<vstrImageLeft.size();ii++)
{
  // if(ii!=45) continue;
  /* load image */
  cv::Mat imageMat = imread(pathSeq+vstrImageLeft[ii],cv::IMREAD_UNCHANGED);
  // cout<<"the original channels "<<imageMat.channels()<<endl;
  if( imageMat.data == NULL )
  {
    std::cout << "Error, image could not be loaded. Please, check its path" << std::endl;
    return -1;
  }
  opencv_edlines.clear();
  my_edlines.clear();
  cv::Mat mask = Mat::ones(imageMat.size(),CV_8UC1);
  /* extract lines */
  cv::Mat opencv_edimg = imageMat.clone();
  cv::Mat my_edimg = imageMat.clone();
  std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
  opencv_ed->detect(imageMat, opencv_edlines, mask);
  std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
  my_ed->detect(imageMat,my_edlines);
  std::chrono::steady_clock::time_point t3 = std::chrono::steady_clock::now();
  double opencv_time=std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();
  double my_time=std::chrono::duration_cast<std::chrono::duration<double> >(t3- t2).count();

  /* draw lines extracted from octave 0 */
  if( opencv_edimg.channels() == 1 ) cvtColor( opencv_edimg, opencv_edimg, COLOR_GRAY2BGR );
  if( my_edimg.channels() == 1 ) cvtColor( my_edimg, my_edimg, COLOR_GRAY2BGR );

  vector<float> lsd_len(opencv_edlines.size());
  float length_th=40;
  int myed_num_th=0;
  int opencved_num_th=0;
  for(size_t i = 0; i < opencv_edlines.size(); i++ )
  {
    KeyLine kl = opencv_edlines[i];
    lsd_len[i]=kl.lineLength;
    if( kl.octave == 0&&kl.lineLength>length_th)
    {
      /* get a random color */
      int R = ( rand() % (int) ( 255 + 1 ) );
      int G = ( rand() % (int) ( 255 + 1 ) );
      int B = ( rand() % (int) ( 255 + 1 ) );
      /* get extremes of line */
      Point pt1 = Point2f( kl.startPointX, kl.startPointY );
      Point pt2 = Point2f( kl.endPointX, kl.endPointY );
      /* draw line */
      line( opencv_edimg, pt1, pt2, Scalar(B, G, R ), 2 );// B, G, R 
      opencved_num_th++;
    }
  }
  vector<float> ed_len(my_edlines.size());
  for ( size_t i = 0; i < my_edlines.size(); i++ )
  {
    LineDescriptor::KeyLine kl = my_edlines[i];
    ed_len[i]=kl.lineLength;
    if( kl.octave == 0&&kl.lineLength>length_th)
    {
      int R = ( rand() % (int) ( 255 + 1 ) );
      int G = ( rand() % (int) ( 255 + 1 ) );
      int B = ( rand() % (int) ( 255 + 1 ) );
      /* get extremes of line */
      Point pt1 = Point2f( kl.startPointX, kl.startPointY );
      Point pt2 = Point2f( kl.endPointX, kl.endPointY );
      /* draw line */
      line(my_edimg, pt1, pt2, Scalar(B, G, R ), 2 );//B, G, R
      myed_num_th++;
    }
  }
  line(my_edimg, Point2f(0,0), Point2f(0,3), Scalar(0,0,255), 2 );
  cout<<"opencv ed lines: "<<opencv_edlines.size()<<" my ed lines: "<<my_edlines.size();
  cout<<" culling opencved lines: "<<opencved_num_th<<" culling myed lines: "<<myed_num_th;
  cout<<" the opencv time: "<<opencv_time<<" the my time: "<<my_time<<" image id:"<<ii<<endl;
  // sort(len.begin(),len.end());
  // for(size_t i=0;i<lines.size();i++) cout<<"the line id: "<< i <<" the length : "<<len[i]<<endl;
  /* show lines on image */
  // imshow( "Lines", output );

  Mat gray=imageMat.clone();
  //cvtColor(imageMat,gray,COLOR_BGR2GRAY);
  //设置提取直方图的相关变量
  Mat hist;//用于存放直方图计算结果
  const int channels[1]={0};//通道索引.对于彩色图像，可以是 {0, 1, 2}，分别对应于蓝色、绿色和红色通道。
  float inRanges[2]={0,255};
  const float *ranges[1]={inRanges};//像素灰度值范围
  const int bins[1]={256};//直方图的维度，其实就是像素灰度值的最大值
  //1：这是图像通道的数量。1 表示计算单通道图像的直方图。对于灰度图像，这个值通常是 1。对于彩色图像，如果分别计算每个颜色通道的直方图，这个值可以是 3（对应于 BGR 通道）。
  //mat()掩码用于指定图像中哪些区域应该被包含在直方图计算中
  //1：这是直方图的维度。在这里，1 表示直方图是一维的。
  //bins：这是直方图的桶（bins）数量。桶的数量决定了直方图的分辨率。对于8位图像，bins 通常是 256，因为 8 位图像有 256 个可能的像素值。
  //ranges：这是一个包含两个元素的数组，定义了直方图计算的值的范围。对于8位图像，这个数组通常是{0,256}，表示考虑所有可能的像素值。
//   calcHist(&gray,1,channels,Mat(),hist,1,bins,ranges);//计算图像直方图
//   // cv::normalize(hist, hist, 0, 1, cv::NORM_L1);
// //================计算梯度直方图===========================================
//   cv::Mat grad_x, grad_y;
//   cv::Mat abs_grad_x, abs_grad_y, grad;
//   cv::Sobel(imageMat, grad_x, CV_16S, 1, 0);
//   cv::Sobel(imageMat, grad_y, CV_16S, 0, 1);

//   // 计算绝对值
//   cv::convertScaleAbs(grad_x, abs_grad_x);
//   cv::convertScaleAbs(grad_y, abs_grad_y);
//   // imshow("x",abs_grad_x);
//   // imshow("y",abs_grad_y);
//   // 合并梯度
//   abs_grad_x += abs_grad_y;
//   grad = abs_grad_x;

//   imshow("tidu",grad);
//   cv::Mat hist1;
//   calcHist(&grad, 1, 0, cv::Mat(), hist1, 1,bins,ranges, true, false);
// //========================================================================

//   //准备绘制直方图
//   int hist_w=512;
//   int hist_h=400;
//   int width=2;
//   Mat histImage1=Mat::zeros(hist_h,hist_w,CV_8UC3);
//   Mat histImage2=Mat::zeros(hist_h,hist_w,CV_8UC3);

//   for(int i=1;i<=hist.rows;++i)
//   {
//       rectangle(histImage1,Point(width*(i-1),hist_h-1),Point(width*i-1,hist_h-cvRound(hist.at<float>(i-1)/20)),Scalar(255,255,255),-1);
//   }
//   for(int i=1;i<=hist1.rows;++i)
//   {
//       rectangle(histImage2,Point(width*(i-1),hist_h-1),Point(width*i-1,hist_h-cvRound(hist1.at<float>(i-1)/20)),Scalar(255,255,255),-1);
//   }
//   Mat toShow;
//   resize(opencv_edimg,opencv_edimg,Size(512,400));
//   resize(my_edimg,my_edimg,Size(512,400));

  // cvtColor(histImage1, histImage1, COLOR_BGR2GRAY );
  // cout<<"the image size "<<output.size().width<<" "<<output.size().height<<" "<<output.channels()<<endl;
  // cout<<"the image size "<<histImage1.size().width<<" "<<histImage1.size().height<<" "<<histImage1.channels()<<endl;
  // hconcat(histImage1,histImage2,toShow);
  // imshow("hist and dx",toShow);
  cv::Mat output;
  hconcat(opencv_edimg,my_edimg,output);
  imshow("opencv and my",output);

  waitKey(0);
}
 
}


