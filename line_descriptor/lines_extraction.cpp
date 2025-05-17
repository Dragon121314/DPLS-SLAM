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
using namespace LineDescriptor;
// using namespace line_descriptor;
using namespace std;

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
  // string pathSeq="/home/robot/Datasets/EuRoC/V1_02_medium";
  string pathSeq="/home/robot/Datasets/TUM/rgbd_dataset_freiburg3_walking_rpy/";
  string pathCam0 = pathSeq + "/mav0/cam0/data";
  string pathCam1 = pathSeq + "/mav0/cam1/data";
  string pathTimeStamps="/home/robot/ORB_SLAM/ORB-LINE-SLAM/Examples/Stereo-Line/EuRoC_TimeStamps/V102.txt";
  vector<string>  vstrImageLeft;
  vector<string>  vstrImageRight;
  vector<double> vTimestampsCam;
    string strAssociationFilename ="/home/robot/Datasets/TUM/rgbd_dataset_freiburg3_walking_rpy/associate.txt";

  // LoadImages(pathCam0, pathCam1, pathTimeStamps, vstrImageLeft, vstrImageRight, vTimestampsCam);
  LoadImagesRGB(strAssociationFilename, vstrImageLeft, vstrImageRight, vTimestampsCam);
  cv::Mat imageMat = imread(pathSeq+vstrImageLeft[0],cv::IMREAD_UNCHANGED);
  cv::Mat ed_output = imageMat.clone();
  float max=imageMat.cols>imageMat.rows?imageMat.cols:imageMat.rows;

//   // LoadImages(pathCam0, pathCam1, pathTimeStamps, vstrImageLeft, vstrImageRight, vTimestampsCam);
//   /* get parameters from comand line */
vector<float> fac;
vector<float> tt;

// for(int ii=0; ii<10;ii++)
// {

//   BinaryDescriptor::EDLineParam param;
//   param.anchorThreshold=ii;
//   param.gradientThreshold=18;
//   param.ksize=5;
//   param.lineFitErrThreshold=1.4;
//   param.minLineLen=15;
//   param.scanIntervals=2;
//   param.sigma=1.0;
  /* create a pointer to a BinaryDescriptor object with deafult parameters */
  // Ptr<BinaryDescriptor> bd = BinaryDescriptor::createBinaryDescriptor(param);
  Ptr<BinaryDescriptor> bd = BinaryDescriptor::createBinaryDescriptor();
  Ptr<cv::line_descriptor::BinaryDescriptor> cvbd = cv::line_descriptor::BinaryDescriptor::createBinaryDescriptor();
  Ptr<LSDDetector> lsd = LSDDetector::createLSDDetector();
  /* create a structure to store extracted lines */
  vector<KeyLine> lsd_lines,ed_lines;
  vector<cv::line_descriptor::KeyLine> cved_lines;


//   float factor=0.0;
//         int ed_num_th=0;
//   int all=0;
//   float ed_time=0;
//   for(int j=10;j<60;j++)
//   {
//       imageMat = imread(pathSeq+vstrImageLeft[j],cv::IMREAD_UNCHANGED);
//       ed_output = imageMat.clone();

//         std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
//         bd->detect( imageMat, ed_lines);
//         std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
//         ed_time+=std::chrono::duration_cast<std::chrono::duration<float> >(t2 - t1).count();
//         vector<float> ed_len(ed_lines.size());

//           float length_th=35;
//         all+=ed_lines.size();
//         for ( size_t i = 0; i < ed_lines.size(); i++ )
//         {
//           KeyLine kl = ed_lines[i];
//           ed_len[i]=kl.lineLength;
//           if( kl.octave == 0&&kl.lineLength>length_th)
//           {
//             /* get a random color */
//             int R = ( rand() % (int) ( 255 + 1 ) );
//             int G = ( rand() % (int) ( 255 + 1 ) );
//             int B = ( rand() % (int) ( 255 + 1 ) );
//             /* get extremes of line */
//             Point pt1 = Point2f( kl.startPointX, kl.startPointY );
//             Point pt2 = Point2f( kl.endPointX, kl.endPointY );
//             /* draw line */
//             line( ed_output, pt1, pt2, Scalar( B, G, R ), 2 );
//             factor+=kl.lineLength/max;
//             ed_num_th++;
//           }
//         }
//           imshow("lsd and ed",ed_output);
//         waitKey(1);
//   }
//   cout<<"ori:"<<all/50<<" cull:"<<ed_num_th/50<<" factor:"<<factor/50<<" gradth:"<<param.anchorThreshold<<" time:"<<ed_time/50<<endl;
//   fac.push_back(factor/50);
//   tt.push_back(ed_time/50);
//   imshow("lsd and ed",ed_output);
//   waitKey(1);
// }
// std::ofstream outFile("example.txt");

// for(size_t i=0,ie=tt.size(); i<ie; i++)
// {
//   outFile << fac[i]<<" "<<tt[i] << std::endl;
// }
// outFile.close();

for(size_t ii=78 ;ii<vstrImageLeft.size();ii++)
{
  /* load image */
  cv::Mat imageMat = imread(pathSeq+vstrImageLeft[ii],cv::IMREAD_UNCHANGED);
  // cout<<"the original channels "<<imageMat.channels()<<endl;
  if(imageMat.data == NULL)
  {
    std::cout << "Error, image could not be loaded. Please, check its path" << std::endl;
    return -1;
  }
  // cv::imshow("er",imageMat);
  lsd_lines.clear();
  ed_lines.clear();
  cved_lines.clear();
  float max=imageMat.cols>imageMat.rows?imageMat.cols:imageMat.rows;

  cv::Mat mask = Mat::ones( imageMat.size(), CV_8UC1 );
  /* extract lines */
  cv::Mat lsd_output = imageMat.clone();
  cv::Mat ed_output = imageMat.clone();
  cv::Mat cved_output = imageMat.clone();
  std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
  bd->detect( imageMat, ed_lines);
  cvbd->detect( imageMat, cved_lines);
  std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
  lsd->detect( imageMat, lsd_lines, 2, 1, mask );
  std::chrono::steady_clock::time_point t3 = std::chrono::steady_clock::now();
  double lsd_time=std::chrono::duration_cast<std::chrono::duration<double> >(t3 - t2).count();
  double ed_time=std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();

  // cout<<"the edlines :"<<ed_lines.size()<<endl;
  /* draw lines extracted from octave 0 */
  cvtColor(lsd_output,lsd_output, COLOR_BGR2GRAY);
  cvtColor(ed_output,ed_output, COLOR_BGR2GRAY);
  cvtColor(cved_output,cved_output, COLOR_BGR2GRAY);
  if( lsd_output.channels() == 1 ) cvtColor( lsd_output, lsd_output, COLOR_GRAY2BGR );
  if( ed_output.channels() == 1 ) cvtColor( ed_output, ed_output, COLOR_GRAY2BGR );
  if( cved_output.channels() == 1 ) cvtColor( cved_output, cved_output, COLOR_GRAY2BGR );

  vector<float> lsd_len(lsd_lines.size());
  float length_th=40;
  int lsd_num_th=0;
  int ed_num_th=0;
  for(size_t i = 0; i < lsd_lines.size(); i++ )
  {
    KeyLine kl = lsd_lines[i];
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
      line( lsd_output, pt1, pt2, Scalar( 0, 0, 255 ), 2 );
      // line( lsd_output, pt1, pt2, Scalar( B, G, R ), 2 );
      lsd_num_th++;
    }
  }
  vector<float> ed_len(ed_lines.size());
  float factor=0.0;
  for ( size_t i = 0; i < ed_lines.size(); i++ )
  {
    KeyLine kl = ed_lines[i];
    ed_len[i]=kl.lineLength;
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
      line( ed_output, pt1, pt2, Scalar( 0, 255, 0), 2 );
      // line( ed_output, pt1, pt2, Scalar( B, G, R ), 2 );
      ed_num_th++;
      factor+=kl.lineLength/max;
    }
  }

  vector<float> cved_len(cved_lines.size());
  // float factor=0.0;
  for ( size_t i = 0; i < cved_lines.size(); i++ )
  {
    cv::line_descriptor::KeyLine kl = cved_lines[i];
    cved_len[i]=kl.lineLength;
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
      line( cved_output, pt1, pt2, Scalar( 0, 255, 0), 2 );
      // line( ed_output, pt1, pt2, Scalar( B, G, R ), 2 );
      ed_num_th++;
      factor+=kl.lineLength/max;
    }
  }
  // cout<<"imageID: "<<ii<<" orig lsd: "<<lsd_lines.size()<<" orig ed: "<<ed_lines.size();
  // cout<<" culling lsd: "<<lsd_num_th<<" culling ed: "<<ed_num_th<<" lsd time: "<<lsd_time<<" ed time: "<<ed_time<<endl;
  // cout<<"response:"<<factor<<endl;
  // sort(len.begin(),len.end());
  // for(size_t i=0;i<lines.size();i++) cout<<"the line id: "<< i <<" the length : "<<len[i]<<endl;
  /* show lines on image */
  // imshow( "Lines", output );

//   Mat gray=imageMat.clone();
//     // cvtColor(imageMat,gray,COLOR_BGR2GRAY);
//     //设置提取直方图的相关变量
//     Mat hist;//用于存放直方图计算结果
//     const int channels[1]={0};//通道索引.对于彩色图像，可以是 {0, 1, 2}，分别对应于蓝色、绿色和红色通道。
//     float inRanges[2]={0,255};
//     const float*ranges[1]={inRanges};//像素灰度值范围
//     const int bins[1]={256};//直方图的维度，其实就是像素灰度值的最大值
//     //1：这是图像通道的数量。1 表示计算单通道图像的直方图。对于灰度图像，这个值通常是 1。对于彩色图像，如果分别计算每个颜色通道的直方图，这个值可以是 3（对应于 BGR 通道）。
//     //mat()掩码用于指定图像中哪些区域应该被包含在直方图计算中
//     //1：这是直方图的维度。在这里，1 表示直方图是一维的。
//     //bins：这是直方图的桶（bins）数量。桶的数量决定了直方图的分辨率。对于8位图像，bins 通常是 256，因为 8 位图像有 256 个可能的像素值。
//     //ranges：这是一个包含两个元素的数组，定义了直方图计算的值的范围。对于8位图像，这个数组通常是{0,256}，表示考虑所有可能的像素值。
//     // calcHist(&gray,1,channels,Mat(),hist,1,bins,ranges);//计算图像直方图

//         // cv::normalize(hist, hist, 0, 1, cv::NORM_L1);
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

//   // imshow("tidu",grad);
//   cv::Mat hist1;
//   // calcHist(&grad, 1, 0, cv::Mat(), hist1, 1,bins,ranges, true, false);
// //========================================================================

//   // //准备绘制直方图
//   // int hist_w=512;
//   // int hist_h=400;
//   // int width=2;
//   // Mat histImage1=Mat::zeros(hist_h,hist_w,CV_8UC3);
//   // Mat histImage2=Mat::zeros(hist_h,hist_w,CV_8UC3);

//   // for(int i=1;i<=hist.rows;++i)
//   // {
//   //     rectangle(histImage1,Point(width*(i-1),hist_h-1),Point(width*i-1,hist_h-cvRound(hist.at<float>(i-1)/20)),Scalar(255,255,255),-1);
//   // }
//   // for(int i=1;i<=hist1.rows;++i)
//   // {
//   //     rectangle(histImage2,Point(width*(i-1),hist_h-1),Point(width*i-1,hist_h-cvRound(hist1.at<float>(i-1)/20)),Scalar(255,255,255),-1);
//   // }
//   Mat toShow;
//   // resize(lsd_output,lsd_output,Size(512,400));
//   // resize(ed_output,ed_output,Size(512,400));

//   // cvtColor(histImage1, histImage1, COLOR_BGR2GRAY );
//   // cout<<"the image size "<<output.size().width<<" "<<output.size().height<<" "<<output.channels()<<endl;
//   // cout<<"the image size "<<histImage1.size().width<<" "<<histImage1.size().height<<" "<<histImage1.channels()<<endl;
//   // hconcat(histImage1,histImage2,toShow);
//   // imshow("hist and dx",toShow);
  cv::Mat out,output;
  hconcat(lsd_output,cved_output,out);
  hconcat(out,ed_output,output);
  imshow("lsd and ed",output);

  waitKey(0);
}
 




}