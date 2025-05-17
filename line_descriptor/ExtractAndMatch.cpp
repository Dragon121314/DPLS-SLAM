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
#include <opencv2/video/tracking.hpp>
#include <opencv2/calib3d.hpp>
#include <eigen3/Eigen/Dense>
#include <opencv2/line_descriptor/descriptor.hpp>


using namespace cv;
using namespace LineDescriptor;
// using namespace line_descriptor;
using namespace std;
float m_inv_K11 = 1.0 / 458.654;
float m_inv_K13 = -367.215 / 458.654;
float m_inv_K22 = 1.0 / 457.296;
float m_inv_K23 = -248.375 / 457.296;

inline float GetPixelValue(const cv::Mat &img, float x, float y) {
    // boundary check
    if (x < 0) x = 0;
    if (y < 0) y = 0;
    if (x >= img.cols) x = img.cols - 1;
    if (y >= img.rows) y = img.rows - 1;
    uchar *data = &img.data[int(y) * img.step + int(x)];
    float xx = x - floor(x);
    float yy = y - floor(y);
    return float(
        (1 - xx) * (1 - yy) * data[0] +
        xx * (1 - yy) * data[1] +
        (1 - xx) * yy * data[img.step] +
        xx * yy * data[img.step + 1]
    );
}

void distortion(const Eigen::Vector2d& p_u, Eigen::Vector2d& d_u)
{
   double k1 = -0.28340811;
    double k2 = 0.07395907;
    double p1 = 0.00019359;
    double p2 = 1.76187114e-05;

    double mx2_u, my2_u, mxy_u, rho2_u, rad_dist_u;

    mx2_u = p_u(0) * p_u(0);
    my2_u = p_u(1) * p_u(1);
    mxy_u = p_u(0) * p_u(1);
    rho2_u = mx2_u + my2_u;
    rad_dist_u = k1 * rho2_u + k2 * rho2_u * rho2_u;
    d_u << p_u(0) * rad_dist_u + 2.0 * p1 * mxy_u + p2 * (rho2_u + 2.0 * mx2_u),
           p_u(1) * rad_dist_u + 2.0 * p2 * mxy_u + p1 * (rho2_u + 2.0 * my2_u);
}

void LoadImages(const string &strPathLeft, const string &strPathRight, const string &strPathTimes,vector<string> &vstrImageLeft, vector<string> &vstrImageRight, vector<double> &vTimeStamps)
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

void ExtractPoint(KeyLine &kl,int &Len,vector<Point2f> &vec)
{
    vector<Point2f> vec11;
    double length=kl.lineLength;
    if(length>=round(0.5*Len))
    {

    }
    else if(length>=round(0.25*Len))
    {

    }
    else
    {
        vec.push_back(Point2f());

    }

}

bool inBorder(const cv::Point2f &pt)
{
    float img_x = pt.x;
    float img_y = pt.y;
    return 0.0 <= img_x && img_x < 752.0  && 0.0 <= img_y && img_y < 480.0 ;
}

void reconstructline(KeyLine &kl,vector<Point2f> &vec, int minlen)
{
    vec.clear();
    if(round(kl.lineLength)>=0.5*minlen)
    {
        Point2f start=Point2f(kl.startPointX,kl.startPointY),end=Point2f(kl.endPointX,kl.endPointY);
        Point2f md1=Point2f((start+kl.pt)/2),md2=Point2f((kl.pt+end)/2);
        vec.push_back(start);vec.push_back((start+md1)/2);vec.push_back(md1);vec.push_back((kl.pt+md1)/2);vec.push_back(kl.pt);
        vec.push_back((kl.pt+md2)/2);vec.push_back(md2);vec.push_back((end+md2)/2);vec.push_back(end);
    }
    else if(round(kl.lineLength)>=0.25*minlen)
    {
        Point2f start=Point2f(kl.startPointX,kl.startPointY),end=Point2f(kl.endPointX,kl.endPointY);
        vec.push_back(start);vec.push_back((start+kl.pt)/2);vec.push_back(kl.pt);
        vec.push_back((end+kl.pt)/2);vec.push_back(end);
    }
    else if(round(kl.lineLength)>=0.125*minlen)
    {
        vec.push_back(Point2f(kl.startPointX,kl.startPointY));vec.push_back(kl.pt);
        vec.push_back(Point2f(kl.endPointX,kl.endPointY));
    }
}

int main( int argc, char** argv )
{
  // string pathSeq="/home/robot/Datasets/EuRoC/V1_02_medium";
  // string pathSeq="/home/robot/Datasets/TUM/rgbd_dataset_freiburg1_desk2/";
  string pathSeq="/home/robot/Datasets/TUM/rgbd_dataset_freiburg3_walking_rpy/";
  // string pathSeq="/home/robot/Datasets/OpenLORIS/market1-1/";
  string pathCam0 = pathSeq + "/mav0/cam0/data";
  string pathCam1 = pathSeq + "/mav0/cam1/data";
  string pathTimeStamps="/home/robot/ORB_SLAM/ORB-LINE-SLAM/Examples/Stereo-Line/EuRoC_TimeStamps/V102.txt";
  string strSettings="/home/robot/ProgramFiles/orbslam2-with-KLT/Examples/Stereo/EuRoC.yaml";
  vector<string>  vstrImageLeft;
  vector<string>  vstrImageRight;
  vector<double> vTimestampsCam;
  // string strAssociationFilename ="/home/robot/Datasets/OpenLORIS/market1-1/associate.txt";
  string strAssociationFilename ="/home/robot/Datasets/TUM/rgbd_dataset_freiburg3_walking_rpy/associate.txt";

  // LoadImages(pathCam0, pathCam1, pathTimeStamps, vstrImageLeft, vstrImageRight, vTimestampsCam);
  LoadImagesRGB(strAssociationFilename, vstrImageLeft, vstrImageRight, vTimestampsCam);

  /* get parameters from comand line */
  /* create a pointer to a BinaryDescriptor object with deafult parameters */
  Ptr<BinaryDescriptor> bd = BinaryDescriptor::createBinaryDescriptor(); 
  Ptr<BinaryDescriptorMatcher> bdm = BinaryDescriptorMatcher::createBinaryDescriptorMatcher();
  /* create a structure to store extracted lines */
 
cv::Mat imageMat1 ;//= imread( vstrImageLeft[0], 1 );
 cv::Mat imageMat2;
 cv::Mat mask1 = Mat::ones( imageMat1.size(), CV_8UC1 );
  cv::Mat mask2 = Mat::ones( imageMat2.size(), CV_8UC1 );
  // std::vector<KeyLine> keylines1, keylines2;
  // cv::Mat descr1, descr2;
  // bd->detect( imageMat1, keylines1, mask1 );
  // bd->compute( imageMat1, keylines1, descr1 );

for(int i=1,e=vstrImageLeft.size(); i<e; i++)
{
  std::vector<KeyLine> keylines1, keylines2;
  cv::Mat descr1, descr2;

   imageMat1 = imread(pathSeq+vstrImageLeft[i-1], 1 );
   imageMat2 = imread( pathSeq+vstrImageLeft[i], 1 );
  if( imageMat1.data == NULL || imageMat2.data == NULL )
  {
    std::cout << "Error, images could not be loaded. Please, check their paths" << std::endl;
  }
  std::chrono::steady_clock::time_point t1=std::chrono::steady_clock::now();
  bd->detect( imageMat1, keylines1);
  bd->detect( imageMat2, keylines2);

  std::chrono::steady_clock::time_point t2=std::chrono::steady_clock::now();
  bd->compute( imageMat1, keylines1, descr1 );
  bd->compute( imageMat2, keylines2, descr2 );
  std::chrono::steady_clock::time_point t3=std::chrono::steady_clock::now();

  std::vector<KeyLine> lbd_octave1, lbd_octave2;
  Mat left_lbd, right_lbd;
  for ( int i = 0; i < (int) keylines1.size(); i++ )
  {
    if( keylines1[i].octave == 0 && keylines1[i].lineLength>20)
    {
      lbd_octave1.push_back( keylines1[i] );
      left_lbd.push_back( descr1.row( i ) );
    }
  }

  for ( int j = 0; j < (int) keylines2.size(); j++ )
  {
    if( keylines2[j].octave == 0 && keylines1[j].lineLength>20)
    {
      lbd_octave2.push_back( keylines2[j] );
      right_lbd.push_back( descr2.row(j) );
    }
  }
cout<<"des size: "<<right_lbd.size()<<endl;
  /* create a BinaryDescriptorMatcher object */
  /* require match */
  std::vector<DMatch> matches;
  std::vector<std::vector<DMatch>> matchesknn;
  // bdm->match( left_lbd, right_lbd, matches );
  bdm->knnMatch( left_lbd, right_lbd, matchesknn ,2);
  for(size_t i=0,e=lbd_octave1.size();i<e;i++)
  {
    matches.push_back(matchesknn[i][0]);
  } 

  // bdm->match( left_lbd, right_lbd, matches );
cout<<left_lbd.size()<<right_lbd.size()<<matches.size()<<endl;
std::chrono::steady_clock::time_point t4=std::chrono::steady_clock::now();
double detect=std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();
double compute=std::chrono::duration_cast<std::chrono::duration<double> >(t3-t2).count();
double match=std::chrono::duration_cast<std::chrono::duration<double> >(t4-t3).count();
double all=std::chrono::duration_cast<std::chrono::duration<double> >(t4 - t1).count();

cout<<"detect "<<detect<<" compute "<<compute<<" match time: "<<match<<" all "<<all<<endl; 
cv::Mat outImg;
// cv::Mat line;
std::vector<char> mask( matches.size(), 1 );
// drawKeylines(imageMat1,keylines1,line,Scalar::all( -1 ),DrawLinesMatchesFlags::DEFAULT);
drawLineMatches(imageMat1,lbd_octave1, imageMat2, lbd_octave2, matches,outImg,Scalar::all(-1),Scalar::all(-1),mask,DrawLinesMatchesFlags::DEFAULT );
// keylines1.clear();
// keylines2.clear();
  imshow( "Mes", outImg );
  // imshow( "Matches", line );
  waitKey(0);







}



 
}



   