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
#include <iomanip>


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

using namespace cv;
using namespace LineDescriptor;
using namespace std;
float m_inv_K11 = 1.0 / 458.654;
float m_inv_K13 = -367.215 / 458.654;
float m_inv_K22 = 1.0 / 457.296;
float m_inv_K23 = -248.375 / 457.296;
float height,width;


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
    return 0.0 <= img_x && img_x <= width  && 0.0 <= img_y && img_y <= height ;
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
    else if(kl.lineLength>=40)//0.125*minlen
    {
        vec.push_back(Point2f(kl.startPointX,kl.startPointY));vec.push_back(kl.pt);
        vec.push_back(Point2f(kl.endPointX,kl.endPointY));
    }
}


void LoadImagesRGB(const string &strAssociationFilename, vector<string> &vstrImageFilenamesRGB, vector<string> &vstrImageFilenamesD, vector<double> &vTimestamps)
{
    ifstream fAssociation;
    fAssociation.open(strAssociationFilename.c_str());
    // ifstream fAssociation(strAssociationFilename.c_str()); // Open file directly in constructor
    if (!fAssociation.is_open())
    {
        cerr << "Error: Unable to open file " << strAssociationFilename << endl;
        return; // Exit function or handle error accordingly
    }
    // if(fAssociation.)
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
  string pathSeq="/home/robot/Datasets/EuRoC/V1_02_medium";
  string pathCam0 = pathSeq + "/mav0/cam0/data";
  string pathCam1 = pathSeq + "/mav0/cam1/data";
  string pathTimeStamps="/home/robot/ORB_SLAM/ORB-LINE-SLAM/Examples/Stereo-Line/EuRoC_TimeStamps/V102.txt";
  string strSettings="/home/robot/ProgramFiles/orbslam2-with-KLT/Examples/Stereo/EuRoC.yaml";
  vector<string>  vstrImageLeft;
  vector<string>  vstrImageRight;
  vector<double> vTimestampsCam;
//   LoadImages(pathCam0, pathCam1, pathTimeStamps, vstrImageLeft, vstrImageRight, vTimestampsCam);

    // string path="/home/robot/Datasets/OpenLORIS/cafe1-2/";
    string tum_path="/home/robot/Datasets/TUM/rgbd_dataset_freiburg3_walking_xyz/";
    vector<double> vTimestamps;
    // string strSequence="/home/robot/Datasets/OpenLORIS/cafe1-2/";
    //  string strAssociationFilename = "/home/robot/Datasets/OpenLORIS/cafe1-2/color.txt";
        string strSequence="/home/robot/Datasets/OpenLORIS/cafe1-2/";
     string strAssociationFilename ="/home/robot/Datasets/TUM/rgbd_dataset_freiburg3_walking_xyz/associate.txt";
      LoadImagesRGB(strAssociationFilename, vstrImageLeft, vstrImageRight, vTimestamps);
    //   vector<string> vstrImageFilenamesRGB;
    // vector<string> vstrImageFilenamesD;
    // vector<double> vTimestamps;
    // string strSequence="/home/robot/Datasets/TUM/rgbd_dataset_freiburg3_walking_xyz";
    // string strAssociationFilename = "/home/robot/ORB_SLAM/ORB3_EDLines_KL_Yolov8seg/Examples/RGB-D/associations/fr3_walking_xyz.txt";
    // LoadImagesRGB(strAssociationFilename, vstrImageFilenamesRGB, vstrImageFilenamesD, vTimestamps);
  Ptr<BinaryDescriptorMatcher> bdm = BinaryDescriptorMatcher::createBinaryDescriptorMatcher();


  /* get parameters from comand line */
  /* create a pointer to a BinaryDescriptor object with deafult parameters */
  Ptr<BinaryDescriptor> bd = BinaryDescriptor::createBinaryDescriptor();
  Ptr<LSDDetector> lsd = LSDDetector::createLSDDetector();
  /* create a structure to store extracted lines */
  vector<KeyLine> lsd_lines,ed_lines;
  cv::Mat lastimage,lsdlastimage;
    vector<Point2f> coods1,lsdcoods1;coods1.resize(1000);
    vector<Point2f> coods2,lsdcoods2;coods2.resize(1000);
    vector<Point2f> coodsl,coods,lsdcoods;coods.resize(1000);
    vector<uchar> status,fstatus;
    vector<float> error;
    vector<cv::Point2f> ll, rr;
    vector<int> lineID;
    vector<KeyLine> trackedLine;
  vector<int> trackedlineid;
    vector<KeyLine> tracked;
    vector<KeyLine> kltrackedLine;
    vector<int> kllineID; 
    int LastImagePoint,LKtrackedPoint,FFtrackedPoint;

for(size_t ii=0;ii<vstrImageLeft.size();ii++)
{
    // stringstream ss;
    // ss << setfill('0') << setw(10) << ii;
    // string path="/home/robot/Datasets/KITTI/2011_09_30_drive_0020_sync/image_00/data/" + ss.str() + ".png";
    cout<<fixed;
    cout<<"time======"<<vTimestamps[ii]<<"================================"<<endl;
    string path1=tum_path+vstrImageLeft[ii];
    cv::Mat imageMat = imread(path1,cv::IMREAD_UNCHANGED);
    height=imageMat.rows;
    width=imageMat.cols;


//   cv::Mat imageMat = imread(vstrImageLeft[ii],cv::IMREAD_UNCHANGED);
  if( imageMat.data == NULL )
  {
    std::cout << "Error, image could not be loaded. Please, check its path" << path1<<std::endl;
    return -1;
  }
    cv::Mat lsd_output= imageMat.clone();
    cv::Mat ed_output = imageMat.clone();
    int minlenth=min(imageMat.rows,imageMat.cols);

  if(ii==0) //检测
  {
    ed_lines.clear();
    coods1.clear();
    trackedLine.clear();
    lineID.clear();
    std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
    bd->detect(imageMat, ed_lines);
    // lsd->detect( imageMat, ed_lines, 2, 1, mask );
    lastimage=imageMat.clone();
    int numpt=0;
    for(size_t i = 0; i < ed_lines.size(); i++ )
    {
      KeyLine kl = ed_lines[i];
      if( kl.octave == 0 && kl.lineLength>40)
      {
        vector<Point2f> vec;
        reconstructline(kl,vec,minlenth);
        if(vec.size()!=0)
        {
            coods1.insert(coods1.end(),vec.begin(),vec.end()); numpt+=vec.size();
            lineID.push_back(numpt);
            trackedLine.push_back(kl);
        }
      }
    }
    cout<<"the detected line: "<<lineID.size()<<" the point size: "<<coods1.size()<<" image size: "<<height<<" "<<width<<endl;
    std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
    double t=std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();
    cout<<"the detect time: "<<t<<endl;
    continue;
  }
  else  //先光流跟踪，后提取
  { 
    std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
    cv::calcOpticalFlowPyrLK(lastimage, imageMat, coods1, coods2, status, error,Size(10,10),3,cv::TermCriteria((TermCriteria::COUNT) + (TermCriteria::EPS), 20, (0.01)),0,1.0E-4);
    ll.clear();rr.clear();  //在这里剔除超过边界点
    vector<int> index;
    for(size_t i=0,endi=status.size(); i<endi; i++)
    {
        if(status[i]==1  && inBorder(coods2[i])) {ll.push_back(coods1[i]);rr.push_back(coods2[i]);index.push_back(i);}
        else status[i]=0;
    }
    cv::findFundamentalMat(ll, rr, cv::FM_RANSAC, 3.0, 0.99, fstatus);
    for(size_t i=0,end=fstatus.size(); i<end; i++)
    {
        if(fstatus[i]==0) status[index[i]]=0;
    }

    ll.clear();rr.clear();  //本次成功跟踪需要显示的
    tracked.clear();
    trackedlineid.clear();
    coods.clear();
    int lineid=0,newlindeid=0; 
    vector<pair<Point2f,Point2f>> vec;//光流跟踪前后单个直线对应的点
    double min=1000.0,max=0.0,dd=0;
    for(size_t i=0,endi=status.size();i<endi;i++)
    {
        if(i<lineID[lineid] && i!=endi-1) 
        {
            if(status[i]==1) {vec.push_back(make_pair(coods1[i],coods2[i]));}
        }
        else
        {
            if(i==endi-1 && status[i]==1) vec.push_back(make_pair(coods1[i],coods2[i]));
            KeyLine kl=trackedLine[lineid];
            size_t n=vec.size();
//===============================culling the long distance=====================
            // vector<double> dist;
            // // for(size_t i=0;i<n;i++)cout<<"the dist[i] "<<dist[i]<<endl;
            // int min=1000.0;
            // for(size_t i=0;i<n;i++) 
            // {
            //     dist.push_back(sqrt(pow(vec[i].first.x-vec[i].second.x,2)+pow(vec[i].first.y-vec[i].second.y,2)));
            //     if(dist[i]<min) {min=dist[i];}
            //     // cout<<"the dist[i]: "<<dist[i]<<endl;
            // }
            // vector<std::pair<cv::Point2f, cv::Point2f>> vec0;
            // for(size_t i=0;i<n;i++) if(dist[i]-min<100) {vec0.push_back(vec[i]);}//vec[i].first.x=-1; 
            // if(n!=vec0.size())
            // {
            //     cout<<"before dist culling point: "<<n<<" after dist culling: "<<vec0.size()<<endl;
            //     n=vec0.size();
            //     vec=vec0;
            // }
//=============================================================================
            if(n>1)
            {
                kl.startPointX=vec[0].second.x;kl.startPointY=vec[0].second.y;
                kl.endPointX=vec[n-1].second.x;kl.endPointY=vec[n-1].second.y;
                kl.pt=(vec[0].second+vec[n-1].second)/2;kl.lineLength=(float)sqrt(pow(kl.startPointX-kl.endPointX,2)+pow(kl.startPointY-kl.endPointY,2));
                vector<Point2f> point;
                reconstructline(kl,point,minlenth);
                if(!point.empty())
                {
                    for(int i=0;i<n;i++)
                    {
                        ll.push_back(vec[i].first);rr.push_back(vec[i].second);
                    }
                    newlindeid+=point.size();
                    coods.insert(coods.end(),point.begin(),point.end()); //下一次光流跟踪需要的直线点
                    trackedlineid.push_back(newlindeid);  //下一次光流跟踪需要的线ID
                    tracked.push_back(kl);
                }
            }
            vec.clear(); //加入下一条直线第一个点
            if(status[i]==1) {vec.push_back(make_pair(coods1[i],coods2[i]));}
            lineid++;
        }
    }
    // cout<<"===================the coods size: "<<coods.size()<<"the trackend line ie:"<<trackedlineid[trackedlineid.size()-1]<<endl;
    // cout<<"============================the RANSAC tracked  line: ========================"<<tracked.size()<<endl;
    lsd_output=lastimage.clone();
    // cout<<"original point "<<kf<<" last iamge point: "<<LastImagePoint<<" LK tracked: "<<LKtrackedPoint<<" FF tracked: "<<FFtrackedPoint<<" the image id: "<<ii<<"======="<<endl;
    cout<<"the tracked line: "<<tracked.size()<<" the image id: "<<ii;
    std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
    double t=std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1).count();
    cout<<"the KL tracked time: "<<t<<endl;
  }  


  /* draw lines extracted from octave 0 */
  if( lsd_output.channels() == 1 ) cvtColor( lsd_output, lsd_output, COLOR_GRAY2BGR );
  if( ed_output.channels() == 1 ) cvtColor( ed_output, ed_output, COLOR_GRAY2BGR );
//   cv::line(ed_output, Point2f(0,0),Point2f(0,(int)imageMat.rows*0.1),  Scalar(0,0,255), 2);
// cv::line(ed_output, Point2f(imageMat.cols,0),Point2f(imageMat.cols,3),  Scalar(0,0,255), 2);

    for(size_t i = 0; i < trackedLine.size(); i++) //上一帧提取
    {
        KeyLine kl=trackedLine[i];
        cv::line(lsd_output, Point2f(kl.startPointX,kl.startPointY),Point2f(kl.endPointX,kl.endPointY),  Scalar(0,0,255), 2);
    }

    for(size_t i=0;i<tracked.size();i++) //这一帧跟踪
    {
        KeyLine kl=tracked[i];
        cv::line(ed_output, Point2f(kl.startPointX,kl.startPointY),Point2f(kl.endPointX,kl.endPointY),  Scalar(0,0,255), 2);
    }
    for(size_t i = 0; i < rr.size(); i++ )
    {
        cv::line(ed_output, ll[i], rr[i],Scalar(0,255,0), 1);
        cv::circle(ed_output,rr[i],2,(0,255,0),-1);
    } 
    //提取这一帧
    ed_lines.clear();
    coods1.clear();
    trackedLine.clear();
    lineID.clear();
    std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
    bd->detect(imageMat, ed_lines);
    // lsd->detect( imageMat, ed_lines, 2, 1, mask );
    lastimage=imageMat.clone();
    int numpt=0;
    for(size_t i = 0; i < ed_lines.size(); i++ )
    {
      KeyLine kl = ed_lines[i];
      if( kl.octave == 0 && kl.lineLength>40)
      {
        vector<Point2f> vec;
        reconstructline(kl,vec,minlenth);
        if(vec.size()!=0)
        {
            coods1.insert(coods1.end(),vec.begin(),vec.end()); numpt+=vec.size();
            lineID.push_back(numpt);
            trackedLine.push_back(kl);
        }
      }
    }
    cout<<"the detected line: "<<lineID.size()<<" the point size: "<<coods1.size()<<" image size: "<<height<<" "<<width<<endl;
    std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
    double t=std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();
    cout<<"the detect time: "<<t<<endl;

  cv::Mat output;
  hconcat(lsd_output,ed_output,output);
  imshow("opencviter and editer",output);
  waitKey(0);
}
 
}



   