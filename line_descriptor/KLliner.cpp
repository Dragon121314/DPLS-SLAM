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

using namespace cv;
using namespace LineDescriptor;
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

class Rectify
{
public:
    Rectify(const string& config);
    void doRectifyL(const cv::Mat& img_src, cv::Mat& img_rec);
    void doRectifyR(const cv::Mat& img_src, cv::Mat& img_rec);
    cv::Mat M1l,M2l,M1r,M2r;
};
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
    return 0.0 <= img_x && img_x < 752.0  && 0.0 <= img_y && img_y < 480.0 ;
}
void distcheck(vector<double> &dist,vector<pair<Point2f,Point2f>> &vec)
{
    int n=dist.size();
    int min=numeric_limits<int>::max();
    for(size_t i=0;i<n;i++)if(dist[i]!=-1&&dist[i]<min)min=dist[i];
    for(size_t i=0;i<dist.size();i++)if(dist[i]>3*min)dist[i]==-1;

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
  LoadImages(pathCam0, pathCam1, pathTimeStamps, vstrImageLeft, vstrImageRight, vTimestampsCam);
  Rectify rectifyer(strSettings);
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
    vector<uchar> status,lsdstatus;
    vector<float> error;
    bool NeedNew=false;
    bool New=false;
    float kf;
    int numkf=0;
    vector<cv::Point2f> ll, rr;
    vector<int> lineID;
    vector<KeyLine> trackedLine,lastLine;
  vector<int> trackedlineid;
    vector<KeyLine> tracked;
    vector<KeyLine> kltrackedLine;
    vector<int> kllineID; 
    int LastImagePoint,LKtrackedPoint,FFtrackedPoint;

for(size_t ii=0;ii<vstrImageLeft.size();ii++)
{
  cv::Mat imageMat = imread(vstrImageLeft[ii],cv::IMREAD_UNCHANGED);
  if( imageMat.data == NULL )
  {
    std::cout << "Error, image could not be loaded. Please, check its path" << std::endl;
    return -1;
  }
    cv::Mat lsd_output=imageMat.clone();
    cv::Mat ed_output = imageMat.clone();


    numkf++;
    NeedNew=false;
    New=true;
    ed_lines.clear();
    coods1.clear();
    trackedLine.clear();
    lineID.clear();
    cv::Mat mask = Mat::ones( imageMat.size(), CV_8UC1 );
    std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
    bd->detect( imageMat, ed_lines );

    int numpt=0;
    for(size_t i = 0; i < ed_lines.size(); i++ )
    {
      KeyLine kl = ed_lines[i];
      if( kl.octave == 0 && kl.lineLength>0)
      {
        vector<Point2f> vec;
        int minlenth=min(imageMat.rows,imageMat.cols);
        if(round(kl.lineLength)>=0.5*minlenth)
        {
            Point2f start=Point2f(kl.startPointX,kl.startPointY),end=Point2f(kl.endPointX,kl.endPointY);
            Point2f md1=Point2f((start+kl.pt)/2),md2=Point2f((kl.pt+end)/2);
            vec.push_back(start);vec.push_back((start+md1)/2);vec.push_back(md1);vec.push_back((kl.pt+md1)/2);vec.push_back(kl.pt);
            vec.push_back((kl.pt+md2)/2);vec.push_back(md2);vec.push_back((end+md2)/2);vec.push_back(end);
        }
        else if(round(kl.lineLength)>=0.25*minlenth)
        {
            Point2f start=Point2f(kl.startPointX,kl.startPointY),end=Point2f(kl.endPointX,kl.endPointY);
            vec.push_back(start);vec.push_back((start+kl.pt)/2);vec.push_back(kl.pt);
            vec.push_back((end+kl.pt)/2);vec.push_back(end);
        }
        else if(round(kl.lineLength)>=0.125*minlenth)
        {
            vec.push_back(Point2f(kl.startPointX,kl.startPointY));vec.push_back(kl.pt);
            vec.push_back(Point2f(kl.endPointX,kl.endPointY));
        }
        // cout<<"======================: "<<vec.size()<<endl;
        if(vec.size()!=0)
        {
            coods1.insert(coods1.end(),vec.begin(),vec.end()); numpt+=vec.size();
            cout<<"the vec size================: "<<vec.size()<<" and the numpt "<<numpt<<endl;
            lineID.push_back(numpt);
            trackedLine.push_back(kl);
        }
      }
    }
    
    
    if(ii==0){coods=coods1;lastimage=imageMat.clone();kllineID=lineID;kltrackedLine=trackedLine;continue;}

    LastImagePoint=coods.size();
    cv::calcOpticalFlowPyrLK(lastimage, imageMat, coods, coods2, status, error,Size(10,10),3,cv::TermCriteria((TermCriteria::COUNT) + (TermCriteria::EPS), 20, (0.01)),0,1.0E-4);
    // for(size_t i=0;i<error.size();i++)cout<<"error: "<<error[i]<<endl;
    lastimage=imageMat.clone();
    ll.clear();rr.clear();//在这里剔除超过边界点和距离过大点
    tracked.clear();
    trackedlineid.clear();
    vector<pair<Point2f,Point2f>> vec;//光流跟踪前后单个直线对应的点
    vector<double> dist;
    int lineid=0;
    int index=0;
    // cout<<"the lineID.front"<<lineID[lineID.size()-1]<<" the status.size() "<<status.size()<<endl;
    for(size_t i=0,endi=status.size();i<endi;i++)
    {
        if(i<kllineID[lineid] && i!=endi-1) 
        {
            if(status[i]==1  && inBorder(coods2[i])) {vec.push_back(make_pair(coods[i],coods2[i]));dist.push_back(abs(coods2[i].x-coods[i].x)+abs(coods2[i].y-coods[i].y));}
            else{vec.push_back(make_pair(Point2f(),Point2f()));dist.push_back(-1);}
        }
        else
        {
            if(i==endi-1)        

            {
                if(status[i]==1 && inBorder(coods2[i])){vec.push_back(make_pair(coods[i],coods2[i]));dist.push_back(abs(coods2[i].x-coods[i].x)+abs(coods2[i].y-coods[i].y));}
                else{vec.push_back(make_pair(Point2f(),Point2f()));dist.push_back(-1);}
            }
            KeyLine kl=kltrackedLine[lineid];
            size_t n=vec.size();
            Point2f fstart,fend,start,end;
            int flength=0,length=0,endindex;
            double min=1000.0;
            // for(size_t i=0;i<n;i++)cout<<"the dist[i] "<<dist[i]<<endl;
            for(size_t i=0;i<n;i++) if(dist[i]!=-1 && dist[i]<min) {min=dist[i];}
            for(size_t i=0;i<n;i++) if(dist[i]>5*(min+1)) {dist[i]=-1;}
            // for(size_t i=0;i<n;i++)cout<<"the dist[i]=== "<<dist[i]<<endl;
            int index0=0;
            for(int i=0;i<n;i++)
            {
                if(dist[i]!=-1)
                {
                    if(length){end=vec[i].second;length++;}
                    else{start=vec[i].second;end=vec[i].second;length++;}
                }
                else
                {
                    if(flength==0 && length==0){index0=i+1;}
                    else if(length>flength){fstart=start;fend=end;flength=length;endindex=i-1;length=0;}
                    else length=0;
                }
            }
            if(length>1)
            {
                if(flength>0)
                {
                    kl.startPointX=fstart.x;kl.startPointY=fstart.y;
                    kl.endPointX=fend.x;kl.endPointY=fend.y;
                    kl.pt=(fstart+fend)/2;
                    tracked.push_back(kl);
                    index+=flength;
                    // cout<<" tracked point : "<<flength<<" index: "<<index<<endl;
                    trackedlineid.push_back(index);
                    for(int i=0;i<flength;i++,endindex--)
                    {
                        ll.push_back(vec[endindex].first);rr.push_back(vec[endindex].second);
                    }
                }
                else //flength==0 直线所有点都跟踪到了的；或者前几个点是跟踪失败，后面几个点都跟踪成功
                {
                    if(index0)
                    {
                        kl.startPointX=vec[index0].second.x;kl.startPointY=vec[index0].second.y;
                        kl.endPointX=vec.back().second.x;kl.endPointY=vec.back().second.y;
                        kl.pt=(vec[index0].second+vec.back().second)/2;
                        tracked.push_back(kl);
                        index+=(n-index0);
                        // cout<<" tracked point : "<<(vec.size()-index0)<<" index: "<<index<<endl;
                        // int aa=n-index0;
                        // cout<<"======================the n-index0: "<<aa<<"the length: ====================================="<<length<<endl;
                        trackedlineid.push_back(index);
                        for(int i=index0;i<n;i++)
                        {
                            ll.push_back(vec[i].first);rr.push_back(vec[i].second);
                        }
                    }
                    else
                    {

                        for(int en=vec.size(),i=0;i<en;i++)
                        {
                            ll.push_back(vec[i].first);rr.push_back(vec[i].second);
                        }
                        kl.startPointX=vec[0].second.x;kl.startPointY=vec[0].second.y;
                        kl.endPointX=vec.back().second.x;kl.endPointY=vec.back().second.y;
                        kl.pt=(vec[0].second+vec.back().second)/2;
                        tracked.push_back(kl);
                        index+=vec.size();
                        // cout<<" all point tracked: "<<vec.size()<<" index: "<<index<<endl;
                        trackedlineid.push_back(index);
                    }
                }
                // cout<<"the LK ================== "<<kl.startPointX<<" "<<kl.startPointY<<" "<<kl.endPointX<<" "<<kl.endPointY<<endl;

            }
            // cout<<" single line point: "<<vec.size()<<" index: "<<index<<endl;
            vec.clear();dist.clear();//加入下一条直线第一个点
            if(status[i]==1 && inBorder(coods2[i])) {vec.push_back(make_pair(coods[i],coods2[i]));dist.push_back(abs(coods2[i].x-coods[i].x)+abs(coods2[i].y-coods[i].y));}
            else{vec.push_back(make_pair(Point2f(),Point2f()));dist.push_back(-1);}
            lineid++;
        }
        // cout<<"the line number: "<<lineID.size()<<" the first line pt: "<<lineID[lineid]<<endl;
    }
    LKtrackedPoint=ll.size();
   kltrackedLine=tracked;
   kllineID=trackedlineid;  
    tracked.clear();
    trackedlineid.clear();
 
    cv::findFundamentalMat(ll, rr, cv::FM_RANSAC, 3.0, 0.99, status);

    coodsl.clear();
    coods.clear();

    vec.clear();//RANSAC前后对应的点
    dist.clear();
    lineid=0;
    index=0;
    // cout<<"the lineID.front"<<lineID[lineID.size()-1]<<" the status.size() "<<status.size()<<endl;
    for(size_t i=0,endi=status.size();i<endi;i++)
    {
        if(i<kllineID[lineid] && i!=endi-1) 
        {
            if(status[i]==1) {vec.push_back(make_pair(ll[i],rr[i]));dist.push_back(1);}
            else{vec.push_back(make_pair(Point2f(),Point2f()));dist.push_back(-1);}
            // cout<<"the kl line number: "<<kllineID.size()<<" the kl line pt: "<<kllineID[lineid]<<endl;
        }
        else
        {
            if(i==endi-1)
            {
                if(status[i]==1){vec.push_back(make_pair(ll[i],rr[i]));dist.push_back(1);}
                else{vec.push_back(make_pair(Point2f(),Point2f()));dist.push_back(-1);}
            }
            KeyLine kl=kltrackedLine[lineid];
            size_t n=vec.size();
            Point2f fstart,fend,start,end;
            int flength=0,length=0,endindex;
            int index0=0;
            for(int i=0;i<n;i++)
            {
                if(dist[i]!=-1)
                {
                    if(length){end=vec[i].second;length++;}
                    else{start=vec[i].second;end=vec[i].second;length++;}
                }
                else
                {
                    if(flength==0 && length==0){index0=i+1;}
                    else if(length>flength){fstart=start;fend=end;flength=length;endindex=i-1;length=0;}
                    else length=0;
                }
            }
            if(length>1)
            {
                if(flength>0)
                {
                    kl.startPointX=fstart.x;kl.startPointY=fstart.y;
                    kl.endPointX=fend.x;kl.endPointY=fend.y;
                    kl.pt=(fstart+fend)/2;
                    tracked.push_back(kl);
                    index+=flength;
                    // cout<<" tracked point : "<<flength<<" index: "<<index<<endl;
                    trackedlineid.push_back(index);
                    for(int i=0;i<flength;i++,endindex--)
                    {
                        coods.push_back(vec[endindex].second);
                        coodsl.push_back(vec[endindex].first);
                    }
                }
                else //flength==0 直线所有点都跟踪到了的；或者前几个点是跟踪失败，后面几个点都跟踪成功
                {
                    if(index0)
                    {
                        kl.startPointX=vec[index0].second.x;kl.startPointY=vec[index0].second.y;
                        kl.endPointX=vec[vec.size()-1].second.x;kl.endPointY=vec[vec.size()-1].second.y;
                        kl.pt=(vec[index0].second+vec.back().second)/2;
                        tracked.push_back(kl);
                        index+=(n-index0);
                        // cout<<" tracked point : "<<(vec.size()-index0)<<" index: "<<index<<endl;
                        trackedlineid.push_back(index);
                        for(int i=index0;i<n;i++)
                        {
                            coods.push_back(vec[i].second);
                            coodsl.push_back(vec[i].first);
                        }
                    }
                    else
                    {
                        for(int en=vec.size(),i=0;i<en;i++)
                        {
                            coods.push_back(vec[i].second);
                            coodsl.push_back(vec[i].first);
                        }
                        kl.startPointX=vec[0].second.x;kl.startPointY=vec[0].second.y;
                        kl.endPointX=vec.back().second.x;kl.endPointY=vec.back().second.y;
                        kl.pt=(vec[0].second+vec.back().second)/2;
                        tracked.push_back(kl);
                        index+=vec.size();
                        // cout<<" all point tracked: "<<vec.size()<<" index: "<<index<<endl;
                        trackedlineid.push_back(index);
                    }
                }
            }
            // cout<<" single line point: "<<vec.size()<<" index: "<<index<<endl;
            vec.clear();dist.clear();//加入下一条直线第一个点
            if(status[i]==1) {vec.push_back(make_pair(ll[i],rr[i]));dist.push_back(1);}
            else{vec.push_back(make_pair(Point2f(),Point2f()));dist.push_back(-1);}
            // cout<<"the kl line number: "<<kllineID.size()<<" the kl line pt: "<<kllineID[lineid]<<endl;
            lineid++;
        }
    }
    // cout<<"===================the coods size: "<<coods.size()<<"the trackend line ie:"<<trackedlineid[trackedlineid.size()-1]<<endl;
    // cout<<"============================the RANSAC tracked  line: ========================"<<tracked.size()<<endl;
    FFtrackedPoint=coods.size();
    if(coods.size()/kf<0.2) NeedNew=true;
    cout<<"original point "<<kf<<" last iamge point: "<<LastImagePoint<<" LK tracked: "<<LKtrackedPoint<<" FF tracked: "<<FFtrackedPoint<<" the image id: "<<ii<<"======="<<endl;

  /* draw lines extracted from octave 0 */
  if( lsd_output.channels() == 1 ) cvtColor( lsd_output, lsd_output, COLOR_GRAY2BGR );
  if( ed_output.channels() == 1 ) cvtColor( ed_output, ed_output, COLOR_GRAY2BGR );
  // printMatType(ed_output);
  // printMatType(lsd_output);
//    cv::line(ed_output, Point2f(0,0),Point2f(0,(int)imageMat.rows*0.1),  Scalar(0,0,255), 2);


    for(size_t i = 0; i < trackedLine.size(); i++)
    {
        KeyLine kl=trackedLine[i];
        cv::line(lsd_output, Point2f(kl.startPointX,kl.startPointY),Point2f(kl.endPointX,kl.endPointY),  Scalar(0,0,255), 2);
    }

    for(size_t i=0;i<tracked.size();i++)
    {
        KeyLine kl=tracked[i];
        cv::line(ed_output, Point2f(kl.startPointX,kl.startPointY),Point2f(kl.endPointX,kl.endPointY),  Scalar(0,0,255), 2);
    }
    for(size_t i = 0; i < coods.size(); i++ )
    {
        cv::line(ed_output, coodsl[i], coods[i],Scalar(0,255,0), 1);
        cv::circle(ed_output,coods[i],2,(0,255,0),-1);
    }
    coods=coods1;kllineID=lineID;kltrackedLine=trackedLine;

//   vector<float> lsd_len(lsd_lines.size());

//   float length_th=40;
//   int lsd_num_th=0;
//   int ed_num_th=0;
//   for(size_t i = 0; i < lsd_lines.size(); i++ )
//   {
//     KeyLine kl = lsd_lines[i];
//     lsd_len[i]=kl.lineLength;
//     if( kl.octave == 0&&kl.lineLength>length_th)
//     {
//       /* get a random color */
//       int R = ( rand() % (int) ( 255 + 1 ) );
//       int G = ( rand() % (int) ( 255 + 1 ) );
//       int B = ( rand() % (int) ( 255 + 1 ) );
//       /* get extremes of line */
//       Point pt1 = Point2f( kl.startPointX, kl.startPointY );
//       Point pt2 = Point2f( kl.endPointX, kl.endPointY );
//       /* draw line */
//       line( lsd_output, pt1, pt2, Scalar( B, G, R ), 2 );
//       lsd_num_th++;
//     }
//   }
//   vector<float> ed_len(ed_lines.size());
//   for ( size_t i = 0; i < ed_lines.size(); i++ )
//   {
//     KeyLine kl = ed_lines[i];
//     ed_len[i]=kl.lineLength;
//     if( kl.octave == 0&&kl.lineLength>length_th)
//     {
//       /* get a random color */
//       int R = ( rand() % (int) ( 255 + 1 ) );
//       int G = ( rand() % (int) ( 255 + 1 ) );
//       int B = ( rand() % (int) ( 255 + 1 ) );
//       /* get extremes of line */
//       Point pt1 = Point2f( kl.startPointX, kl.startPointY );
//       Point pt2 = Point2f( kl.endPointX, kl.endPointY );
//       /* draw line */
//       line( ed_output, pt1, pt2, Scalar( B, G, R ), 2 );
//       ed_num_th++;
//     }
//   }
//   cout<<"original lsd lines: "<<lsd_lines.size()<<" original ed lines: "<<ed_lines.size();
//   cout<<" culling lsd lines: "<<lsd_num_th<<" culling ed lines: "<<ed_num_th<<" the lsd time: "<<lsd_time<<" the ed time: "<<ed_time<<endl;





//   Mat gray=imageMat.clone();
//     // cvtColor(imageMat,gray,COLOR_BGR2GRAY);
//     //设置提取直方图的相关变量
//     Mat hist;//用于存放直方图计算结果
//     const int channels[1]={0};//通道索引.对于彩色图像，可以是 {0, 1, 2}，分别对应于蓝色、绿色和红色通道。
//     float inRanges[2]={0,255};
//     const float*ranges[1]={inRanges};//像素灰度值范围
//     const int bins[1]={256};//直方图的维度，其实就是像素灰度值的最大值
    //1：这是图像通道的数量。1 表示计算单通道图像的直方图。对于灰度图像，这个值通常是 1。对于彩色图像，如果分别计算每个颜色通道的直方图，这个值可以是 3（对应于 BGR 通道）。
    //mat()掩码用于指定图像中哪些区域应该被包含在直方图计算中
    //1：这是直方图的维度。在这里，1 表示直方图是一维的。
    //bins：这是直方图的桶（bins）数量。桶的数量决定了直方图的分辨率。对于8位图像，bins 通常是 256，因为 8 位图像有 256 个可能的像素值。
    //ranges：这是一个包含两个元素的数组，定义了直方图计算的值的范围。对于8位图像，这个数组通常是{0,256}，表示考虑所有可能的像素值。
    // calcHist(&gray,1,channels,Mat(),hist,1,bins,ranges);//计算图像直方图

    // cv::normalize(hist, hist, 0, 1, cv::NORM_L1);
//================计算梯度直方图===========================================
//   cv::Mat grad_x, grad_y;
//   cv::Mat abs_grad_x, abs_grad_y, grad;
//   cv::Sobel(imageMat, grad_x, CV_16S, 1, 0);
//   cv::Sobel(imageMat, grad_y, CV_16S, 0, 1);

//   // 计算绝对值
//   cv::convertScaleAbs(grad_x, abs_grad_x);
//   cv::convertScaleAbs(grad_y, abs_grad_y);
//   imshow("x",abs_grad_x);
//   imshow("y",abs_grad_y);
//   // 合并梯度
//   abs_grad_x += abs_grad_y;
//   grad = abs_grad_x;

//   imshow("tidu",grad);
//   cv::Mat hist1;
//   calcHist(&grad, 1, 0, cv::Mat(), hist1, 1,bins,ranges, true, false);
//========================================================================

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
  // resize(lsd_output,lsd_output,Size(512,400));
  // resize(ed_output,ed_output,Size(512,400));

  // cvtColor(histImage1, histImage1, COLOR_BGR2GRAY );
  // cout<<"the image size "<<output.size().width<<" "<<output.size().height<<" "<<output.channels()<<endl;
  // cout<<"the image size "<<histImage1.size().width<<" "<<histImage1.size().height<<" "<<histImage1.channels()<<endl;
//   hconcat(histImage1,histImage2,toShow);
//   imshow("hist and dx",toShow);
  cv::Mat output;
  hconcat(lsd_output,ed_output,output);
  imshow("opencviter and editer",output);
  waitKey(0);
}
 
 cout<<"the images: "<<vstrImageLeft.size()<<" the kf num: "<<numkf<<endl;
}



Rectify::Rectify(const string& config)
{
    // Load settings related to stereo calibration
    cv::FileStorage fsSettings(config, cv::FileStorage::READ);
    if(!fsSettings.isOpened())
    {
        cerr << "ERROR: Wrong path to settings" << endl;
        //return -1;
        throw;
    }

    cv::Mat K_l, K_r, P_l, P_r, R_l, R_r, D_l, D_r;
    fsSettings["LEFT.K"] >> K_l;
    fsSettings["RIGHT.K"] >> K_r;

    fsSettings["LEFT.P"] >> P_l;
    fsSettings["RIGHT.P"] >> P_r;

    fsSettings["LEFT.R"] >> R_l;
    fsSettings["RIGHT.R"] >> R_r;

    fsSettings["LEFT.D"] >> D_l;
    fsSettings["RIGHT.D"] >> D_r;

    int rows_l = fsSettings["LEFT.height"];
    int cols_l = fsSettings["LEFT.width"];
    int rows_r = fsSettings["RIGHT.height"];
    int cols_r = fsSettings["RIGHT.width"];

    if(K_l.empty() || K_r.empty() || P_l.empty() || P_r.empty() || R_l.empty() || R_r.empty() || D_l.empty() || D_r.empty() ||
            rows_l==0 || rows_r==0 || cols_l==0 || cols_r==0)
    {
        cerr << "ERROR: Calibration parameters to rectify stereo are missing!" << endl;
        //return -1;
        throw;
    }

    cv::initUndistortRectifyMap(K_l,D_l,R_l,P_l.rowRange(0,3).colRange(0,3),cv::Size(cols_l,rows_l),CV_32F,M1l,M2l);
    cv::initUndistortRectifyMap(K_r,D_r,R_r,P_r.rowRange(0,3).colRange(0,3),cv::Size(cols_r,rows_r),CV_32F,M1r,M2r);
}

void Rectify::doRectifyL(const cv::Mat& img_src, cv::Mat& img_rec)
{
    cv::remap(img_src,img_rec,M1l,M2l,cv::INTER_LINEAR);
}

void Rectify::doRectifyR(const cv::Mat& img_src, cv::Mat& img_rec)
{
    cv::remap(img_src,img_rec,M1r,M2r,cv::INTER_LINEAR);
}