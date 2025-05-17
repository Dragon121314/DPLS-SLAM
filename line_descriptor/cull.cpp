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
    return 0.0 <= img_x && img_x < width  && 0.0 <= img_y && img_y < height ;
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


void LoadImagesRGB(const string &strAssociationFilename, vector<string> &vstrImageFilenamesRGB,
                vector<string> &vstrImageFilenamesD, vector<double> &vTimestamps)
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
    string path="/home/robot/Datasets/TUM/rgbd_dataset_freiburg1_desk2/";
    vector<double> vTimestamps;
    // string strSequence="/home/robot/Datasets/OpenLORIS/cafe1-2/";
    //  string strAssociationFilename = "/home/robot/Datasets/OpenLORIS/cafe1-2/color.txt";
        string strSequence="/home/robot/Datasets/OpenLORIS/cafe1-2/";
     string strAssociationFilename ="/home/robot/ORB_SLAM3_mod/Examples/RGB-D/associations/fr1_desk2.txt";
    //   LoadImagesRGB(strAssociationFilename, vstrImageLeft, vstrImageRight, vTimestamps);
    // vector<double> vTimestamps;
    // string strSequence="/home/robot/Datasets/TUM/rgbd_dataset_freiburg3_walking_xyz";
    LoadImagesRGB(strAssociationFilename, vstrImageLeft, vstrImageRight,vTimestamps);


  /* get parameters from comand line */
  /* create a pointer to a BinaryDescriptor object with deafult parameters */
  Ptr<BinaryDescriptor> bd = BinaryDescriptor::createBinaryDescriptor();
  Ptr<BinaryDescriptorMatcher> bdm = BinaryDescriptorMatcher::createBinaryDescriptorMatcher();
  Ptr<LSDDetector> lsd = LSDDetector::createLSDDetector();
  /* create a structure to store extracted lines */
  vector<KeyLine> lsd_lines,ed_lines ;
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

    for(size_t ii=1;ii<vstrImageLeft.size();ii++)
    { 
        // string path1=vstrImageLeft[ii]; 
        string path1=path+vstrImageLeft[ii]; 
        cv::Mat imageMat = imread(path1,cv::IMREAD_UNCHANGED);
        height=imageMat.rows;
        width=imageMat.cols;
        if( imageMat.data == NULL )
        {
            std::cout << "Error, image could not be loaded. Please, check its path" << std::endl;
            return -1;
        }
        cv::Mat lsd_output= imageMat.clone();
        cv::Mat ed_output = imageMat.clone();
        int minlenth=min(imageMat.rows,imageMat.cols);


        ed_lines.clear();
        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
        bd->detect(imageMat, ed_lines);
        // bd->detect(imageMat0, ed_lines0);
        // lsd->detect( imageMat, ed_lines, 2, 1, mask );
        std::chrono::steady_clock::time_point t2=std::chrono::steady_clock::now();
        int nLine=ed_lines.size();
        vector<bool> b(nLine,true);
        vector<int> c(nLine,-1);
        float ss[nLine][nLine],se[nLine][nLine],es[nLine][nLine],ee[nLine][nLine];
        float ang[nLine];

        for(size_t i = 0; i < nLine; i++ )
        {
            KeyLine kl1=ed_lines[i];
            if(kl1.angle<0)ang[i]=-kl1.angle;
            else ang[i]=kl1.angle;

            for(int j=i+1;j<nLine;j++)
            {
                KeyLine kl2=ed_lines[j];
                if(kl1.lineLength>=kl2.lineLength)
                {
                    // cv::Point2f l(kl1.endPointX-kl1.startPointX,kl1.endPointY-kl1.startPointY);
                    cv::Point2f l=(kl1.pe-kl1.ps);
                    cv::Point2f ssv=(kl2.ps-kl1.ps);
                    cv::Point2f sev=(kl2.pe-kl1.ps);

                    float ss0=(l.dot(ssv))/kl1.lineLength;
                    float se0=(l.dot(sev))/kl1.lineLength; 
                    ss[i][j]=ss[j][i]=ss0;
                    se[i][j]=se[j][i]=se0;
                }
                else
                {
                    cv::Point2f l=(kl2.pe-kl2.ps);

                    cv::Point2f ssv=(kl1.ps-kl2.ps);
                    cv::Point2f sev=(kl1.pe-kl2.ps);

                    float ss0=(l.dot(ssv))/kl2.lineLength;
                    float se0=(l.dot(sev))/kl2.lineLength; 
                    ss[i][j]=ss[j][i]=ss0;
                    se[i][j]=se[j][i]=se0;
                }

            }
        }
        
        float th=8.0;
        float fuseth=2.0;
        float angleth=0.01745*5;  //0.01745=1度 
        float fuseangth=0.01745*2;   
        vector<KeyLine> fu;
    for(size_t i = 0; i < nLine; i++ )
    {
        if(!b[i]) continue;
        KeyLine &kl1 = ed_lines[i]; 
        for(size_t j = i+1; j < nLine; j++ )
        {
            if(!b[j]) continue;
            KeyLine &kl2 = ed_lines[j];
            if(abs(ang[i]-ang[j])<angleth || abs(ang[i]+ang[j]-CV_PI)<angleth)
            {
                float lss=ss[i][j],lse=se[i][j];
// //////////////////////////////
//                 if(abs(ang[i]-ang[j])<fuseangth || abs(ang[i]+ang[j]-CV_PI)<fuseangth) //====计算是否可以融合
//                 {
//                     // cout<<"start fuse ...."<<" "<<ss[i][j]<<" "<<se[i][j]<<" "<<i<<" "<<j<<" "<<kl1.lineLength<<" "<<kl2.lineLength<<" "<<kl1.angle<<" "<<kl2.angle<<endl;
//                     if(kl1.lineLength>=kl2.lineLength)//是否可以融合K2
//                     {
//                         if(lss==0&&lse==0) continue;
//                         cv::Point2f s=kl2.getStartPoint(),e=kl2.getEndPoint();
//                         float d1=kl1.DistancePointoLine(s),d2=kl1.DistancePointoLine(e);
//                         if(d1>th || d2>th) continue;
//                         else if(d1<fuseth&&d2<fuseth)
//                         {
//                             if(lss>=kl1.lineLength || lse>=kl1.lineLength) //开始融合
//                             {
//                                 // if(lss>lse && lse-kl1.lineLength<fuseth) kl1.ps=s; //kl1.setEndPoint(kl2.ps);
//                                 // else if(lse>lss && lss-kl1.lineLength<fuseth) kl1.pe=e; //kl1.setEndPoint(kl2.pe);
                                
//                                 if(lss>lse && lse-kl1.lineLength<fuseth) {fu.push_back(kl2);kl1.setEndPoint(kl2.ps);}
//                                 else if(lse>lss && lss-kl1.lineLength<fuseth) {fu.push_back(kl2);kl1.setEndPoint(kl2.pe);}
//                                 else continue;
//                             }
//                             else if(lss<0 || lse<0)
//                             {
//                                 // if(lss<lse && lse>-fuseth)kl1.ps=s;// kl1.setStartPoint(s);
//                                 // else if(lse<lss && lss>-fuseth) kl1.pe=e; //kl1.setStartPoint(e);
                                
//                                 if(lss<lse && lse>-fuseth) {fu.push_back(kl2);kl1.setStartPoint(s);}
//                                 else if(lse<lss && lss>-fuseth) {fu.push_back(kl2);kl1.setStartPoint(e);}
//                                 else continue;
//                             }

//                             cout<<"end fuse line"<<endl;
//                         }
//                         else if(lss>=kl1.lineLength&&lse>=kl1.lineLength || lss<=0&&lse<=0) continue;
//                         b[j]=false;c[j]=i; 
//                     }  
//                     else  //kl1.lineLength<kl2.lineLength                 //是否可以融合k1
//                     {
//                         if(lss==0&&lse==0) continue;
//                         cv::Point2f s=kl1.getStartPoint(),e=kl1.getEndPoint();
//                         float d1=kl2.DistancePointoLine(s),d2=kl2.DistancePointoLine(e);
//                         if(d1>th || d2>th) continue;                           
//                         else if(d1<fuseth&&d2<fuseth)
//                         {
//                             if(lss>=kl2.lineLength || lse>=kl2.lineLength) //开始融合
//                             {
//                                 // if(lss>lse&& lse-kl2.lineLength<fuseth)kl2.ps=s;//{kl2.setEndPoint(s);}
//                                 // else if(lss<lse && lss-kl2.lineLength<fuseth)kl2.pe=e;// kl2.setEndPoint(e);
                               

//                                 if(lss>lse&& lse-kl2.lineLength<fuseth){kl2.setEndPoint(s);fu.push_back(kl1);}
//                                 else if(lss<lse && lss-kl2.lineLength<fuseth) {fu.push_back(kl1);kl2.setEndPoint(e);}
//                                 else continue; //两直线平行 但是端点相距很远   
//                             }
//                             else if(lss<0 || lse<0)
//                             {
//                                 // if(lss<lse && lse>-fuseth) kl2.ps=s; //kl2.setStartPoint(kl1.ps);
//                                 // else if(lse<lss && lss>-fuseth) kl2.pe=e; //kl2.setStartPoint(kl1.pe); 
//                                 if(lss<lse && lse>-fuseth) {fu.push_back(kl1);kl2.setStartPoint(kl1.ps);}
//                                 else if(lse<lss && lss>-fuseth) {fu.push_back(kl1);kl2.setStartPoint(kl1.pe);}
//                                 else continue;
//                             }
//                             cout<<"end fuse line"<<endl;

//                         }
//                         else if(lss>kl2.lineLength&&lse>kl2.lineLength || lss<=0&&lse<=0) continue;
//                         b[i]=false;c[i]=j; 
//                     } 
//                 }
//                 else     //剔除
                {
                    if(kl1.lineLength>=kl2.lineLength)
                    {
                        if(lss>=kl1.lineLength&&lse>=kl1.lineLength || lss<=0&&lse<=0) continue;//两直线不重合
                        cv::Point2f s=kl2.getStartPoint(),e=kl2.getEndPoint();
                        float d1=kl1.DistancePointoLine(s),d2=kl1.DistancePointoLine(e);
                        if(d1<th&&d2<th){b[j]=false;c[j]=i;}
                        // cout<<"d1, d2:"<<d1<<" "<<d2<<endl;
                    }
                    else
                    {
                        if(lss>kl2.lineLength&&lse>kl2.lineLength || lss<=0&&lse<=0) continue;
                        cv::Point2f s=kl1.getStartPoint(),e=kl1.getEndPoint();
                        float d1=kl2.DistancePointoLine(s),d2=kl2.DistancePointoLine(e);
                        if(d1<th&&d2<th){b[i]=false;c[i]=j;}
                    }
                }
            }
            if(!b[i]) break;
        }
    } 
 
        std::chrono::steady_clock::time_point t3=std::chrono::steady_clock::now();
        double t=std::chrono::duration_cast<std::chrono::duration<double>>(t3-t2).count();
        double y=std::chrono::duration_cast<std::chrono::duration<double>>(t2-t1).count();
        int ncull=0;
        cv::line(ed_output, Point2f(0,0),Point2f(0,th),  Scalar(0,0,255), 2);
        for(int i=0;i<ed_lines.size();i++)
        {
            KeyLine kl=ed_lines[i];
            // cout<<i<<" "<<kl.angle<<endl;
            // cv::putText(lsd_output, to_string(i), Point2f(kl.startPointX,kl.startPointY), cv::FONT_HERSHEY_SIMPLEX,0.5, Scalar(255,0,0), 1);

            if(b[i]) //长线，都画
            {
                cv::line(ed_output, Point2f(kl.startPointX,kl.startPointY),Point2f(kl.endPointX,kl.endPointY),  Scalar(0,0,255), 2);
                cv::line(lsd_output, Point2f(kl.startPointX,kl.startPointY),Point2f(kl.endPointX,kl.endPointY),  Scalar(0,0,255), 2);
                if(find(c.begin(),c.end(),i)!=c.end())cv::putText(ed_output, to_string(i), Point2f(kl.startPointX,kl.startPointY), cv::FONT_HERSHEY_SIMPLEX,0.5, Scalar(255,0,0), 1);
                // cv::putText(ed_output, to_string(i), Point2f(kl.startPointX,kl.startPointY), cv::FONT_HERSHEY_SIMPLEX,0.5, Scalar(255,0,0), 1);
                ncull++;
            }
            else
            {
                cv::line(lsd_output, Point2f(kl.startPointX,kl.startPointY),Point2f(kl.endPointX,kl.endPointY),  Scalar(255,0,255), 2);
                cv::putText(lsd_output, to_string(c[i]), Point2f(kl.startPointX,kl.startPointY), cv::FONT_HERSHEY_SIMPLEX,0.5, Scalar(255,0,0), 1);
                
            }
        } 
        for(int i=0;i<fu.size();i++)
        {
            KeyLine kl=fu[i];
            cv::line(lsd_output, Point2f(kl.startPointX,kl.startPointY),Point2f(kl.endPointX,kl.endPointY),  Scalar(255,255,0), 2);

        }
        cout<<ii<<" detect: "<<y<<" time: "<<t<<" all:"<<ed_lines.size()<<" good:"<<ncull<<endl;

        cv::Mat output;
        hconcat(lsd_output,ed_output,output);
        imshow("opencviter and editer",output);
        waitKey(0);
    }

}
 




   