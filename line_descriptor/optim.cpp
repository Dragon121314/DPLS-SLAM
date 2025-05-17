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
#include <Eigen/Dense>
#include <random>

using namespace cv;
using namespace LineDescriptor;
using namespace std;

typedef Eigen::Matrix<float,6,1> vector6f;

float m_inv_K11 = 1.0 / 458.654;
float m_inv_K13 = -367.215 / 458.654;
float m_inv_K22 = 1.0 / 457.296;
float m_inv_K23 = -248.375 / 457.296;
float height,width;
float cx=318.643, cy=255.314, invfx=0.00193309, invfy=0.00193622;
 class mLineIterator 
    {
    public:
        mLineIterator(const double x1_, const double y1_, const double x2_, const double y2_)
                    : x1(x1_), y1(y1_), x2(x2_), y2(y2_), steep(std::abs(y2_ - y1_) > std::abs(x2_ - x1_)) 
        {
            if(steep) 
            {
                std::swap(x1, y1);
                std::swap(x2, y2);
            }
            if(x1 > x2) 
            {
                std::swap(x1, x2);
                std::swap(y1, y2);
            }
            dx = x2 - x1;
            dy = std::abs(y2 - y1);
            error = dx / 2.0;
            ystep = (y1 < y2) ? 1 : -1;
            x = x1;
            y = y1;
            maxX = x2;
        }

        bool getNext(cv::Point2f &pixel)
        {

            if (x > maxX) return false;
            if (steep) pixel = cv::Point2f(y, x);
            else pixel = cv::Point2f(x, y);
            error -= dy;
            if (error < 0) 
            {
                y += ystep;
                error += dx;
            }
            x++;
            return true;
        }
    private:
        double x1, y1, x2, y2;
        double dx, dy, error;
        float maxX, ystep;
        float y, x;
        const bool steep;
    };


     inline void getLineCoords(const cv::Point2f &p1, const cv::Point2f &p2, std::vector<cv::Point2f> &line_coords) 
    {
        line_coords.clear();
        mLineIterator it(p1.x, p1.y, p2.x, p2.y);
        cv::Point2f p;
        bool step=true;
        while (it.getNext(p)) 
        {
            if(step){line_coords.push_back(p);step=false;}
            else step=true;
        }

        if(abs(line_coords[0].x-p1.x)<3.0 && abs(line_coords[0].y-p1.y)<3.0)
        {
            line_coords[line_coords.size()-1]=p2;
        }
        else
        {
            line_coords[line_coords.size()-1]=p1;
            reverse(line_coords.begin(),line_coords.end());
        }
    }



vector6f fitPlaneRANSAC(const std::vector<cv::Point3f> &points, int maxIterations = 50, double distanceThreshold = 0.01) 
{
    int bestInlierCount = 0, n=points.size()*0.9;
    vector6f bestPlane;
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<> dis(0, points.size() - 1);

    for (int iter = 0; iter < maxIterations; ++iter) 
    {
        // 随机选择三个点
        int idx1 = dis(gen);
        int idx2 = dis(gen);

        if (idx1 == idx2)  continue; // 避免选到相同的点
        cv::Point3f p1 = points[idx1];
        cv::Point3f p2 = points[idx2];
        // 构造两个向量
        Eigen::Vector3f v1(p1.x,p1.y,p1.z);
        Eigen::Vector3f v2(p2.x,p2.y,p2.z);
        // 计算法向量
        Eigen::Vector3f normal = v1-v2;
        // Eigen::Vector3f normal = v1.cross(v2);
        if (normal.norm() == 0) continue; // 避免共线
        normal[0]=1/normal[0];
        normal[1]=1/normal[1];
        normal[2]=1/normal[2];
        normal.normalize();
        // 平面方程：ax + by + cz + d = 0
        // double d = -normal.dot(v3);
        // Eigen::Vector4f plane(normal[0], normal[1], normal[2], d);
        // 计算内点数量
        int inlierCount = 0;
        // for (const auto &p : points) 
        // {
        //     Eigen::Vector3f point(p.x, p.y, p.z);
        //     double distance = std::abs(plane.head<3>().dot(point) + plane[3]);
        //     if (distance < distanceThreshold) inlierCount++;
        // }
        for(int i=0,iend=points.size(); i<iend; i++) 
        {
            // Eigen::Vector3f point(p.x, p.y, p.z);
            float a=(p1.x-points[i].x)*normal[0],b=(p1.y-points[i].y)*normal[1],c=(p1.z-points[i].z)*normal[2];
            double distance =abs(a-b)+abs(a-c)+abs(b-c);
            // double distance =points[i].z-Eigen::Vector3f(plane[0],plane[1],plane[3]).dot(Eigen::Vector3f(points[i].x,points[i].y,1))/plane[2];
            if (distance < distanceThreshold) inlierCount++;
        }
        // 更新最佳平面
        if(inlierCount > bestInlierCount) {
            bestInlierCount = inlierCount;
            bestPlane.head<3>() = normal;
            bestPlane.tail<3>() = v1;
        }
        if(inlierCount>n) break;
    }
    return bestPlane;
}

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

void optimline(vector<KeyLine> &ed_lines, vector<bool> &b)
{
        int nLine=ed_lines.size();
        b.resize(nLine,true);
        // vector<int> c(nLine,-1);
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
                if(abs(ang[i]-ang[j])<fuseangth || abs(ang[i]+ang[j]-CV_PI)<fuseangth) //====计算是否可以融合
                {
                    if(kl1.lineLength>=kl2.lineLength)//是否可以融合K2
                    {
                        if(lss==0&&lse==0) continue;
                        cv::Point2f s=kl2.getStartPoint(),e=kl2.getEndPoint();
                        float d1=kl1.DistancePointoLine(s),d2=kl1.DistancePointoLine(e);
                        if(d1>th || d2>th) continue;
                        else if(d1<fuseth&&d2<fuseth)
                        {
                            if(lss>=kl1.lineLength || lse>=kl1.lineLength) //开始融合
                            { 
                                if(lss>lse && lse-kl1.lineLength<fuseth) {kl1.setEndPoint(kl2.ps);}
                                else if(lse>lss && lss-kl1.lineLength<fuseth) {kl1.setEndPoint(kl2.pe);}
                                else continue;
                            }
                            else if(lss<0 || lse<0)
                            { 
                                if(lss<lse && lse>-fuseth) {kl1.setStartPoint(s);}
                                else if(lse<lss && lss>-fuseth) {kl1.setStartPoint(e);}
                                else continue;
                            }
                        }
                        else if(lss>=kl1.lineLength&&lse>=kl1.lineLength || lss<=0&&lse<=0) continue;
                        b[j]=false;
                    }  
                    else  //kl1.lineLength<kl2.lineLength                 //是否可以融合k1
                    {
                        if(lss==0&&lse==0) continue;
                        cv::Point2f s=kl1.getStartPoint(),e=kl1.getEndPoint();
                        float d1=kl2.DistancePointoLine(s),d2=kl2.DistancePointoLine(e);
                        if(d1>th || d2>th) continue;                           
                        else if(d1<fuseth&&d2<fuseth)
                        {
                            if(lss>=kl2.lineLength || lse>=kl2.lineLength) //开始融合
                            { 
                                if(lss>lse&& lse-kl2.lineLength<fuseth){kl2.setEndPoint(s);}
                                else if(lss<lse && lss-kl2.lineLength<fuseth) {kl2.setEndPoint(e);}
                                else continue; //两直线平行 但是端点相距很远   
                            }
                            else if(lss<0 || lse<0)
                            { 
                                if(lss<lse && lse>-fuseth) {kl2.setStartPoint(kl1.ps);}
                                else if(lse<lss && lss>-fuseth) {kl2.setStartPoint(kl1.pe);}
                                else continue;
                            }
                        }
                        else if(lss>kl2.lineLength&&lse>kl2.lineLength || lss<=0&&lse<=0) continue;
                        b[i]=false; 
                    } 
                }
                else     //剔除
                {
                    if(kl1.lineLength>=kl2.lineLength)
                    {
                        if(lss>=kl1.lineLength&&lse>=kl1.lineLength || lss<=0&&lse<=0) continue;//两直线不重合
                        cv::Point2f s=kl2.getStartPoint(),e=kl2.getEndPoint();
                        float d1=kl1.DistancePointoLine(s),d2=kl1.DistancePointoLine(e);
                        if(d1<th&&d2<th){b[j]=false;}
                    }
                    else
                    {
                        if(lss>kl2.lineLength&&lse>kl2.lineLength || lss<=0&&lse<=0) continue;
                        cv::Point2f s=kl1.getStartPoint(),e=kl1.getEndPoint();
                        float d1=kl2.DistancePointoLine(s),d2=kl2.DistancePointoLine(e);
                        if(d1<th&&d2<th){b[i]=false;}
                    }
                }
            }
            if(!b[i]) break;
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

    for(size_t ii=1;ii<vstrImageLeft.size();ii++)
    { 
        // string path1=vstrImageLeft[ii]; 
        string path1=path+vstrImageLeft[ii]; 
        string path2=path+vstrImageRight[ii]; 
        cv::Mat imageMat = imread(path1,cv::IMREAD_UNCHANGED);
        cv::Mat imDepth=imread(path2,cv::IMREAD_UNCHANGED);
        imDepth.convertTo(imDepth,CV_32F,0.0002);

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

        vector<bool> b;
        optimline(ed_lines,b);
//     
        int ncull=0;
        lsd_lines.clear();
        // cv::line(ed_output, Point2f(0,0),Point2f(0,th),  Scalar(0,0,255), 2);
        for(int i=0;i<ed_lines.size();i++)
        {
            KeyLine kl=ed_lines[i];
            // cout<<i<<" "<<kl.angle<<endl;
            // cv::putText(lsd_output, to_string(i), Point2f(kl.startPointX,kl.startPointY), cv::FONT_HERSHEY_SIMPLEX,0.5, Scalar(255,0,0), 1);

            if(b[i]) //长线，都画
            {
                lsd_lines.push_back(kl);
                // cv::line(ed_output, Point2f(kl.startPointX,kl.startPointY),Point2f(kl.endPointX,kl.endPointY),  Scalar(0,0,255), 2);
                cv::line(lsd_output, Point2f(kl.startPointX,kl.startPointY),Point2f(kl.endPointX,kl.endPointY),  Scalar(0,0,255), 2);
            //     if(find(c.begin(),c.end(),i)!=c.end())cv::putText(ed_output, to_string(i), Point2f(kl.startPointX,kl.startPointY), cv::FONT_HERSHEY_SIMPLEX,0.5, Scalar(255,0,0), 1);
                cv::putText(ed_output, to_string(i), Point2f(kl.startPointX,kl.startPointY), cv::FONT_HERSHEY_SIMPLEX,0.5, Scalar(255,0,0), 1);
            //     ncull++;
            }
            else
            {
                cv::line(lsd_output, Point2f(kl.startPointX,kl.startPointY),Point2f(kl.endPointX,kl.endPointY),  Scalar(255,0,255), 2);
                // cv::putText(lsd_output, to_string(c[i]), Point2f(kl.startPointX,kl.startPointY), cv::FONT_HERSHEY_SIMPLEX,0.5, Scalar(255,0,0), 1);
                
            }
        } 
        // for(int i=0;i<fu.size();i++)
        // {
        //     KeyLine kl=fu[i];
        //     cv::line(ed_output, Point2f(kl.startPointX,kl.startPointY),Point2f(kl.endPointX,kl.endPointY),  Scalar(255,255,0), 2);

        // }
        // cout<<ii<<" detect: "<<y<<" time: "<<t<<" all:"<<ed_lines.size()<<" good:"<<ncull<<endl;


for(int i=0;i<lsd_lines.size();i++)
{
    KeyLine kl=lsd_lines[i];
    getLineCoords(kl.getStartPoint(),kl.getEndPoint(),kl.linecoods);
    vector<cv::Point2f> linecood=kl.linecoods;
    int nn=kl.linecoods.size();
    vector<cv::Point3f> coods;  coods.reserve(nn);
    for(int j=0;j<nn;j++)
    {
        cv::Point2f p = kl.linecoods[j];
        cv::circle(ed_output, p, 1, Scalar(0,0,255), -1);
        const float z = imDepth.at<float>(p.y,p.x);
        if(z>0)
        {
            float x = (p.x-cx)*z*invfx;
            float y = (p.y-cy)*z*invfy;
            coods.push_back(cv::Point3f(x,y,z));
        }
    }
    if(coods.size()>3)
        {
            vector6f out,out1;
            // std::chrono::steady_clock::time_point t1=chrono::steady_clock::now();
            // fitPlaneToPoints(coods,out);
            std::chrono::steady_clock::time_point t2=chrono::steady_clock::now();
            out1=fitPlaneRANSAC(coods);
            std::chrono::steady_clock::time_point t3=chrono::steady_clock::now();
            // double ti=chrono::duration_cast<chrono::duration<double,milli>>(t2-t1).count();
            double tt=chrono::duration_cast<chrono::duration<double,milli>>(t3-t2).count();
            // fitPlaneToPoints2(coods,out1);
            for(int i=0,iend=coods.size(); i<iend; i++)
            {
                // float r1=-(out[0]*coods[i].x+out[1]*coods[i].y+out[3])/out[2];
                // float r2=0;

                float r2= (coods[i].y-out1[4])*out1[1]/out1[2]+out1[5];
                // cout<<coods[i]<<endl;
                cout<<"real:"<<coods[i].z<<" error:"<<r2<<" "<<" ransc:"<<tt<<"--"<<out1[0]<<" "<<out1[1]<<" "<<out1[2]<<" "<<out1[3]<<endl;
            }
            cout<<i<<"---------------------------------------------------"<<endl;
        
        }







}
        cv::Mat output;
        hconcat(lsd_output,ed_output,output);
        imshow("opencviter and editer",output);
        waitKey(0);
    }

}
 




   