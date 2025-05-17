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


#ifndef FRAME_H
#define FRAME_H

#include<vector>

#include "Thirdparty/DBoW2/DBoW2/BowVector.h"
#include "Thirdparty/DBoW2/DBoW2/FeatureVector.h"

#include "Thirdparty/Sophus/sophus/geometry.hpp"

#include "ImuTypes.h"
#include "ORBVocabulary.h"

#include "Converter.h"
#include "Settings.h"

#include <mutex>
#include <opencv2/opencv.hpp>

#include "Eigen/Core"
#include "sophus/se3.hpp"
#include "LineMatcher.h"

#include "../line_descriptor/include/line_descriptor.hpp"
#include <../yolov8/include/utils.h>
using namespace LineDescriptor;
typedef Eigen::Matrix<float,6,1> vector6f;

namespace ORB_SLAM3
{
#define FRAME_GRID_ROWS 48//网格的行数
#define FRAME_GRID_COLS 64//网格的列数
class MapLine;
class MapPoint;
class KeyFrame;
class ConstraintPoseImu;
class GeometricCamera;
class ORBextractor;
class Frame
{
public:
    Frame();
    /**
     * @brief 拷贝构造函数 ， Copy constructor.
     * @details 复制构造函数, mLastFrame = Frame(mCurrentFrame) \n
     * 如果不是自定以拷贝函数的话，系统自动生成的拷贝函数对于所有涉及分配内存的操作都将是浅拷贝 \n
     * @param[in] frame 引用
     * @note 另外注意，调用这个函数的时候，这个函数中隐藏的this指针其实是指向目标帧的
     */
    Frame(const Frame &frame);
    /**
     * @brief 为双目相机准备的构造函数
     * @param[in] imLeft            左目图像
     * @param[in] imRight           右目图像
     * @param[in] timeStamp         时间戳
     * @param[in] extractorLeft     左目图像特征点提取器句柄
     * @param[in] extractorRight    右目图像特征点提取器句柄
     * @param[in] voc               ORB字典句柄
     * @param[in] K                 相机内参矩阵
     * @param[in] distCoef          相机去畸变参数
     * @param[in] bf                相机基线长度和焦距的乘积
     * @param[in] thDepth           远点和近点的深度区分阈值
     */
    Frame(const cv::Mat &imLeft, const cv::Mat &imRight, const double &timeStamp, ORBextractor* extractorLeft, ORBextractor* extractorRight, ORBVocabulary* voc, cv::Mat &K, cv::Mat &distCoef, const float &bf, const float &thDepth, GeometricCamera* pCamera,Frame* pPrevF = static_cast<Frame*>(NULL), const IMU::Calib &ImuCalib = IMU::Calib());

    /**
     * @brief 为RGBD相机准备的帧构造函数
     * 
     * @param[in] imGray        对RGB图像灰度化之后得到的灰度图像
     * @param[in] imDepth       深度图像
     * @param[in] timeStamp     时间戳
     * @param[in] extractor     特征点提取器句柄
     * @param[in] voc           ORB特征点词典的句柄
     * @param[in] K             相机的内参数矩阵
     * @param[in] distCoef      相机的去畸变参数
     * @param[in] bf            baseline*bf
     * @param[in] thDepth       远点和近点的深度区分阈值
     */
    Frame(const cv::Mat &imGray, const cv::Mat &imDepth,const cv::Mat &imInstance,  const double &timeStamp, ORBextractor* extractor,ORBVocabulary* voc, cv::Mat &K, cv::Mat &distCoef, const float &bf, const float &thDepth, GeometricCamera* pCamera,Frame* pPrevF = static_cast<Frame*>(NULL), const IMU::Calib &ImuCalib = IMU::Calib());

    /**
     * @brief 为单目相机准备的帧构造函数
     * 
     * @param[in] imGray                            //灰度图
     * @param[in] timeStamp                         //时间戳
     * @param[in & out] extractor                   //ORB特征点提取器的句柄
     * @param[in] voc                               //ORB字典的句柄
     * @param[in] K                                 //相机的内参数矩阵
     * @param[in] distCoef                          //相机的去畸变参数
     * @param[in] bf                                //baseline*f
     * @param[in] thDepth                           //区分远近点的深度阈值
     */
    Frame(const cv::Mat &imGray, const double &timeStamp, ORBextractor* extractor,ORBVocabulary* voc, GeometricCamera* pCamera, cv::Mat &distCoef, const float &bf, const float &thDepth, Frame* pPrevF = static_cast<Frame*>(NULL), const IMU::Calib &ImuCalib = IMU::Calib());

    // Destructor
    // ~Frame();

    /**
     * @brief 提取图像的ORB特征，提取的关键点存放在mvKeys，描述子存放在mDescriptors
     * 
     * @param[in] flag          标记是左图还是右图。0：左图  1：右图
     * @param[in] im            等待提取特征点的图像
     */
    void ExtractORB(int flag, const cv::Mat &im, const int x0, const int x1);

    // 存放在mBowVec中
    /**
     * @brief 计算词袋模型，计算词包 mBowVec 和 mFeatVec ，其中 mFeatVec 记录了属于第i个node（在第4层）的ni个描述子
     * @see CreateInitialMapMonocular() TrackReferenceKeyFrame() Relocalization()
     */
    void ComputeBoW();
    /**
     * @brief 用 Tcw 更新 mTcw 以及类中存储的一系列位姿
     * 
     * @param[in] Tcw 从世界坐标系到当前帧相机位姿的变换矩阵
     */
    void SetPose(const Sophus::SE3<float> &Tcw);

    // Set IMU velocity
    void SetVelocity(Eigen::Vector3f Vw);

    Eigen::Vector3f GetVelocity() const;

    // Set IMU pose and velocity (implicitly changes camera pose)
    void SetImuPoseVelocity(const Eigen::Matrix3f &Rwb, const Eigen::Vector3f &twb, const Eigen::Vector3f &Vwb);

    Eigen::Matrix<float,3,1> GetImuPosition() const;
    Eigen::Matrix<float,3,3> GetImuRotation();
    Sophus::SE3<float> GetImuPose();

    Sophus::SE3f GetRelativePoseTrl();
    Sophus::SE3f GetRelativePoseTlr();
    Eigen::Matrix3f GetRelativePoseTlr_rotation();
    Eigen::Vector3f GetRelativePoseTlr_translation();

    void SetNewBias(const IMU::Bias &b);

    // Check if a MapPoint is in the frustum of the camera
    // and fill variables of the MapPoint to be used by the tracking
    bool isInFrustum(MapPoint* pMP, float viewingCosLimit);
    bool isInFrustum_l(MapLine *pML, float viewingCosLimit);

    bool ProjectPointDistort(MapPoint* pMP, cv::Point2f &kp, float &u, float &v);

    Eigen::Vector3f inRefCoordinates(Eigen::Vector3f pCw);

    // Compute the cell of a keypoint (return false if outside the grid)
    bool PosInGrid(const cv::KeyPoint &kp, int &posX, int &posY);

    vector<size_t> GetFeaturesInArea(const float &x, const float  &y, const float  &r, const int minLevel=-1, const int maxLevel=-1, const bool bRight = false) const;

    // Search a match for each keypoint in the left image to a keypoint in the right image.
    // If there is a match, depth is computed and the right coordinate associated to the left keypoint is stored.
    void ComputeStereoMatches();

    // Associate a "right" coordinate to a keypoint if there is valid depth in the depthmap.
    void ComputeStereoFromRGBD(const cv::Mat &imDepth);
    void ComputeStereoFromRGBD(const cv::Mat &imDepth,const cv::Mat &imInstance, const vector<util::Box> &box, vector<bool> &vbDyanmic);

    // Backprojects a keypoint (if stereo/depth info available) into 3D world coordinates.
    bool UnprojectStereo(const int &i, Eigen::Vector3f &x3D);





bool inBorder(const cv::Point2f &pt)
{
    float img_x = pt.x;
    float img_y = pt.y;
    return 0.0 <= img_x && img_x < mWidth  && 0.0 <= img_y && img_y < mHeight ;
}

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

    Eigen::Vector3f UnprojectStereoLines(const double &u, const double &v, const double &z)
    {
        // convert the point in image to the point in world coordinate.
        Eigen::Vector3f P;      // point in camera coordinate
        P(0) = (u-cx)*z*invfx;
        P(1) = (v-cy)*z*invfy;
        P(2) = z;
        return mRwc*P+mOw;
    }

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


vector6f fitPlaneRANSAC(const std::vector<Eigen::Vector3f> &points, int maxIterations = 50, double distanceThreshold = 0.01) 
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
        Eigen::Vector3f v1 = points[idx1];
        Eigen::Vector3f v2 = points[idx2];
        // 构造两个向量
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
            float a=(v1[0]-points[i][0])*normal[0], b=(v1[1]-points[i][1])*normal[1], c=(v1[2]-points[i][2])*normal[2];
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


float findDepth(const cv::Mat &imDepth, const cv::Point2f &p, const float &f)
{

    float d=0.0;
    int w=2;
    for(int i=-w; i<=w; i++)
    {
        for(int j=-w; j<=w; j++)
        {
            if(p.x+i<0 || p.x+i>mWidth || p.y+j<0 || p.y+j>mHeight) continue;
            d=imDepth.at<float>(p.y+j,p.x+i);
            if(d>0 && abs(d-f)<0.015) return d;
        }
    }
    return -1.0;
}


// 空间一致性校验，剔除动态线特征
void UndistortKeyLines(const cv::Mat &imDepth, const cv::Mat &imInstance, const cv::Mat &imgray)
{ 
    bool bUndist=true;
    if(mDistCoef.at<float>(0)==0.0)
    {
        bUndist=false;
    }
    N_l = mvKeys_Line.size();   // update N_l
    if(N_l==0) return;

    int np=0;
    for(int i=0; i<N_l; i++)
    { 
        mvKeys_Line[i].linecoods.reserve(mvKeys_Line[i].lineLength);
        getLineCoords(mvKeys_Line[i].getStartPoint(),mvKeys_Line[i].getEndPoint(),mvKeys_Line[i].linecoods);
        np += mvKeys_Line[i].linecoods.size();
    } 
    mvKeysUn_Line = mvKeys_Line; 
    //step 1 畸变矫正
    if(bUndist)
    {
        cv::Mat mat_s(np,2,CV_32F);
        int idx=0;
        for(int i=0; i<N_l; i++)
        {
            vector<cv::Point2f> vp=mvKeys_Line[i].linecoods;
            for(int j=0,jend=vp.size(); j<jend; j++)
            {
                mat_s.at<float>(idx,0)=vp[j].x;
                mat_s.at<float>(idx,1)=vp[j].y;
                idx++;
            }
        }  
        mat_s=mat_s.reshape(2); 
        cv::undistortPoints(mat_s,mat_s,mK,mDistCoef,cv::Mat(),mK); 
        mat_s=mat_s.reshape(1); 
        mvKeysUn_Line.resize(N_l); 
        int in=0;
        for(int i=0; i<N_l; i++)
        { 
            mvKeysUn_Line[i]=mvKeys_Line[i]; 
            int n=mvKeys_Line[i].linecoods.size();
            vector<cv::Point2f> lc(n); 
            for(int j=0; j<n; j++,in++)
            {
                lc[j]=cv::Point2f(mat_s.at<float>(in,0),mat_s.at<float>(in,1));
            }
            mvKeysUn_Line[i].linecoods=lc;
            mvKeysUn_Line[i].setStartPoint(cv::Point2f(lc[0])); 
            mvKeysUn_Line[i].setEndPoint(cv::Point2f(lc[n-1])); 
        } 
    }

    //=============空间一致性校验================
    vector<KeyLine> keyline,keylineUn;
    vector<vector<float>> ld;
    keyline.reserve(N_l);keylineUn.reserve(N_l);ld.reserve(N_l);

    for(int i=0; i<N_l; i++)
    {
        //从未矫正的特征点提供的坐标来读取深度图像拿到这个点的深度数据.
        KeyLine kl = mvKeys_Line[i];
        KeyLine klU = mvKeysUn_Line[i];
        vector<cv::Point2f> linecood=kl.linecoods;
        const int nn=kl.linecoods.size();
        vector<Eigen::Vector3f> coods;  coods.reserve(nn);//计算空间直线存储真实3D坐标的容器
        vector<Eigen::Vector3f> l3dc(nn); //存储归一化x,y坐标和真实d的容器
        for(int j=0;j<nn;j++)
        {
            cv::Point2f p = kl.linecoods[j];
            const float z = imDepth.at<float>(p.y,p.x);
            float x = (p.x-cx)*invfx;
            float y = (p.y-cy)*invfy;
            if(z>0)
            {
                coods.push_back(Eigen::Vector3f(x*z,y*z,z));
                l3dc[j]=Eigen::Vector3f(x,y,z);
            }
            else l3dc[j]=Eigen::Vector3f(x,y,0);
        }

        if(coods.size()<=2) continue;
        
        vector6f out1;
        std::chrono::steady_clock::time_point t2=chrono::steady_clock::now();
        out1=fitPlaneRANSAC(coods);
        std::chrono::steady_clock::time_point t3=chrono::steady_clock::now();
        double tt=chrono::duration_cast<chrono::duration<double,milli>>(t3-t2).count();

        //============================================================================================
        bool bstart=false, bend=false;
        float ds=0,de=0;
        int ls=0,le=nn-1;
        while((!bstart||!bend) && (ls<le))  //初步选定直线起止点
        {
            if(!bstart)
            {
                if(l3dc[ls][2]>0)
                {
                    float r2=((l3dc[ls][0]*l3dc[ls][2]-out1[3])*out1[0]+(l3dc[ls][1]*l3dc[ls][2]-out1[4])*out1[1])/(2*out1[2])+out1[5];
                    if(abs(r2-l3dc[ls][2])<0.015){ds=l3dc[ls][2];bstart=true; /*klU.setStartPoint(klU.linecoods[ls]);cout<<"real:"<<l3dc[ls][2]<<" start:"<<r2<<endl;*/}
                }
                // else
                // {
                //     float r2=(out1[1]*out1[4]-out1[2]*out1[5])/(out1[1]*l3dc[ls][1]-out1[2]); //计算深度，然后在3*3窗口找合适值
                //     float d=findDepth(imDepth,linecood[ls],r2);
                //     if(d>0){ds=(d+r2)/2;bstart=true; /*klU.setStartPoint(klU.linecoods[ls]);*/cout<<"real:"<<d<<" jisuan strt:"<<r2<<endl;}
                // }
                ls++;
            }
            if(!bend)
            {
                if(l3dc[le][2]>0)
                {
                    float r2=((l3dc[le][0]*l3dc[le][2]-out1[3])*out1[0]+(l3dc[le][1]*l3dc[le][2]-out1[4])*out1[1])/(2*out1[2])+out1[5];
                    if(abs(r2-l3dc[le][2])<0.015){de=l3dc[le][2]; bend=true;/* klU.setEndPoint(klU.linecoods[le]);cout<<"real:"<<l3dc[le][2]<<" end:"<<r2<<endl;*/}
                    
                }
                // else
                // {
                //     float r2=(out1[1]*out1[4]-out1[2]*out1[5])/(out1[1]*l3dc[le][1]-out1[2]); //计算深度，然后在3*3窗口找合适值
                //     float d=findDepth(imDepth,linecood[le],r2);
                //     if(d>0){de=(d+r2)/2; bend=true; /*klU.setEndPoint(klU.linecoods[le]);*/cout<<"real:"<<d<<" jisuan end:"<<r2<<endl;}
                    
                // }
                le--;
            }
        }
        int newls=ls,newle=le;
        if(ls!=0)
        {
            while(newls>0) if(l3dc[newls-1][2]>0 && abs(l3dc[newls-1][2]-l3dc[newls][2])<0.08) newls--;
                            else break;
        }
        if(le!=nn-1)
        {
            while(newle<nn-1) if(l3dc[newle+1][2]>0 && abs(l3dc[newle+1][2]-l3dc[newle][2])<0.08) newle++;
                                else break;
        }
        // int leiji=0;  //TODO....截断长的留下
        // for(int i=newls; i<newle; i++)
        // {
        //     if(l3dc[i+1][2]>0)
        //     {
        //         if(leiji==0 && abs(l3dc[i+1][2]-l3dc[i][2])<0.08) continue;
        //         else
        //         {
        //             if(abs(l3dc[i+1][2]-l3dc[i-leiji][2])<0.08) {leiji=0;continue;}
        //             else leiji++;

        //             if(leiji==3)
        //             {
        //                 ls=i-3;
        //                 if(ls-newls>newle-ls)
        //                 {
        //                     klU.setStartPoint(klU.linecoods[newls]);
        //                     klU.setEndPoint(klU.linecoods[ls]);
        //                 }
        //                 else
        //                 {
        //                     klU.setStartPoint(klU.linecoods[ls]);
        //                     klU.setEndPoint(klU.linecoods[newle]);
        //                 }
        //                 break;
        //             }
        //         }
        //     }
        // }
        bool badline=false;   //直接舍弃
        for(int i=newls; i<newle; i++)
        {
            if(l3dc[i][2]>0) 
            {
                if(abs(l3dc[i+1][2]-l3dc[i][2])<0.08) continue;
                else {badline=true;break;}
            }
            else {badline=true;break;}
            // else
            // {
            //     float r2=(out1[1]*out1[4]-out1[2]*out1[5])/(out1[1]*l3dc[i][1]-out1[2]);
            //     float r1=(out1[1]*out1[4]-out1[2]*out1[5])/(out1[1]*l3dc[i-1][1]-out1[2]);
            //     if(abs(r2-r1)<0.04) l3dc[i][2]=(r2+l3dc[i-1][2])/2;
            //     else l3dc[i][2]=l3dc[i-1][2]+(r2-r1)*0.1;
            // }
        }

        if(badline) continue;
        klU.setStartPoint(klU.linecoods[newls]);
        klU.setEndPoint(klU.linecoods[newle]);
        if(klU.lineLength<30) continue; 

        klU.linecoods.assign(klU.linecoods.begin()+newls, klU.linecoods.begin()+newle+1);
        vector<float> dept(klU.linecoods.size());
        for(int i=0,j=newls; j<=newle; i++,j++)
        {
            dept[i]=l3dc[j][2];
        }
        kl.setStartPoint(kl.linecoods[newls]);
        kl.setEndPoint(kl.linecoods[newle]);
        kl.linecoods.assign(kl.linecoods.begin()+newls, kl.linecoods.begin()+newle+1);
        
        bool bls=(imInstance.at<float>(kl.startPointY,kl.startPointX)==0);
        bool ble=(imInstance.at<float>(kl.endPointY,kl.endPointX)==0);
        if(!bls || !ble) continue;
        keyline.push_back(kl);
        keylineUn.push_back(klU);
        ld.push_back(dept);

        //==========================================================
        // bool bstart=false, bend=false;
        // float ds=0,de=0;
        // for(int i=0,j=nn-1; (!bstart || !bend)&&(i<j); i++,j--)
        // {
        //     if(l3dc[i][2]>0 && !bstart)
        //     {
        //         float r2=(l3dc[i][1]-out1[4])*out1[1]/out1[2]+out1[5];
        //         ds=l3dc[i][2];
        //         if(abs(r2-ds)<0.015){bstart=true; klU.setStartPoint(klU.linecoods[i]);}
        //     }
        //     if(l3dc[j][2]>0 && !bend)
        //     {
        //         float r2=(l3dc[j][1]-out1[4])*out1[1]/out1[2]+out1[5];
        //         de=l3dc[j][2];
        //         if(abs(r2-de)<0.015){bend=true; klU.setEndPoint(klU.linecoods[j]);}
        //     }
        // }

        //=========================================================
        // if(ds>0 && de>0)
        // {
        //     nline++;
        //     std::pair<float,float> d(ds,de);
        //     std::pair<float,float> s;
        //     s.first=klU.startPointX-mbf/ds;
        //     s.second=klU.endPointX-mbf/de; 
        //     mvDepth_l[i]=d;
        //     mvDisparity_l[i]=s;
        //     // cout<<"---------------------------------------------------youxiao:"<<newls<<" "<<newle<<endl;

        // }
        // else cout<<"---------------------------------------------------"<<newls<<" "<<newle<<endl;
    
        
        // // 获取其横纵坐标，注意 NOTICE 是校正前的特征点的
        // const float &sx = kl.startPointX;
        // const float &sy = kl.startPointY;
        // const float &ex = kl.endPointX;
        // const float &ey = kl.endPointY;

        // const float ds = imDepth.at<float>(sy,sx);
        // const float de = imDepth.at<float>(ey,ex); 

        // // 如果获取到的深度点合法(d>0), 那么就保存这个特征点的深度,并且计算出等效的在假想的右图中该特征点所匹配的特征点的横坐标
        // if(ds>0 && de>0)
        // {
        //     std::pair<float,float> d(ds,de);
        //     std::pair<float,float> s;
        //     s.first=klU.startPointX-mbf/ds;
        //     s.second=klU.endPointX-mbf/de; 
        //     mvDepth_l[i]=d;
        //     mvDisparity_l[i]=s;
        // }
    } 

    N_l=keyline.size();
    mvKeys_Line=keyline;
    mvKeysUn_Line=keylineUn;
    mLinepointDepth=ld;
    if(N_l==0) return;
    // //===============end for 空间一致性校验========


    
    mvDepth_l.resize(N_l,pair<float,float>(-1.0f,-1.0f));
    mvDisparity_l.resize(N_l,pair<float,float>(-1,-1));
    mvle_l.resize(N_l,Eigen::Vector3d(0,0,0));
    mvpMapLines = vector<MapLine*>(N_l,static_cast<MapLine*>(NULL));
    mvbOutlier_Line = vector<bool>(N_l,false);
    for (int i=0; i < N_l; i++) 
    { 
        // KeyLine kl=mvKeys_Line[i];
        // float ds=imDepth.at<float>(kl.getStartPoint().y,kl.getStartPoint().x);
        // float de=imDepth.at<float>(kl.getEndPoint().y,kl.getEndPoint().x);
        // std::pair<float,float> d(ds,de);

        std::pair<float,float> d(mLinepointDepth[i][0],mLinepointDepth[i].back());
        if(d.first<=0 || d.second<=0) continue;
        std::pair<float,float> s;
        s.first=mvKeysUn_Line[i].startPointX-mbf/d.first;
        s.second=mvKeysUn_Line[i].endPointX-mbf/d.second;
        mvDepth_l[i]=d;
        mvDisparity_l[i]=s;
        Eigen::Vector3d sp_lun; sp_lun << mvKeysUn_Line[i].startPointX, mvKeysUn_Line[i].startPointY, 1.0;
        Eigen::Vector3d ep_lun; ep_lun << mvKeysUn_Line[i].endPointX,   mvKeysUn_Line[i].endPointY,   1.0;
        Eigen::Vector3d le_l; le_l << sp_lun.cross(ep_lun); le_l = le_l/sqrt(le_l(0)*le_l(0) + le_l(1)*le_l(1) );
        mvle_l[i] = le_l;
    }

    Ptr<BinaryDescriptor>  lbd = BinaryDescriptor::createBinaryDescriptor();
    lbd->compute(imgray, mvKeys_Line, mDescriptors_Line); 
}

void UndistortKeyLinesNoEquation(const cv::Mat &imDepth, const cv::Mat &imInstance, const cv::Mat &imgray)
{ 
    bool bUndist=true;
    if(mDistCoef.at<float>(0)==0.0)
    {
        bUndist=false;
    }
    N_l = mvKeys_Line.size();   // update N_l
    if(N_l==0) return;

    int np=0;
    for(int i=0; i<N_l; i++)
    { 
        mvKeys_Line[i].linecoods.reserve(mvKeys_Line[i].lineLength);
        getLineCoords(mvKeys_Line[i].getStartPoint(),mvKeys_Line[i].getEndPoint(),mvKeys_Line[i].linecoods);
        np += mvKeys_Line[i].linecoods.size();
    } 
    mvKeysUn_Line = mvKeys_Line; 
    //step 1 畸变矫正
    if(bUndist)
    {
        cv::Mat mat_s(np,2,CV_32F);
        int idx=0;
        for(int i=0; i<N_l; i++)
        {
            vector<cv::Point2f> vp=mvKeys_Line[i].linecoods;
            for(int j=0,jend=vp.size(); j<jend; j++)
            {
                mat_s.at<float>(idx,0)=vp[j].x;
                mat_s.at<float>(idx,1)=vp[j].y;
                idx++;
            }
        }  
        mat_s=mat_s.reshape(2); 
        cv::undistortPoints(mat_s,mat_s,mK,mDistCoef,cv::Mat(),mK); 
        mat_s=mat_s.reshape(1); 
        mvKeysUn_Line.resize(N_l); 
        int in=0;
        for(int i=0; i<N_l; i++)
        { 
            mvKeysUn_Line[i]=mvKeys_Line[i]; 
            int n=mvKeys_Line[i].linecoods.size();
            vector<cv::Point2f> lc(n); 
            for(int j=0; j<n; j++,in++)
            {
                lc[j]=cv::Point2f(mat_s.at<float>(in,0),mat_s.at<float>(in,1));
            }
            mvKeysUn_Line[i].linecoods=lc;
            mvKeysUn_Line[i].setStartPoint(cv::Point2f(lc[0])); 
            mvKeysUn_Line[i].setEndPoint(cv::Point2f(lc[n-1])); 
        } 
    }

    mvDepth_l.resize(N_l,pair<float,float>(-1.0f,-1.0f));
    mvDisparity_l.resize(N_l,pair<float,float>(-1,-1));
    mvle_l.resize(N_l,Eigen::Vector3d(0,0,0));
    mvpMapLines = vector<MapLine*>(N_l,static_cast<MapLine*>(NULL));
    mvbOutlier_Line = vector<bool>(N_l,false);
    for (int i=0; i < N_l; i++) 
    {
        float ds=imDepth.at<float>(mvKeys_Line[i].startPointY,mvKeys_Line[i].startPointX);
        float de=imDepth.at<float>(mvKeys_Line[i].endPointY,mvKeys_Line[i].endPointX);
        if(ds>0 && de>0)
        {
            std::pair<float,float> d(ds,de);
            std::pair<float,float> s;
            s.first=mvKeysUn_Line[i].startPointX-mbf/d.first;
            s.second=mvKeysUn_Line[i].endPointX-mbf/d.second; 
            mvDepth_l[i]=d;
            mvDisparity_l[i]=s;

        }
        Eigen::Vector3d sp_lun; sp_lun << mvKeysUn_Line[i].startPointX, mvKeysUn_Line[i].startPointY, 1.0;
        Eigen::Vector3d ep_lun; ep_lun << mvKeysUn_Line[i].endPointX,   mvKeysUn_Line[i].endPointY,   1.0;
        Eigen::Vector3d le_l; le_l << sp_lun.cross(ep_lun); le_l = le_l/sqrt(le_l(0)*le_l(0) + le_l(1)*le_l(1) );
        mvle_l[i] = le_l;
    }
    Ptr<BinaryDescriptor>  lbd = BinaryDescriptor::createBinaryDescriptor();
    lbd->compute(imgray, mvKeys_Line, mDescriptors_Line); 
}

// 线特征剔除融合
void optimline(vector<KeyLine> &ed_lines, /*vector<bool> &b*/vector<KeyLine> &out)
{
    int nLine=ed_lines.size(); 
    vector<bool> b(nLine,true);
    float ss[nLine][nLine],se[nLine][nLine];//es[nLine][nLine],ee[nLine][nLine];
    float ang[nLine];
    for(int i = 0; i < nLine; i++ )
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
    // float fuseth=2.0;
    float angleth=0.01745*5;  //0.01745=1度 
    // float fuseangth=0.01745*2;   
    for(int i = 0; i < nLine; i++ )
    {
        if(!b[i]) continue;
        KeyLine &kl1 = ed_lines[i]; 
        for(int j = i+1; j < nLine; j++ )
        {
            if(!b[j]) continue;
            KeyLine &kl2 = ed_lines[j];
            if(abs(ang[i]-ang[j])<angleth || abs(ang[i]+ang[j]-CV_PI)<angleth)
            {
                float lss=ss[i][j],lse=se[i][j];
                // if(abs(ang[i]-ang[j])<fuseangth || abs(ang[i]+ang[j]-CV_PI)<fuseangth) //====计算是否可以融合
                // {
                //     if(kl1.lineLength>=kl2.lineLength)//是否可以融合K2
                //     {
                //         if(lss==0&&lse==0) continue;
                //         cv::Point2f s=kl2.getStartPoint(),e=kl2.getEndPoint();
                //         float d1=kl1.DistancePointoLine(s),d2=kl1.DistancePointoLine(e);
                //         if(d1>th || d2>th) continue;
                //         else if(d1<fuseth&&d2<fuseth)
                //         {
                //             if(lss>=kl1.lineLength || lse>=kl1.lineLength) //开始融合
                //             { 
                //                 if(lss>lse && lse-kl1.lineLength<fuseth) {kl1.setEndPoint(kl2.ps);}
                //                 else if(lse>lss && lss-kl1.lineLength<fuseth) {kl1.setEndPoint(kl2.pe);}
                //                 else continue;
                //             }
                //             else if(lss<0 || lse<0)
                //             { 
                //                 if(lss<lse && lse>-fuseth) {kl1.setStartPoint(s);}
                //                 else if(lse<lss && lss>-fuseth) {kl1.setStartPoint(e);}
                //                 else continue;
                //             }
                //         }
                //         else if((lss>=kl1.lineLength&&lse>=kl1.lineLength) || (lss<=0&&lse<=0)) continue;
                //         b[j]=false;
                //     }  
                //     else  //kl1.lineLength<kl2.lineLength                 //是否可以融合k1
                //     {
                //         if(lss==0&&lse==0) continue;
                //         cv::Point2f s=kl1.getStartPoint(),e=kl1.getEndPoint();
                //         float d1=kl2.DistancePointoLine(s),d2=kl2.DistancePointoLine(e);
                //         if(d1>th || d2>th) continue;                           
                //         else if(d1<fuseth&&d2<fuseth)
                //         {
                //             if(lss>=kl2.lineLength || lse>=kl2.lineLength) //开始融合
                //             { 
                //                 if(lss>lse&& lse-kl2.lineLength<fuseth){kl2.setEndPoint(s);}
                //                 else if(lss<lse && lss-kl2.lineLength<fuseth) {kl2.setEndPoint(e);}
                //                 else continue; //两直线平行 但是端点相距很远   
                //             }
                //             else if(lss<0 || lse<0)
                //             { 
                //                 if(lss<lse && lse>-fuseth) {kl2.setStartPoint(kl1.ps);}
                //                 else if(lse<lss && lss>-fuseth) {kl2.setStartPoint(kl1.pe);}
                //                 else continue;
                //             }
                //         }
                //         else if((lss>kl2.lineLength&&lse>kl2.lineLength) || (lss<=0&&lse<=0)) continue;
                //         b[i]=false; 
                //     } 
                // }
                // else     //剔除
                {
                    if(kl1.lineLength>=kl2.lineLength)
                    {
                        if((lss>=kl1.lineLength&&lse>=kl1.lineLength) || (lss<=0&&lse<=0)) continue;//两直线不重合
                        cv::Point2f s=kl2.getStartPoint(),e=kl2.getEndPoint();
                        float d1=kl1.DistancePointoLine(s),d2=kl1.DistancePointoLine(e);
                        if(d1<th&&d2<th){b[j]=false;}
                    }
                    else
                    {
                        if((lss>kl2.lineLength&&lse>kl2.lineLength) || (lss<=0&&lse<=0)) continue;
                        cv::Point2f s=kl1.getStartPoint(),e=kl1.getEndPoint();
                        float d1=kl2.DistancePointoLine(s),d2=kl2.DistancePointoLine(e);
                        if(d1<th&&d2<th){b[i]=false;}
                    }
                }
            }
            if(!b[i]) break;
        }
    }
    for(int i=0;i<nLine;i++) if(b[i]) out.push_back(ed_lines[i]); 
}
    



    ConstraintPoseImu* mpcpi;

    bool imuIsPreintegrated();
    void setIntegrated();

    bool isSet() const;

    // Computes rotation, translation and camera center matrices from the camera pose.
    /**
     * @brief 根据相机位姿,计算相机的旋转,平移和相机中心等矩阵.
     * @details 其实就是根据Tcw计算mRcw、mtcw和mRwc、mOw.
     */
    void UpdatePoseMatrices();

    // Returns the camera center.
    inline Eigen::Vector3f GetCameraCenter(){
        return mOw;
    }

    // Returns inverse of rotation
    inline Eigen::Matrix3f GetRotationInverse(){
        return mRwc;
    }

    inline Sophus::SE3<float> GetPose() const {
        //TODO: can the Frame pose be accsessed from several threads? should this be protected somehow?
        return mTcw;
    }

    inline Eigen::Matrix3f GetRwc() const {return mRwc;}
    inline Eigen::Matrix3f GetRcw() const {return mRcw;}
    inline Eigen::Matrix<float,3,1> Gettcw() const {return mtcw;}
    inline Eigen::Vector3f GetOw() const {return mOw;}

    inline bool HasPose() const {return mbHasPose;}

    inline bool HasVelocity() const {return mbHasVelocity;}

private:
    //Sophus/Eigen migration
    Sophus::SE3<float> mTcw;
    Eigen::Matrix<float,3,3> mRwc;//相机坐标相对世界坐标的旋转
    Eigen::Matrix<float,3,1> mOw;//相机坐标系原点在世界中的位置坐标
    Eigen::Matrix<float,3,3> mRcw;
    Eigen::Matrix<float,3,1> mtcw;
    bool mbHasPose;

    //Rcw_ not necessary as Sophus has a method for extracting the rotation matrix: Tcw_.rotationMatrix()
    //tcw_ not necessary as Sophus has a method for extracting the translation vector: Tcw_.translation()
    //Twc_ not necessary as Sophus has a method for easily computing the inverse pose: Tcw_.inverse()

    Sophus::SE3<float> mTlr, mTrl;
    Eigen::Matrix<float,3,3> mRlr;
    Eigen::Vector3f mtlr;


    // IMU linear velocity
    Eigen::Vector3f mVw;
    bool mbHasVelocity;

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    // Vocabulary used for relocalization.
    ORBVocabulary* mpORBvocabulary;

    // Feature extractor. The right is used only in the stereo case.
    ORBextractor* mpORBextractorLeft, *mpORBextractorRight;//ORB特征提取器句柄,其中右侧的提取器句柄只会在双目输入的情况中才会被用到

    // Frame timestamp.
    double mTimeStamp;//帧的时间戳

    // Calibration matrix and OpenCV distortion parameters.
    cv::Mat mK;//相机的内参数矩阵
    Eigen::Matrix3f mK_;
    static float fx;        ///<x轴方向焦距
    static float fy;        ///<y轴方向焦距
    static float cx;        ///<x轴方向光心偏移
    static float cy;        ///<y轴方向光心偏移
    static float invfx;     ///<x轴方向焦距的逆
    static float invfy;     ///<x轴方向焦距的逆
    cv::Mat mDistCoef;//去畸变参数.格式为：(k1,k2,p1,p2,k3)

    // Stereo baseline multiplied by fx.
    float mbf;//baseline x fx

    // Stereo baseline in meters.
    float mb;//相机的基线长度,单位为米

    // Threshold close/far points. Close points are inserted from 1 view.
    // Far points are inserted as in the monocular case from 2 views.
    float mThDepth;//判断远点和近点的深度阈值

    //双目Pinhole模型只取左图上的特征点个数(mvKeys.size())作为该帧的特征点个数N，没有用到右影像上提取的特征点个数(mvKeysRight.size())
    //双目KannalaBrandt8模型分别获取左右影像上的特征点个数，并将其分别赋给Nleft、Nright。而最终的总特征点个数N为两者之和。
    int N,N_l;

    // Vector of keypoints (original for visualization) and undistorted (actually used by the system).
    // In the stereo case, mvKeysUn is redundant as images must be rectified.
    // In the RGB-D case, RGB images can be distorted.
    std::vector<cv::KeyPoint> mvKeys, mvKeysRight,mvkeyDyan;
    std::vector<cv::KeyPoint> mvKeysUn;//校正mvKeys后的特征点


    std::vector<KeyLine> mvKeys_Line, mvKeysRight_Line,mvKeysLine; //mvKeys_Line正常跟踪用的线，mvKeysLine通过融合和优化后的线，没有剔除短线
    std::vector<KeyLine> mvKeysUn_Line;
    float mHeight,mWidth;
    std::vector<MapLine*> mvpMapLines;
    std::vector<pair<float,float>> mvDepth_l;
    vector<vector<float>> mLinepointDepth; //直线上点的深度
    std::vector<int> matches_12;  //当前帧和上一帧通过光流匹配的直线效果
    // for PLslam
    // std::vector<LineFeature*> stereo_ls;
    std::vector<pair<float,float>> mvDisparity_l;
    std::vector<Eigen::Vector3d> mvle_l;//线段起点终点坐标叉乘结果，即线平面的法向量
    cv::Mat mDescriptors_Line, mDescriptorsRight_Line;
    std::vector<bool> mvbOutlier_Line;
    float inv_width, inv_height; 
    vector<bool> mvbInDynamic,mvbInDynaline;



    ///@note 之所以对于双目摄像头只保存左图像矫正后的特征点,是因为对于双目摄像头,一般得到的图像都是矫正好的,这里再矫正一次有些多余.\n
    ///校正操作是在帧的构造函数中进行的。
    
    // Corresponding stereo coordinate and depth for each keypoint.
    // "Monocular" keypoints have a negative value.
    // 对于双目，mvuRight存储了左目像素点在右目中的对应点的横坐标 （因为纵坐标是一样的）
    // mvDepth对应的深度
    // 单目摄像头，这两个容器中存的都是-1

    ///@note 对于单目摄像头，这两个容器中存的都是-1
    ///对于双目相机,存储左目像素点在右目中的对应点的横坐标 （因为纵坐标是一样的）
    // Corresponding stereo coordinate and depth for each keypoint.
    std::vector<MapPoint*> mvpMapPoints;// 每个特征点对应的MapPoint.如果特征点没有对应的地图点,那么将存储一个空指针
    // "Monocular" keypoints have a negative value.
    std::vector<float> mvuRight;//存储右图匹配点索引 u-指代横坐标,因为最后这个坐标是通过各种拟合方法逼近出来的，所以使用float存储
    std::vector<float> mvDepth;//对应的深度

    // 内部实际存储的是std::map<WordId, WordValue>，WordId 和 WordValue 表示Word在叶子中的id 和TF-IDF权重
    DBoW2::BowVector mBowVec;
    // 内部实际存储 std::map<NodeId, std::vector<unsigned int> >
    // NodeId 表示节点id，std::vector<unsigned int> 中实际存的是该节点id下所有特征点在图像中的索引
    DBoW2::FeatureVector mFeatVec;

    // ORB descriptor, each row associated to a keypoint.
    cv::Mat mDescriptors, mDescriptorsRight;// 左目摄像头和右目摄像头特征点对应的描述子

    // MapPoints associated to keypoints, NULL pointer if no association.
    // Flag to identify outlier associations.
    std::vector<bool> mvbOutlier; // 观测不到Map中的3D点，属于外点的特征点标记,在 Optimizer::PoseOptimization 使用了
    int mnCloseMPs;

    // Keypoints are assigned to cells in a grid to reduce matching complexity when projecting MapPoints.
    static float mfGridElementWidthInv;//一个像素占据的格子宽度
    static float mfGridElementHeightInv;//一个像素占据的格子高度
    // 每个格子分配的特征点数，将图像分成格子，保证提取的特征点比较均匀，这个向量中存储的是每个图像网格内特征点的id（左图）
    std::vector<std::size_t> mGrid[FRAME_GRID_COLS][FRAME_GRID_ROWS];

    IMU::Bias mPredBias;//当前帧预计分时用的偏置

    // IMU bias
    IMU::Bias mImuBias;

    // Imu calibration
    IMU::Calib mImuCalib;

    
    IMU::Preintegrated* mpImuPreintegrated;//相对于上一关键帧的预计分，在tracking的PreintegrateIMU()中计算完成
    KeyFrame* mpLastKeyFrame;

    // Pointer to previous frame,在构建帧时使用lastframe初始化
    Frame* mpPrevFrame;
    IMU::Preintegrated* mpImuPreintegratedFrame;//相对于上一帧的预计分，在tracking的PreintegrateIMU()中计算完成

    // Current and Next Frame id.
    static long unsigned int nNextId;//< Next Frame id.
    long unsigned int mnId;//< Current Frame id.

    
    KeyFrame* mpReferenceKF;//在UpdateLocalKeyFrames函数中将与自己共视程度最高的关键帧作为参考关键帧

    // Scale pyramid info.
    int mnScaleLevels;//<图像金字塔的层数
    float mfScaleFactor;//<图像金字塔的尺度因子
    float mfLogScaleFactor;//<图像金字塔的尺度因子的对数值，用于仿照特征点尺度预测地图点的尺度
    vector<float> mvScaleFactors;//<图像金字塔每一层的缩放因子
    vector<float> mvInvScaleFactors;//<图像金字塔每一层的缩放因子倒数
    vector<float> mvLevelSigma2;// 高斯模糊的时候，使用的方差
    vector<float> mvInvLevelSigma2;// 高斯模糊的时候，使用的方差倒数

    // Undistorted Image Bounds (computed once).
     //用于确定画格子时的边界 ，未校正图像的边界，只需要计算一次，因为是类的静态成员变量）

    static float mnMinX;
    static float mnMaxX;
    static float mnMinY;
    static float mnMaxY;

    static bool mbInitialComputations;

    map<long unsigned int, cv::Point2f> mmProjectPoints;//当前帧进行重投影的像素坐标，第一个是对应地图点的索引，第二个是像素点
    map<long unsigned int, cv::Point2f> mmMatchedInImage;

    string mNameFile;

    int mnDataset;

#ifdef REGISTER_TIMES
    double mTimeORB_Ext;//提取图像ORB特征用时
    double mTimeStereoMatch;//双目鱼眼相机左右图像匹配的耗时
#endif

private:

    // Undistort keypoints given OpenCV distortion parameters.
    // Only for the RGB-D case. Stereo must be already rectified!
    // (called in the constructor).
    //用内参对特征点去畸变，结果报存在mvKeysUn中
    void UndistortKeyPoints();

    // Computes image bounds for the undistorted image (called in the constructor).
    /**
     * @brief 计算去畸变图像的边界
     * 
     * @param[in] imLeft            需要计算边界的图像
     */
    void ComputeImageBounds(const cv::Mat &imLeft);

    // Assign keypoints to the grid for speed up feature matching (called in the constructor).
    /**
     * @brief 将提取到的特征点分配到图像网格中，该函数由构造函数调用
     */
    void AssignFeaturesToGrid();

    bool mbIsSet;//图像帧是否已经设置了位姿

    bool mbImuPreintegrated;//当前帧是否完成了预计分

    std::mutex *mpMutexImu;

public:
    //Pinhole模型只创建一个mpCamera相机模型；只有KannalaBrandt8双目相机才创建mpCamera、mpCamera2两个相机模型对象，见tracking类构造函数从setting类或配置文件加载参数
    GeometricCamera* mpCamera, *mpCamera2;

    int Nleft, Nright;//只有双目KannalaBrandt8模型Nleft、Nright才有值；其余情况下只有N有值，Nleft和Nright默认都为-1
    //Number of Non Lapping Keypoints
    int monoLeft, monoRight;

    //For stereo matching
    std::vector<int> mvLeftToRightMatch, mvRightToLeftMatch;//双目中左右图像之间的匹配点，即观察到的时同一个地图点

    //For stereo fisheye matching
    static cv::BFMatcher BFmatcher;

    //Triangulated stereo observations using as reference the left camera. These are
    //computed during ComputeStereoFishEyeMatches
    std::vector<Eigen::Vector3f> mvStereo3Dpoints;

    //Grid for the right image
    std::vector<std::size_t> mGridRight[FRAME_GRID_COLS][FRAME_GRID_ROWS];

    Frame(const cv::Mat &imLeft, const cv::Mat &imRight, const double &timeStamp, ORBextractor* extractorLeft, ORBextractor* extractorRight, ORBVocabulary* voc, cv::Mat &K, cv::Mat &distCoef, const float &bf, const float &thDepth, GeometricCamera* pCamera, GeometricCamera* pCamera2, Sophus::SE3f& Tlr,Frame* pPrevF = static_cast<Frame*>(NULL), const IMU::Calib &ImuCalib = IMU::Calib());

    //Stereo fisheye
    void ComputeStereoFishEyeMatches();

    bool isInFrustumChecks(MapPoint* pMP, float viewingCosLimit, bool bRight = false);

    Eigen::Vector3f UnprojectStereoFishEye(const int &i);

    cv::Mat imgLeft, imgRight;

    void PrintPointDistribution(){
        int left = 0, right = 0;
        int Nlim = (Nleft != -1) ? Nleft : N;
        for(int i = 0; i < N; i++){
            if(mvpMapPoints[i] && !mvbOutlier[i]){
                if(i < Nlim) left++;
                else right++;
            }
        }
        cout << "Point distribution in Frame: left-> " << left << " --- right-> " << right << endl;
    }

    Sophus::SE3<double> T_test;
};

}// namespace ORB_SLAM

#endif // FRAME_H
