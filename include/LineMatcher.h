/**
* This file is part of ORB-LINE-SLAM
*
* Copyright (C) 2020-2021 John Alamanos, National Technical University of Athens.
* Copyright (C) 2016-2018, Ruben Gomez-Ojeda, University of Malaga.
* Copyright (C) 2016-2018, David Zuñiga-Noël, University of Malaga.         
* Copyright (C) 2016-2018, MAPIR group, University of Malaga.    
*
* ORB-LINE-SLAM is free software: you can redistribute it and/or modify it under the terms of the GNU General Public
* License as published by the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-LINE-SLAM is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even
* the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License along with ORB-LINE-SLAM.
* If not, see <http://www.gnu.org/licenses/>.
*/

#pragma once

//STL
#include <utility>
#include <vector>

//OpenCV
#include <opencv2/core.hpp>

#include <algorithm>
#include <stdexcept>
#include <cmath> 
#include "Frame.h"
#include "MapPoint.h"
#include "MapLine.h"
#include <list>
#include <unordered_set>
#include "../line_descriptor/include/line_descriptor.hpp"
using namespace LineDescriptor;

namespace ORB_SLAM3 {

class Frame;
class MapPoint;
class MapLine;

typedef std::pair<int, int> point_2d;
typedef std::pair<point_2d, point_2d> line_2d;

inline double dot(const std::pair<double, double> &a, const std::pair<double, double> &b) {
    return (a.first*b.first + a.second*b.second);
}

inline void normalize(std::pair<double, double> &v) {
    double magnitude = std::sqrt(dot(v, v));

    v.first /= magnitude;
    v.second /= magnitude;
}

struct GridWindow {
    std::pair<int, int> width, height;
};

class GridStructure {
public:

    int rows, cols;

    GridStructure();
    
    GridStructure(int rows, int cols);

    ~GridStructure();

    std::list<int>& at(int x, int y);

    void get(int x, int y, const GridWindow &w, std::unordered_set<int> &indices) const;

    void clear();

private:

    std::vector<std::vector<std::list<int>>> grid;
    std::list<int> out_of_bounds;
};

class LineMatcher
{
public:
    int static matchNNR(const cv::Mat &desc1, const cv::Mat &desc2, float nnr, std::vector<int> &matches_12);

    int static match(const std::vector<MapLine*> &mvpLocalMapLines, Frame &CurrentFrame, float nnr, std::vector<int> &matches_12);

    int static match(const cv::Mat &desc1, const cv::Mat &desc2, float nnr, std::vector<int> &matches_12);

    int static distance(const cv::Mat &a, const cv::Mat &b);

    int static matchGrid(const std::vector<line_2d> &lines1, const cv::Mat &desc1, const GridStructure &grid, const cv::Mat &desc2, const std::vector<std::pair<double, double>> &directions2, const GridWindow &w, std::vector<int> &matches_12);

    int static SearchByProjection(Frame &CurrentFrame, Frame &LastFrame, const GridStructure &grid, const float &th, const float &angth);
    
    int static matchKL(const cv::Mat &im1, const cv::Mat &im2, const vector<KeyLine> &lastline,const vector<KeyLine> &curtline,vector<int> &matches)
    {
        chrono::steady_clock::time_point t1=chrono::steady_clock::now();
        matches.resize(lastline.size(),-1);
        float width=im1.cols,height=im1.rows;
        int match=0;

        vector<cv::Point2f> coods1,coods2,ll,rr;coods1.reserve(1000);
        vector<int> index(lastline.size());
        coods1=lastline[0].linecoods;
        index[0]=coods1.size();

        for(size_t i=1,ie=lastline.size(); i<ie; i++)
        {
            vector<cv::Point2f> kl=lastline[i].linecoods;
            coods1.insert(coods1.end(),kl.begin(),kl.end());
            index[i]=index[i-1]+kl.size();
        }

        vector<uchar> status,fstatus;
        vector<float> error;
        cv::calcOpticalFlowPyrLK(im1, im2, coods1, coods2, status, error,Size(10,10),3,cv::TermCriteria((cv::TermCriteria::COUNT)+(cv::TermCriteria::EPS), 20, (0.01)),0,1.0E-4);
        
        vector<int> indexfm;
        for(size_t i=0,endi=status.size(); i<endi; i++)
        {
            bool inborder= (0.0<=coods2[i].x && coods2[i].x<=width && 0.0<=coods2[i].y && coods2[i].y<=height);
            if(status[i]==1 && inborder) {ll.push_back(coods1[i]);rr.push_back(coods2[i]);indexfm.push_back(i);}
            else status[i]=0;
        }
        chrono::steady_clock::time_point t2=chrono::steady_clock::now();

        cv::findFundamentalMat(ll, rr, cv::FM_RANSAC, 3.0, 0.99, fstatus);
        for(size_t i=0,end=fstatus.size(); i<end; i++)  if(fstatus[i]==0) status[indexfm[i]]=0;
        chrono::steady_clock::time_point t3=chrono::steady_clock::now();
        cv::Mat last=im1.clone(),curt=im2.clone(),im;
        
        //验证直线匹配
        for(size_t i=0,ie=lastline.size(); i<ie; i++)
        {
            vector<cv::Point2f> coods;
            if(i==0) 
            {
                for(size_t ii=0,iie=index[0]; ii<iie; ii++)
                    if(status[ii]==1)
                    {
                        coods.push_back(coods2[ii]);
                        cv::circle(curt,coods2[ii],2,cv::Scalar(255,0,255),-1);
                    }
            }
            else
            {
                for(size_t ii=index[i-1],iie=index[i]; ii<iie; ii++) 
                    if(status[ii]==1)
                    {
                        coods.push_back(coods2[ii]);
                        cv::circle(curt,coods2[ii],2,cv::Scalar(255,0,255),-1);
                    }
            }
            if(coods.size()<3) continue;
            float minerror=1000.0;
            int id=-1;
            cv::Point2f pt;
            cv::Point3f normal;

            fitPlaneRANSAC(coods,normal,pt);

            for(size_t j=0,je=curtline.size(); j<je; j++)
            {

                KeyLine kl=curtline[j];
                cv::Point3f ls(kl.getStartPoint().x,kl.getStartPoint().y,1);
                cv::Point3f le(kl.getEndPoint().x,kl.getEndPoint().y,1);
                // cout<<"currt ps and pe:"<<ls<<" "<<le<<endl;

                float e=abs(normal.dot(ls))+abs(normal.dot(le));

                if(e<10.0 && e<minerror)
                {
                    // cout<<e<<endl;
                    minerror=e;
                    // id=j;
                    cv::Point2f l=(kl.pe-kl.ps);
                    cv::Point2f ssv=(kl.ps-pt);
                    cv::Point2f sev=(kl.pe-pt);

                    float ss=(l.dot(ssv))/kl.lineLength;
                    float se=(l.dot(sev))/kl.lineLength;
                    if((ss>0.0&&se<0.0) || (ss<0.0&&se>0.0)) id=j;

                }
            }
            if(id!=-1) {matches[i]=id;match++;}
        }
        // vector<int> dd=matches;
        // sort(dd.begin(),dd.end());
        // for(int i=0;i<dd.size();i++) cout<<dd[i]<<endl;

        chrono::steady_clock::time_point t4=chrono::steady_clock::now();
        double tkl=chrono::duration_cast<chrono::duration<double>>(t2-t1).count();
        double tfm=chrono::duration_cast<chrono::duration<double>>(t3-t2).count();
        double tma=chrono::duration_cast<chrono::duration<double>>(t4-t3).count();
        double all=chrono::duration_cast<chrono::duration<double>>(t4-t1).count();
        // cout<<"matchKL:"<<tkl<<" FM:"<<tfm<<" FitRancac:"<<tma<<" all:"<<all<<" "<<match<<endl;

        for(int i=0;i<lastline.size();i++) cv::line(last,lastline[i].ps,lastline[i].pe,Scalar(0,255,0),1);
        int col=im1.cols;
        for(int i=0;i<curtline.size();i++) 
        {
            cv::Point2f psc,pec;
            psc.x=int(curtline[i].startPointX+col);psc.y=int(curtline[i].startPointY);
            pec.x=int(curtline[i].endPointX+col);pec.y=int(curtline[i].endPointY);
            cv::line(curt,psc,pec,Scalar(0,255,0),1);
        }

        cv::hconcat(last,curt,im);
        for(int i=0,ie=matches.size(); i<ie; i++)
        {
            if(matches[i]==-1)continue;
            int j=matches[i];
            cv::Point2f psc,pec; 
            psc.x=int(curtline[j].pt.x+col);psc.y=int(curtline[j].pt.y);

            // cv::line(im,ps,pe,Scalar(0,255,0),2);
            // cv::line(im,psc,pec,Scalar(0,255,0),2);
            cv::line(im,lastline[i].pt,psc,Scalar(0,255,255),1);
        }
        // cv::imshow("dd",im);
        // cv::waitKey(0);

        return match;
    }

void static fitPlaneRANSAC(const std::vector<cv::Point2f> &points, cv::Point3f &bestPlane, cv::Point2f &pt, int maxIterations = 50, double distanceThreshold = 0.001) 
{
    // cout<<"5.1"<<endl;
    int bestInlierCount = 0, n=points.size()*0.9;
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<> dis(0, points.size() - 1);
// cout<<"5.2"<<endl;
    for (int iter = 0; iter < maxIterations; ++iter) 
    {
        int idx1 = dis(gen);
        int idx2 = dis(gen);

        if (idx1 == idx2)  continue; // 避免选到相同的点 
        const cv::Point2f  p1 = points[idx1];
        const cv::Point2f  p2 = points[idx2];

        // 计算直线参数 Ax + By + C = 0
        double A = p2.y - p1.y;      // A = y2 - y1
        double B = p1.x - p2.x;      // B = x1 - x2
        double C = p2.x * p1.y - p1.x * p2.y; // C = x2*y1 - x1*y2
        double D = 1.0/sqrt(pow(A,2)+pow(B,2));
        cv::Point3f normal=cv::Point3f(A*D,B*D,C*D);

        int inlierCount = 0; 
        for(int i=0,iend=points.size(); i<iend; i++) 
        {
            cv::Point3f point(points[i].x, points[i].y, 1);
            double distance =abs(normal.dot(point));
            // double distance =points[i].z-Eigen::Vector3f(plane[0],plane[1],plane[3]).dot(Eigen::Vector3f(points[i].x,points[i].y,1))/plane[2];
            if (distance < distanceThreshold) inlierCount++;
        }
        // 更新最佳平面
        if(inlierCount > bestInlierCount) {
            bestInlierCount = inlierCount;
            bestPlane = normal;
            pt=p1;
        }
        if(inlierCount>n) break;
    }
}

};


} // namesapce ORB_SLAM3
