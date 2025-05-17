
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
#include <unordered_set>
#include <ctime>
#include <cstdlib>
#include <iostream>
#include <random>


using namespace cv;
using namespace LineDescriptor;
using namespace std;

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
                    // cv::circle(curt,coods2[ii],2,cv::Scalar(255,0,255),-1);
                }
        }
        else
        {
            for(size_t ii=index[i-1],iie=index[i]; ii<iie; ii++) 
                if(status[ii]==1)
                {
                    coods.push_back(coods2[ii]);
                    // cv::circle(curt,coods2[ii],2,cv::Scalar(255,0,255),-1);
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

    chrono::steady_clock::time_point t4=chrono::steady_clock::now();
    double tkl=chrono::duration_cast<chrono::duration<double>>(t2-t1).count();
    double tfm=chrono::duration_cast<chrono::duration<double>>(t3-t2).count();
    double tma=chrono::duration_cast<chrono::duration<double>>(t4-t3).count();
    double all=chrono::duration_cast<chrono::duration<double>>(t4-t1).count();
    // cout<<"matchKL:"<<tkl<<" FM:"<<tfm<<" FitRancac:"<<tma<<" all:"<<all<<" "<<match<<endl;

    for(int i=0;i<lastline.size();i++) 
    {
        // if(matches[i]==-1)continue;
        cv::line(last,lastline[i].ps,lastline[i].pe,Scalar(0,255,0),2);
    }
    float col=im1.cols;
    for(int i=0;i<matches.size();i++) 
    {
        int id=matches[i];
        // if(matches[i]==-1)continue;
        cv::Point2f psc,pec;
        psc.x=curtline[id].startPointX;psc.y=curtline[id].startPointY;
        pec.x=curtline[id].endPointX;pec.y=curtline[id].endPointY;
        // cv::line(curt,psc,pec,Scalar(0,255,0),1);
        cv::line(curt,psc,pec,Scalar(0,255,0),2);
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
    cv::imshow("dd",im);
    cv::waitKey(0);

    return match;
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
    for(int i=0;i<nLine;i++)
    {
        if(ed_lines[i].lineLength<40)b[i]=false;
    }
    for(int i=0;i<nLine;i++) if(b[i]) out.push_back(ed_lines[i]); 
}
    


int main( int argc, char** argv )
{

  vector<string>  vstrImageLeft;
  vector<string>  vstrImageRight;
    string tum_path="/home/robot/Datasets/TUM/rgbd_dataset_freiburg3_walking_xyz/";
    vector<double> vTimestamps;

     string strAssociationFilename ="/home/robot/Datasets/TUM/rgbd_dataset_freiburg3_walking_xyz/associate.txt";
      LoadImagesRGB(strAssociationFilename, vstrImageLeft, vstrImageRight, vTimestamps);




  /* get parameters from comand line */
  /* create a pointer to a BinaryDescriptor object with deafult parameters */
  Ptr<BinaryDescriptor> bd = BinaryDescriptor::createBinaryDescriptor();
  /* create a structure to store extracted lines */
  vector<KeyLine> lsd_lines,ed_lines;
  cv::Mat lastimage;


    for(size_t ii=0;ii<vstrImageLeft.size();ii++)
    {
        cout<<fixed;
        cout<<"time======"<<vTimestamps[ii]<<"================================"<<endl;
        string path=tum_path+vstrImageLeft[ii];
        string path1=tum_path+vstrImageLeft[ii+1];
        cv::Mat imageMat = imread(path,cv::IMREAD_UNCHANGED);
        cv::Mat imageMat1 = imread(path1,cv::IMREAD_UNCHANGED);


        //   cv::Mat imageMat = imread(vstrImageLeft[ii],cv::IMREAD_UNCHANGED);
        if( imageMat.data == NULL )
        {
            std::cout << "Error, image could not be loaded. Please, check its path" << path1<<std::endl;
            return -1;
        }
        vector<LineDescriptor::KeyLine> eed_l,sd_l;
        bd->detect(imageMat, sd_l);
        bd->detect(imageMat1, eed_l);
        lsd_lines.clear();
        ed_lines.clear();
        optimline(eed_l,ed_lines);
        optimline(sd_l,lsd_lines);
        for(int i=0;i<lsd_lines.size();i++)
        {
            lsd_lines[i].linecoods.reserve(lsd_lines[i].lineLength);
            getLineCoords(lsd_lines[i].getStartPoint(),lsd_lines[i].getEndPoint(),lsd_lines[i].linecoods);
        }
        for(int i=0;i<ed_lines.size();i++)
        {
            ed_lines[i].linecoods.reserve(ed_lines[i].lineLength);
            getLineCoords(ed_lines[i].getStartPoint(),ed_lines[i].getEndPoint(),ed_lines[i].linecoods);
        }
 
        {
            vector<int> matche;
            matchKL(imageMat,imageMat1,lsd_lines,ed_lines,matche);

        }
    }


    return 0;
}