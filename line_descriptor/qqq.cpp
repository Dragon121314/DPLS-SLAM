

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
// using namespace LineDescriptor;
using namespace line_descriptor;
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
     string strAssociationFilename ="/home/robot/Datasets/TUM/rgbd_dataset_freiburg1_desk2/associate.txt";
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
    bool NeedNew=false;//需要重新提取线特征
    bool New=false;  //已经提取线特征
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



for(size_t ii=1;ii<vstrImageLeft.size();ii++)
{
    // stringstream ss;
    // ss << setfill('0') << setw(10) << ii;
    // string path="/home/robot/Datasets/KITTI/2011_09_30_drive_0020_sync/image_00/data/" + ss.str() + ".png";
    string path1=path+vstrImageLeft[ii];
    string path2=path+vstrImageLeft[ii-1];
    cv::Mat imageMat1 = imread(path1,cv::IMREAD_UNCHANGED);
    cv::Mat imageMat2 = imread(path2,cv::IMREAD_UNCHANGED);
    height=imageMat1.rows;
    width=imageMat1.cols;

  std::vector<KeyLine> keylines1, keylines2;
  cv::Mat descr1, descr2;


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
    if( keylines1[i].octave == 0 && keylines1[i].lineLength>40)
    {
      lbd_octave1.push_back( keylines1[i] );
      left_lbd.push_back( descr1.row( i ) );
    }
  }

  for ( int j = 0; j < (int) keylines2.size(); j++ )
  {
    if( keylines2[j].octave == 0 && keylines1[j].lineLength>40)
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
//   bdm->match( left_lbd, right_lbd, matches );
  bdm->knnMatch( left_lbd, right_lbd, matchesknn ,2);
  for(size_t i=0,e=lbd_octave1.size();i<e;i++)
  {
    matches.push_back(matchesknn[i][0]);
  }

  // bdm->match( left_lbd, right_lbd, matches );
cout<<left_lbd.size()<<right_lbd.size()<<matchesknn.size()<<endl;
std::chrono::steady_clock::time_point t4=std::chrono::steady_clock::now();
double detect=std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();
double compute=std::chrono::duration_cast<std::chrono::duration<double> >(t3-t2).count();
double match=std::chrono::duration_cast<std::chrono::duration<double> >(t4-t3).count();
double all=std::chrono::duration_cast<std::chrono::duration<double> >(t4 - t1).count();
cout<<"detect "<<detect<<" compute "<<compute<<" match time: "<<match<<" all "<<all<<endl;
cv::Mat outImg;
std::vector<char> mask( matches.size(), 1 );
drawLineMatches(imageMat1,lbd_octave1, imageMat2, lbd_octave2, matches, outImg, Scalar::all( -1 ), Scalar::all( -1 ), mask,
                     DrawLinesMatchesFlags::DEFAULT );
  imshow( "Matches", outImg );
  waitKey(0);
} 
 
}



   