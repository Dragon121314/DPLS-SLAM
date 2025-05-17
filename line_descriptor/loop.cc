/*
V1_02_medium数据集
双目视觉惯性
CurrentKT TimeStamp:1403715567.112143    LoopMatched TimeStamp1403715556.612143
CurrentKT TimeStamp:1403715567.4621429   LoopMatched TimeStamp1403715557.612143


CurrentKT TimeStamp:1403715567.112143    LoopMatched TimeStamp1403715556.612143
1403715567112143104.png                                     1403715556612143104.png
双目视觉
CurrentKT TimeStamp:1403715570.3121431    LoopMatched TimeStamp1403715564.2121432
CurrentKT TimeStamp:1403715593.4621429    LoopMatched TimeStamp1403715553.5121431

CurrentKT TimeStamp:1403715591.112143     LoopMatched TimeStamp1403715565.5621431
CurrentKT TimeStamp:1403715593.2121432    LoopMatched TimeStamp1403715554.362143
CurrentKT TimeStamp:1403715598.8121431    LoopMatched TimeStamp1403715570.162143
*/   

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
using namespace cv;
using namespace std;

int main()
{



    namedWindow("histImage",WINDOW_AUTOSIZE);
    for(int i=0;i<10;i++)
    {

    
        Mat img1=imread("/home/robot/Datasets/EuRoC/V1_02_medium/mav0/cam0/data/1403715567112143104.png");

        if(img1.empty())
        {
            cout<<"请确认输入的图片路径是否正确"<<endl;
            return -1;
        }
        Mat gray;
        cvtColor(img1,gray,COLOR_BGR2GRAY);
        //设置提取直方图的相关变量
        Mat hist;//用于存放直方图计算结果
        const int channels[1]={0};//通道索引.对于彩色图像，可以是 {0, 1, 2}，分别对应于蓝色、绿色和红色通道。
        float inRanges[2]={0,255};
        const float*ranges[1]={inRanges};//像素灰度值范围
        const int bins[1]={256};//直方图的维度，其实就是像素灰度值的最大值
    //1：这是图像通道的数量。1 表示计算单通道图像的直方图。对于灰度图像，这个值通常是 1。对于彩色图像，如果分别计算每个颜色通道的直方图，这个值可以是 3（对应于 BGR 通道）。
    //mat()掩码用于指定图像中哪些区域应该被包含在直方图计算中
    //1：这是直方图的维度。在这里，1 表示直方图是一维的。
    //bins：这是直方图的桶（bins）数量。桶的数量决定了直方图的分辨率。对于8位图像，bins 通常是 256，因为 8 位图像有 256 个可能的像素值。
    //ranges：这是一个包含两个元素的数组，定义了直方图计算的值的范围。对于8位图像，这个数组通常是{0,256}，表示考虑所有可能的像素值。
        calcHist(&img1,1,channels,Mat(),hist,1,bins,ranges);//计算图像直方图

        // cv::normalize(hist, hist, 0, 1, cv::NORM_L1);


        //准备绘制直方图
        int hist_w=512;
        int hist_h=400;
        int width=2;
        Mat histImage1=Mat::zeros(hist_h,hist_w,CV_8UC3);

        for(int i=1;i<=hist.rows;++i)
        {
            rectangle(histImage1,Point(width*(i-1),hist_h-1),Point(width*i-1,hist_h-cvRound(hist.at<float>(i-1)/20)),Scalar(255,255,255),-1);
        }
        imshow("histImage"+to_string(i),histImage1);
        imshow("gray"+to_string(i),img1);
    }
    waitKey(0);
    return 0;
}



// int main() 
// {
//     // 读取图像
//     cv::Mat img = cv::imread("/home/robot/Datasets/EuRoC/V1_02_medium/mav0/cam0/data/1403715567112143104.png");
//     if (img.empty()) {
//         std::cerr << "Could not read the image." << std::endl;
//         return 1;
//     }

//     // 定义直方图的参数
//     int bins = 256; // 8位图像的桶数量
//     float range[] = {0, 256}; // 像素值的范围
//     const float* histRange = {range};
//     int channels[] = {0}; // 灰度图像的通道索引

//     // 计算直方图
//     cv::Mat hist;
//     cv::calcHist(&img, 1, channels, cv::Mat(), hist, 1, &bins, &histRange, true, false);

//     // 归一化直方图
//     cv::normalize(hist, hist, 0, 1, cv::NORM_L1);

//     // 绘制直方图
//     cv::Mat histImg = cv::Mat::ones(256, bins, CV_8UC1) * 255;
//     for (int i = 0; i < hist.rows; i++) {
//         cv::line(histImg, cv::Point(i, 256 - cvRound(hist.at<float>(i))), cv::Point(i, 256 - cvRound(hist.at<float>(i))), cv::Scalar(0), 2, 8, 0);
//     }

//     // 显示直方图
//     cv::imshow("Histogram", histImg);
//     cv::waitKey(0);

//     return 0;
// }

