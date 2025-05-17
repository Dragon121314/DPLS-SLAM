#include"../include/yolo.h"
#include"yolov8_seg.h"
using namespace std;
void setParameters(util::InitParameter& initParameters)
{
	initParameters.class_names = util::dataSets::coco80;
	//initParameters.class_names = utils::dataSets::voc20;
	initParameters.num_class = 80; // for coco
	//initParameters.num_class = 20; // for voc2012
	initParameters.batch_size = 1;//8
	initParameters.dst_h = 640;
	initParameters.dst_w = 640;
	initParameters.input_output_names = { "images",  "output0" };
	initParameters.conf_thresh = 0.5f;
	initParameters.iou_thresh = 0.7f;
	initParameters.is_save=false;
	initParameters.is_show=true; 
	initParameters.src_h=480;
	initParameters.src_w=640;
	initParameters.save_path = "/home/robot/";
}
void LoadImagesRGB(const string &strAssociationFilename, vector<string> &vstrImageFilenamesRGB,vector<string> &vstrImageFilenamesD, vector<double> &vTimestamps)
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


void task(YOLOv8Seg& yolo, const util::InitParameter& param, std::vector<cv::Mat>& imgsBatch, const int& delayTime, const int& batchi)
{
	// yolo.copy(imgsBatch);
	std::chrono::steady_clock::time_point t0 = std::chrono::steady_clock::now();
	yolo.run(imgsBatch);
	// utils::DeviceTimer d_t1; yolo.preprocess(imgsBatch);  float t1 = d_t1.getUsedTime();
	// utils::DeviceTimer d_t2; yolo.infer();				  float t2 = d_t2.getUsedTime();
	// utils::DeviceTimer d_t3; yolo.postprocess(imgsBatch); float t3 = d_t3.getUsedTime();
	std::chrono::steady_clock::time_point t = std::chrono::steady_clock::now();
	double ttrack= std::chrono::duration_cast<std::chrono::duration<double> >(t - t0).count();
	std::cout<<"program run time:"<<ttrack<<std::endl;
	
	// float avg_times[3] = { t1 / param.batch_size, t2 / param.batch_size, t3 / param.batch_size };
	// sample::gLogInfo << "preprocess time = " << avg_times[0] << "; "
	// 	"infer time = " << avg_times[1] << "; "
	// 	"postprocess time = " << avg_times[2] << std::endl;
	yolo.showAndSave(param.class_names, delayTime, imgsBatch);
	// cv::Mat instance=yolo.getInstanceMask();
	// cv::imshow("param.winname", instance);
	// cv::imshow("4545", imgsBatch[0] +instance);
	// cv::waitKey(0);
	yolo.reset();
	
}

int main(int argc, char** argv)
{

	string path="/home/robot/Datasets/OpenLORIS/cafe1-2/";
	vector<string> vstrImageFilenamesRGB;
    vector<string> vstrImageFilenamesD;
    vector<double> vTimestamps;
    // string strSequence="/home/robot/Datasets/OpenLORIS/cafe1-2/";
    // string strAssociationFilename = "/home/robot/Datasets/OpenLORIS/cafe1-2/color.txt";
	//     string strSequence="/home/robot/Datasets/TUM/rgbd_dataset_freiburg3_sitting_xyz/";
    //  string strAssociationFilename = "/home/robot/Datasets/TUM/rgbd_dataset_freiburg3_sitting_xyz/associate.txt";
	string strSequence="/home/robot/Datasets/TUM/rgbd_dataset_freiburg3_walking_xyz/";
    string strAssociationFilename = "/home/robot/Datasets/TUM/rgbd_dataset_freiburg3_walking_xyz/associate.txt";
    LoadImagesRGB(strAssociationFilename, vstrImageFilenamesRGB, vstrImageFilenamesD, vTimestamps);
	

	util::InitParameter param;
	setParameters(param);
	std::string model_path = "../model/best1.trt";
	// std::string model_path = "../model/yolov8s-seg.trt";
	std::string video_path = "../../data/data.mp4";
	std::string image_path = "/home/robot/Datasets/OpenLORIS/cafe1-2/color/1560025108.791396.png"; //"../data/bus.jpg";
	int camera_id = 0;
	util::InputStream source;
	source = util::InputStream::IMAGE;
	// source = utils::InputStream::VIDEO;
	//source = utils::InputStream::CAMERA;
	// update params from command line parser
	int size = -1; 
	int batch_size = 8;

	int total_batches = 0;
	int delay_time = 1;
	cv::VideoCapture capture;
	// if(!setInputStream(source, image_path, video_path, camera_id, capture, total_batches, delay_time, param))
	// {
	// 	sample::gLogError << "read the input data errors!" << std::endl;
	// 	return -1;
	// }
	setRenderWindow(param);
	YOLOv8Seg yolo(param);
	std::vector<unsigned char> trt_file = util::loadModel(model_path);
	if (trt_file.empty())
	{
		sample::gLogError << "trt_file is empty!" << std::endl;
		return -1;
	}
	if (!yolo.init(trt_file))
	{
		sample::gLogError << "initEngine() ocur errors!" << std::endl;
		return -1;
	}
	yolo.check();
	cv::Mat frame;
	std::vector<cv::Mat> imgs_batch;
	imgs_batch.reserve(param.batch_size);
	sample::gLogInfo << imgs_batch.capacity() << std::endl;

	for(int i=0;i<1000;i++)
	{

		    string path=strSequence+vstrImageFilenamesRGB[i];
			// cout<<path<<endl;
			frame = cv::imread(path);
			// cout<<path<<endl;
			// cout<<frame.size()<<endl;
			// cv::imshow("ds",frame);
			// cv::waitKey(0);
			imgs_batch.emplace_back(frame.clone());
			task(yolo, param, imgs_batch, 0, 0);
			imgs_batch.clear();

		// int batchi = 0;
		// while (capture.isOpened())
		// {
		// 	if (batchi >= total_batches && source != utils::InputStream::CAMERA)
		// 	{
		// 		break;
		// 	}
		// 	if (imgs_batch.size() < param.batch_size)
		// 	{
		// 		if (source != utils::InputStream::IMAGE)
		// 		{
		// 			capture.read(frame);
		// 		}
		// 		else
		// 		{
		// 			frame = cv::imread(image_path);
		// 		}

		// 		if (frame.empty())
		// 		{
		// 			sample::gLogWarning << "no more video or camera frame" << std::endl;
		// 			task(yolo, param, imgs_batch, 0, 0);
		// 			imgs_batch.clear(); 
		// 			batchi++;
		// 			break;
		// 		}
		// 		else
		// 		{
		// 			imgs_batch.emplace_back(frame.clone());
		// 		}
		// 	}
		// 	else
		// 	{
		// 		task(yolo, param, imgs_batch, delay_time, batchi);
		// 		imgs_batch.clear();
		// 		batchi++;
		// 	}
		// }
	}

	return  -1;
}