#include "include/yolo.h"
#include "include/yolov8.h"

void setParameters(util::InitParameter& initParameters)
{
	initParameters.class_names = util::dataSets::coco80;
	//initParameters.class_names = utils::dataSets::voc20;
	initParameters.num_class = 80; // for coco
	//initParameters.num_class = 20; // for voc2012
	initParameters.batch_size = 1;
	initParameters.dst_h = 640;
	initParameters.dst_w = 640;
	initParameters.input_output_names = { "images",  "output0" };
	initParameters.conf_thresh = 0.25f;
	initParameters.iou_thresh = 0.45f;
	initParameters.save_path = "";
}

void task(YOLOV8& yolo, const util::InitParameter& param, std::vector<cv::Mat>& imgsBatch, const int& delayTime, const int& batchi,
	const bool& isShow, const bool& isSave)
{
	util::DeviceTimer d_t0; yolo.copy(imgsBatch);	      float t0 = d_t0.getUsedTime();
	util::DeviceTimer d_t1; yolo.preprocess(imgsBatch);  float t1 = d_t1.getUsedTime();
	util::DeviceTimer d_t2; yolo.infer();				  float t2 = d_t2.getUsedTime();
	util::DeviceTimer d_t3; yolo.postprocess(imgsBatch); float t3 = d_t3.getUsedTime();

	if(isShow)
		util::show(yolo.getObjectss(), param.class_names, delayTime, imgsBatch);
	if(isSave)
		util::save(yolo.getObjectss(), param.class_names, param.save_path, imgsBatch, param.batch_size, batchi);
	yolo.reset();
}

int main(int argc, char** argv)
{
	std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
	cv::CommandLineParser parser(argc, argv,
		{
			"{model 	|| tensorrt model file	   }"
			"{size      || image (h, w), eg: 640   }"
			"{batch_size|| batch size              }"
			"{video     || video's path			   }"
			"{img       || image's path			   }"
			"{cam_id    || camera's device id	   }"
			"{show      || if show the result	   }"
			"{savePath  || save path, can be ignore}"
		});
	// parameters
	util::InitParameter param;
	setParameters(param);
	// path
	std::string model_path = "../model/yolov8n.trt";
	std::string video_path = "../data/people.mp4";
	std::string image_path = "../data/bus.jpg";
	// camera' id
	int camera_id = 0;
	cv::Mat frame;
	frame = cv::imread(image_path);
	param.src_h = frame.rows;
	param.src_w = frame.cols;
	std::vector<cv::Mat> imgs_batch;
	// get input
	util::InputStream source;
	source = util::InputStream::IMAGE;
	// update params from command line parser
	int size = -1; // w or h
	int batch_size = 8;
	bool is_show = true;
	bool is_save = false;

	int total_batches = 0;
	int delay_time = 1;
	cv::VideoCapture capture;
	// if (!setInputStream(source, image_path, video_path, camera_id,
	// 	capture, total_batches, delay_time, param))
	// {
	// 	sample::gLogError << "read the input data errors!" << std::endl;
	// 	return -1;
	// }

	YOLOV8 yolo(param);
	// read model
	std::vector<unsigned char> trt_file = util::loadModel(model_path);
	if (trt_file.empty())
	{
		sample::gLogError << "trt_file is empty!" << std::endl;
		return -1;
	}
	// init model
	if (!yolo.init(trt_file))
	{
		sample::gLogError << "initEngine() ocur errors!" << std::endl;
		return -1;
	}
	yolo.check();

	imgs_batch.reserve(param.batch_size);
	int batchi = 0;
	imgs_batch.emplace_back(frame.clone());
	task(yolo, param, imgs_batch, 0, batchi, is_show, is_save);


	// while (capture.isOpened()) //视频
	// {
	// 	if (batchi >= total_batches && source != utils::InputStream::CAMERA)
	// 	{
	// 		break;
	// 	}
	// 	if (imgs_batch.size() < param.batch_size) // get input
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
	// 			task(yolo, param, imgs_batch, delay_time, batchi, is_show, is_save);
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
	// 		task(yolo, param, imgs_batch, delay_time, batchi, is_show, is_save);
	// 		imgs_batch.clear();
	// 		batchi++;
	// 	}
	// }
	std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
	double ttrack= std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();
	std::cout<<"program run time:"<<ttrack<<std::endl;
	return  -1;
}

