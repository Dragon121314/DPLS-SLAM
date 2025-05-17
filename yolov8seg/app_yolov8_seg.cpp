#include"../include/yolo.h"
#include"yolov8_seg.h"

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
	initParameters.conf_thresh = 0.25f;
	initParameters.iou_thresh = 0.7f;
	initParameters.is_save=false;
	initParameters.is_show=true;
	initParameters.save_path = "/home/robot/";
}

void task(YOLOv8Seg& yolo, const util::InitParameter& param, std::vector<cv::Mat>& imgsBatch, const int& delayTime, const int& batchi)
{
	yolo.copy(imgsBatch);
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
	// yolo.showAndSave(param.class_names, delayTime, imgsBatch);
	cv::Mat instance;
	std::vector<util::Box> segbox;
	yolo.getInstanceMask(instance,segbox);
	for(int i=0;i<segbox.size();i++)
	{
		util::Box bo=segbox[i];
		cv::rectangle(imgsBatch[0], cv::Rect(bo.left,bo.top,bo.right,bo.bottom), cv::Scalar(0,255,0), 2, cv::LINE_AA);


	}

	cv::Mat black_mask(instance.size(), CV_8UC1, cv::Scalar(0));

    // 遍历彩色掩码的每个像素
    for (int y = 0; y < instance.rows; ++y) {
        for (int x = 0; x < instance.cols; ++x) {
            cv::Vec3b color_pixel = instance.at<cv::Vec3b>(y, x);
            // 如果彩色像素不为黑色（即有颜色），则在黑色掩码中设置为白色
            if (color_pixel[0] > 0 || color_pixel[1] > 0 || color_pixel[2] > 0) {
                black_mask.at<uchar>(y, x) = 255;
            }
        }
    }
	cv::imshow("param.winname", black_mask);
	
	cv::cvtColor(black_mask, black_mask, cv::COLOR_GRAY2BGR);
	std::vector<cv::Mat> images_to_concat = {imgsBatch[0], black_mask};
	cv::Mat concatenated_image;
	cv::hconcat(images_to_concat, concatenated_image);
	// cv::imwrite("../r1.png", concatenated_image);
	cv::imshow("4545", imgsBatch[0] +instance);
	cv::waitKey();
	yolo.reset();
	
}

int main(int argc, char** argv)
{
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
	util::InitParameter param;
	setParameters(param);
	std::string model_path = "../model/yolov8s-seg.trt";
	std::string video_path = "../../data/data.mp4";
	std::string image_path = "/home/robot/Datasets/TUM/rgbd_dataset_freiburg3_walking_xyz/rgb/1341846315.197984.png"; //"../data/bus.jpg";
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
	if(!setInputStream(source, image_path, video_path, camera_id, capture, total_batches, delay_time, param))
	{
		sample::gLogError << "read the input data errors!" << std::endl;
		return -1;
	}
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
	int batchi = 0;
	while (capture.isOpened())
	{
		if (batchi >= total_batches && source != util::InputStream::CAMERA)
		{
			break;
		}
		if (imgs_batch.size() < param.batch_size)
		{
			if (source != util::InputStream::IMAGE)
			{
				capture.read(frame);
			}
			else
			{
				frame = cv::imread(image_path);
			}
			if (frame.empty())
			{
				sample::gLogWarning << "no more video or camera frame" << std::endl;
				task(yolo, param, imgs_batch, delay_time, batchi);
				imgs_batch.clear(); 
				batchi++;
				break;
			}
			else
			{
				imgs_batch.emplace_back(frame.clone());
			}
		}
		else
		{
			task(yolo, param, imgs_batch, delay_time, batchi);
			imgs_batch.clear();
			batchi++;
		}
	}
	return  -1;
}