#include "../include/robot_control/image_processing.h"

ImageProcessing::ImageProcessing(ros::NodeHandle n, char* type): node(n), found(0), bomb_found(false),
	processImageServer(n, PROCESS_IMAGE_ACTION, boost::bind(&ImageProcessing::processImage, this, _1), false){
	camera_sub = node.subscribe(CAMERA_TOPIC(type), 1, &ImageProcessing::handleImage, this);

	processImageServer.start();
}

ImageProcessing::~ImageProcessing(){}

void ImageProcessing::processImage(const robot_control::processImageGoalConstPtr& image_goal){
	ros::Rate r(20.0);

	bomb_found = false;
	
	while(ros::ok()){
		if(bomb_found == true)
			break;

		r.sleep();
	}

	//ROS_INFO("found !");

	robot_control::processImageResult result;

	result.succeeded = true;

    processImageServer.setSucceeded(result);
}

bool ImageProcessing::getBombDisplacement(robot_control::getBombDisplacement::Request& req,
                      robot_control::getBombDisplacement::Response& res){

    cv::Mat img_bgr;

    img_bgr = cv_ptr -> image;

    unsigned int red_pixels = 0;
    cv::Mat img_binary_0;
    cv::Mat img_binary_1;
    cv::Mat img_hsv;
    int x, y;
  	//filter 
    cv::medianBlur(img_bgr, img_bgr, 3);
	cv::cvtColor(img_bgr, img_hsv, CV_BGR2HSV);
	int min = 1000, max = -1000;

	// Perform thresholding
	cv::inRange(img_hsv, cv::Scalar(0, 100, 100), cv::Scalar(10, 255, 255), img_binary_0);
	cv::inRange(img_hsv, cv::Scalar(160, 100, 100), cv::Scalar(179, 255, 255), img_binary_1);

	// Loop over the image
	for(y = 0; y < img_binary_0.rows; y++){
		for(x = 0; x < img_binary_0.cols; x++){
			if(img_binary_0.at<unsigned char>(y,x) == 255){
				if(x > max)
					max = x;
				if(x < min)
					min = x;
			}
		}
	}
	for(y = 0; y < img_binary_1.rows; y++){
		for(x = 0; x < img_binary_1.cols; x++){
			if(img_binary_1.at<unsigned char>(y,x) == 255){
				if(x > max)
					max = x;
				if(x < min)
					min = x;
			}
		}
	}

	//ROS_INFO("min = %d | max = %d || disp = %lf", min, max, (((double)(max + min))/2.0) - ((double) img_binary_1.cols/2));

	res.displacement = (((double)(max + min))/2.0) - (((double) img_binary_1.cols/2));
}

void ImageProcessing::handleImage(const sensor_msgs::ImageConstPtr& img){
    cv::Mat img_bgr;


    try{
      cv_ptr = cv_bridge::toCvCopy(img, "bgr8");
      img_bgr = cv_ptr -> image;
    }
    catch(cv_bridge::Exception& e){
      ROS_ERROR("cv_bridge exception: %s", e.what());

      return;
    }

    unsigned int red_pixels = 0;
    cv::Mat img_binary_0;
    cv::Mat img_binary_1;
    cv::Mat img_hsv;
    int x, y;
  	//filter 
    cv::medianBlur(img_bgr, img_bgr, 3);
	cv::cvtColor(img_bgr, img_hsv, CV_BGR2HSV);

	// Perform thresholding
	cv::inRange(img_hsv, cv::Scalar(0, 100, 100), cv::Scalar(10, 255, 255), img_binary_0);
	cv::inRange(img_hsv, cv::Scalar(160, 100, 100), cv::Scalar(179, 255, 255), img_binary_1);

	// Loop over the image
	for(y = 0; y < img_binary_0.rows; y++){
		for(x = 0; x < img_binary_0.cols; x++){
			if(img_binary_0.at<unsigned char>(y,x) == 255){
				++red_pixels; // Count ’red’ pixels
			}
		}
	}
	for(y = 0; y < img_binary_1.rows; y++){
		for(x = 0; x < img_binary_1.cols; x++){
			if(img_binary_1.at<unsigned char>(y,x) == 255){
				++red_pixels; // Count ’red’ pixels
			}
		}
	}

	if(red_pixels){
		found++;
		//ROS_INFO("red image %d", found);
		if(found >= 10 && ((double) red_pixels/(img_binary_1.rows*img_binary_1.cols)) > 0.01){
			if(!bomb_found)
				ROS_INFO("Percentage: %.4lf %%", 100.0 * ((double) red_pixels/(img_binary_1.rows*img_binary_1.cols)));
			bomb_found = true;
			//cv::imshow("binary image 1", img_binary_1);
			//cv::imshow("binary image 0", img_binary_0);
		}
		
	}
	else
		found = 0;
}

int main(int argc, char **argv) {

    //Initializes ROS, and sets up a node
    ros::init(argc, argv, IMAGE_PROCESSING_NODE);

    if(argc < 2){
        ROS_INFO("Robot type not specified. Shuting down...");
        return -1;
    }

    ros::NodeHandle node;

    ImageProcessing* processor = new ImageProcessing(node, argv[1]);

    ros::ServiceServer getBombDisplacement_service = node.advertiseService(GET_BOMB_DISPLACEMENT_SERVICE, 
    	&ImageProcessing::getBombDisplacement, processor);

    ROS_INFO("Image Processing started.");

    ros::spin();

    delete processor;

    return 0;
}