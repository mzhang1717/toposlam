#include <iostream>
#include <vector>
#include <ros/ros.h>
#include <toposlam/System.h>

//void FASTExtractor(cv::Mat img_in, cv::Mat img_mask, std::vector<cv::KeyPoint>& keypoints, cv::Mat& descriptors, int threshold)
//{
    ////cv::FASTX(img_in, keypoints, threshold, true, cv::FastFeatureDetector::TYPE_9_16);
    
    ////will create a detector that attempts to find
    ////200 - 250 FAST Keypoints, and will at most run
    ////FAST feature detection 50 times until that
    ////number of keypoints are found
    //cv::DynamicAdaptedFeatureDetector detector(new cv::FastAdjuster(threshold, true), 200, 250, 50);
                              
    //detector.detect(img_in, keypoints, img_mask);
    
    //cv::SIFT Sift_Extractor;
    //Sift_Extractor(img_in, img_in, keypoints, descriptors, true);
    
    //return;
//}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "toposlam_core");
	ros::NodeHandle nh;	
	
	System sys;
	
	sys.Run();
    
    return 0;
}


