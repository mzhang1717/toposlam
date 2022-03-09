#include <iostream>
#include <vector>
#include <ros/ros.h>
#include <toposlam/MultiCameraSubscriber.h>
#include <toposlam/Mapping.h>
#include <toposlam/VocabTree.h>
#include <toposlam/Tracking.h>

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
    ros::init(argc, argv, "toposlam_track");
    ros::NodeHandle nh;
    ros::Rate loop_rate(1);
    
	const char *sCamNames[] = {"camera1", "camera2", "camera3", "camera4"};
    
    std::vector<std::string> vCameraNames(sCamNames, sCamNames+4);
    
	ImageBWMap masks;
	for(unsigned i = 0; i < 4; i++)
	{
		std::string filename = "/home/mingfeng/catkin_ws/src/toposlam/masks/" + vCameraNames[i] + "_mask.jpg";
		cv::Mat img = cv::imread(filename, CV_LOAD_IMAGE_GRAYSCALE);
		//std::cout << filename << std::endl;
		//std::cout << "Type: " << img.type() << ", Channels: " << img.channels() << ", Depth: " << img.depth() << std::endl;
		//std::cout << "Size: " << img.size() << std::endl;
		
		//cv:: namedWindow( "Display window", cv::WINDOW_AUTOSIZE );// Create a window for display.
		//cv::imshow( "Display window", img );                   // Show our image inside it.

		//cv::waitKey(0);                                          // Wait for a keystroke in the window
		
		//cv::Mat img_grey;
		//cv::cvtColor( img, img_grey, CV_BGR2GRAY, 1 );
		//cv::Mat mask;
		//cv::threshold(img, mask, 10, 255, cv::THRESH_BINARY);
		masks[vCameraNames[i]] = img;
	}    
    
    
    Tracking myTracker;
    myTracker.SetMasks(masks);
    
    // Build a map from a descriptor set
    std::string filename = "/home/mingfeng/Descriptors.dat";
    
    if ( myTracker.LoadDescriptors(filename) == false)
        return -1;
    
    //cv::FileStorage fs;
    //fs.open(filename, cv::FileStorage::READ);
    
    //if (!fs.isOpened())
    //{
        //std::cerr << "Failed to open " << filename << std::endl;
        //return -1;
    //}
    
    //cv::Mat vDescriptorPool;
    //cv::Mat vDescriptorIndex;
    
    //fs["Descriptors"] >> vDescriptorPool;
    //fs["Index"] >> vDescriptorIndex;
    
    //// Building a vocabulary tree which represents a topological map
    //VocabTree  myTree(6, 10); // 6 levels x 10 branches
    //myTree.BuildBatch(vDescriptorPool, vDescriptorIndex);
    
    
    //// Read image from camera/bagfiles
    //const char *sCamNames[] = {"camera1", "camera2", "camera3", "camera4"};
    //std::vector<std::string> vCameraNames(sCamNames, sCamNames+4);
    //MultiCameraSubscriber CamSub(vCameraNames, false);
    
    // Localization
    std::cout << "Start tracking ..." << std::endl;
    
    while(ros::ok())
    {
        myTracker.Run();
        
        //ImageBWMap imagesBW;
        //CamSub.GetAndFillFrameBW(imagesBW);
        
        //if (imagesBW.empty())
        //{
            //std::cout << "No image is received..." << std::endl;
        //}
        //else
        //{
            //cv::Mat queryDesc;
            
            //for (ImageBWMap::iterator it = imagesBW.begin(); it != imagesBW.end(); it++)
            //{
                //std::vector<cv::KeyPoint> keypoints;
                //cv::Mat descriptors;
                
                //// use FASTX
                //FASTExtractor(it->second, cv::Mat(), keypoints, descriptors, 70);
    
                ////store descriptors 
                //if (queryDesc.empty() == true)
                //{
                    //descriptors.copyTo(queryDesc);
                //}
                //else
                //{
                    //queryDesc.push_back(descriptors);
                //}
            //}
            
            //std::cout << "Querying: " << std::endl;
            //myTree.QueryFrame(queryDesc);
        //}
        
        ros::spinOnce();
        loop_rate.sleep();
    }
    
    return 0;
}


