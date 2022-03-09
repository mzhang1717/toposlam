#include <iostream>
#include <vector>
#include <ros/ros.h>
#include <toposlam/MultiCameraSubscriber.h>
#include <toposlam/Mapping.h>
#include <toposlam/VocabTree.h>

//template <typename T>
//inline std::string ToString(T tX)
//{
    //std::ostringstream oStream;
    //oStream << tX;
    //return oStream.str();
//}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "toposlam");
    ros::NodeHandle nh;
    ros::Rate loop_rate(1);
    
    const char *sCamNames[] = {"camera1", "camera2", "camera3", "camera4"};
    
    std::vector<std::string> vCameraNames(sCamNames, sCamNames+4);
    
    //MultiCameraSubscriber CamSub(vCameraNames);
	//InfoMap camInfo = CamSub.GetInfo();
	
	//for (InfoMap::iterator it = camInfo.begin(); it != camInfo.end(); it++)
	//{
		//std::cout << it->first << ":" << std::endl;
		//std::cout << "Size: " << it->second.height << " x " << it->second.width << std::endl;
		//std::cout << "Distortion model: " << it->second.distortion_model << std::endl;
		
		//cv::Mat K(3, 3, CV_32FC1, &(it->second.K));
		//std::cout << "Intrisic model: " << K << std::endl;
		//std::cout << "----------------------------------" << std::endl;
	//}
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
   
    Mapping MapMaker;
    MapMaker.SetMasks(masks);
     
    while(ros::ok())
    {
        std::cout << "--------------------------------" << std::endl;
        
        //ImageBWMap imagesBW;
        //CamSub.GetAndFillFrameBW(imagesBW);

 
        //std::string indexFrame = ToString(nFrame);
        
        //int nSub =1;
        //for (ImageBWMap::iterator it = imagesBW.begin(); it != imagesBW.end(); it++)
        //{
            //std::string indexSubImage = ToString(nSub);
            //std::string strFileName = std::string("../Img_")  + indexFrame + std::string("_") + indexSubImage + std::string(".jpg");
            
            ////try 
            ////{
                //cv::imwrite(strFileName, it->second);
            ////}
            ////catch (runtime_error& ex) 
            ////{
                ////fprintf(stderr, "Exception converting image to JPG format: %s\n", ex.what());
                ////return 1;
            ////}
            
            //nSub++;
        //}
        
        //nFrame++;
        
        //**********************//
        //MapMaker.Compute(imagesBW);
 
		MapMaker.Run();
		//MapMaker.CreateMasks();
		
        //if (MapMaker.Clustering() == true)
        //    break;
        //*************************//
    
        ros::spinOnce();
        loop_rate.sleep();
    }
    
    return 0;    
}
