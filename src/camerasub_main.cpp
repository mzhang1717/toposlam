#include <iostream>
#include <vector>
#include <ros/ros.h>
#include <toposlam/MultiCameraSubscriber.h>
#include <toposlam/Mapping.h>
#include <toposlam/VocabTree.h>

int main(int argc, char **argv)
{
    std::string filename = "Descriptors.dat";
    
    cv::FileStorage fs;
    
    fs.open(filename, cv::FileStorage::READ);
    
    if (!fs.isOpened())
    {
        std::cerr << "Failed to open " << filename << std::endl;
        return -1;
    }
    
    cv::Mat vDescriptorPool;
    cv::Mat vDescriptorIndex;
    
    fs["Descriptors"] >> vDescriptorPool;
    fs["Index"] >> vDescriptorIndex;
    
    VocabTree  myTree(6, 3);
    myTree.BuildBatch(vDescriptorPool, vDescriptorIndex);
    
    return 0;
}

//int main(int argc, char **argv)
//{
    //ros::init(argc, argv, "toposlam");
    //ros::NodeHandle nh;
    //ros::Rate loop_rate(10);
    
    //const char *sCamNames[] = {"camera1", "camera2", "camera3", "camera4"};
    
    //std::vector<std::string> vCameraNames(sCamNames, sCamNames+4);
    
    //MultiCameraSubscriber CamSub(vCameraNames, false);
    //Mapping MapMaker;
    
    //while(ros::ok())
    //{
        //usrCommand = cv::waitKey(10);
        
        //CamSub.DrawBW();
        
        //ImageBWMap imagesBW;
        //CamSub.GetAndFillFrameBW(imagesBW);
        
        //MapMaker.Compute(imagesBW);
            
        //if (MapMaker.Clustering() == true)
            //break;
    
        //ros::spinOnce();
        //loop_rate.sleep();
    //}
    
    //ros::spin();
    
    //return 0;    
//}
