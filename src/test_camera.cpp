#include <iostream>
#include <vector>
#include <ros/ros.h>
#include <toposlam/MultiCameraSubscriber.h>
#include <toposlam/CameraModel.h>
#include <toposlam/ImageWindow.h>
//#include <toposlam/Mapping.h>
//#include <toposlam/VocabTree.h>
#include <math.h>       /* tan */
#include <toposlam/ImageProcess.h>
#include <toposlam/Types.h>

typedef std::map<std::string, CameraModel> TaylorCameraMap;
//typedef std::map<std::string, TooN::Matrix<4, 3> > Corner3DMap;
//typedef std::map<std::string, TooN::Matrix<4, 2> > Corner2DMap;
//typedef std::map<std::string, TooN::Matrix<3, 3> > CameraMatrixMap;
//typedef std::map<std::string, cv::Mat> ProjectMap;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "toposlam_camera");
    ros::NodeHandle nh;
    ros::Rate loop_rate(10);
    
    const char *sCamNames[] = {"camera1", "camera2", "camera3", "camera4"};
    
    std::vector<std::string> vCameraNames(sCamNames, sCamNames+4);
    
    MultiCameraSubscriber CamSub(vCameraNames);

	InfoMap camInfo = CamSub.GetInfo();
	
	for (InfoMap::iterator it = camInfo.begin(); it != camInfo.end(); it++)
	{
		std::cout << it->first << ":" << std::endl;
		std::cout << "Size: " << it->second.height << " x " << it->second.width << std::endl;
		std::cout << "Distortion model: " << it->second.distortion_model << std::endl;
		
		cv::Mat K(3, 3, CV_32FC1, &(it->second.K));
		std::cout << "Intrisic model: " << K << std::endl;
		std::cout << "----------------------------------" << std::endl;
	}

	ImageSizeMap mImageSizes = CamSub.GetSizes();
	ParamMap mParams = CamSub.GetParams();
	SE3Map mmPoses = CamSub.GetPoses();
	
	TaylorCameraMap mmCameraModels;
	
     // Create camera models here
	for(ImageSizeMap::iterator it = mImageSizes.begin(); it != mImageSizes.end(); ++it)
	{
		std::string camName = it->first;
		
		cv::Size irImageSize = mImageSizes[camName];
		TooN::Vector<2> v2ImgSize = TooN::makeVector(irImageSize.width, irImageSize.height);
		TooN::Vector<9>& v9Params = mParams[camName];
		std::cout << camName << ": size: " << v2ImgSize << std::endl << "Parameters: " << v9Params << std::endl;
		
		mmCameraModels.insert(std::pair<std::string, CameraModel>(camName, CameraModel(v9Params, v2ImgSize)));
	}
	
	ImageProcess imgProc;

	CameraMatrixMap cmm;
	Corner2DMap vPixelCorners;
	Corner3DMap vNewCorners;
	imgProc.FindCameraMatrix(mmCameraModels, mImageSizes, vPixelCorners, vNewCorners, cmm);
	
	ProjectMap mapX, mapY;
	imgProc.FindUndistortMapping(mmCameraModels, cmm, vNewCorners, mapX, mapY);
		
    ImageWindow mWindow;
    ImageBWMap imagesBW;
    ImageBWMap imagesNew;
    
    while(ros::ok())
    {	
		imagesBW.clear();
		imagesNew.clear();
		
        bool b = CamSub.GetAndFillFrameBW(imagesBW);
        
        //imagesNew = imagesBW;
        
        imgProc.RemapToPerspective(imagesBW, imagesNew, mapX, mapY);
        
        //imagesNew.erase("camera1");
        //imagesNew.erase("camera2");
        //imagesNew.erase("camera4");

        mWindow.DrawFrame(imagesNew);
		
        ros::spinOnce();
        loop_rate.sleep();
    }
    
    return 0;    
}
