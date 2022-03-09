#ifndef IMAGEPROCESS_
#define IMAGEPROCESS_

#include <iostream>
#include <vector>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/nonfree/features2d.hpp>
#include <opencv2/nonfree/nonfree.hpp>
#include <toposlam/CameraModel.h>
#include <TooN/TooN.h>
#include <toposlam/Types.h>

typedef std::map<std::string, CameraModel> TaylorCameraMap;
//typedef std::map<std::string, TooN::Matrix<4, 3> > Corner3DMap;
//typedef std::map<std::string, TooN::Matrix<4, 2> > Corner2DMap;
//typedef std::map<std::string, TooN::Matrix<3, 3> > CameraMatrixMap;
//typedef std::map<std::string, cv::Mat> ProjectMap;
//typedef ProjectMap ImageBWMap;

class ImageProcess
{
public:
	ImageProcess();
	ImageProcess(TaylorCameraMap CameraModels, std::string strDetector = "CENSURE", std::string strDescriptor = "SIFT", int nFeatureNumber = 200, int nThreshold = 70);
	~ImageProcess();
	
	void InitUndistort();
	
	template <typename ImageMap>
	void RemapToPerspective(ImageMap& imgIN, ImageMap& imgOut, ProjectMap& mapX, ProjectMap& mapY)
	{
		imgOut = imgIN;
	
		for(ProjectMap::iterator it = mapX.begin(); it != mapX.end(); ++it)
		{
			//std::cout << "remapping" << std::endl;
			cv::Mat src = imgIN.at(it->first);
			cv::Mat map_x = mapX.at(it->first);
			cv::Mat map_y = mapY.at(it->first);
			
			cv::Mat dst;
			
			cv::remap(src, dst, map_x, map_y, CV_INTER_LINEAR, cv::BORDER_CONSTANT, cv::Scalar(0,0, 0));
			
			imgOut[it->first].create(dst.size(), dst.type());
			
			dst.copyTo(imgOut[it->first]);
		}
	}
	
	template <typename ImageMap>
	void RemapToPerspective(ImageMap& imgIN, ImageMap& imgOut)
	{
		RemapToPerspective(imgIN, imgOut, m_mMapX, m_mMapY);
	}
	
	void FindCameraMatrix(ImageSizeMap mImageSizes, Corner2DMap& vPixelCorners, Corner3DMap& vImgCorners, CameraMatrixMap& camMatMap);
	void FindCameraMatrix(TaylorCameraMap cameraModel, ImageSizeMap mImageSizes, Corner2DMap& vPixelCorners, Corner3DMap& vImgCorners, CameraMatrixMap& camMatMap);
	
	void FindUndistortMapping(CameraMatrixMap camMatMap, Corner3DMap corners, ProjectMap& mapx, ProjectMap& mapy);
	void FindUndistortMapping(TaylorCameraMap cameraModel, CameraMatrixMap camMatMap, Corner3DMap corners, ProjectMap& mapx, ProjectMap& mapy);
	
	void FASTExtractor(cv::Mat img_in, cv::Mat img_mask, std::vector<cv::KeyPoint>& keypoints, cv::Mat& descriptors, int nDesc = 200, int threshold = 50);
	
	void STARExtractor(cv::Mat img_in, cv::Mat img_mask, std::vector<cv::KeyPoint>& keypoints, cv::Mat& descriptors, int nDesc = 200, int threshold = 50);
	
	void FeatureExtractor(cv::Mat img_in, cv::Mat img_mask, std::vector<cv::KeyPoint>& keypoints, cv::Mat& descriptors);
	
	void FeatureMatching(const cv::Mat& queryDesc, const cv::Mat& trainDesc, std::vector< cv::DMatch >& goodMatch, double& dSimilarity);
	
	//std::vector< TooN::Vector<3> > Stereo(const cv::Mat& DescA, const cv::Mat& DescB, std::vector<cv::KeyPoint>& PointsA, std::vector<cv::KeyPoint>& PointsB, const TooN::Matrix<3,4>& PA,  const TooN::Matrix<3,4>& PB); //, const TooN::Matrix<3,3>& mCamMatInvA, const TooN::Matrix<3,3>& mCamMatInvB
	
	cv::Mat Stereo( cv::Mat& DescA, cv::Mat& DescB, std::vector<cv::KeyPoint>& PointsA, std::vector<cv::KeyPoint>& PointsB, const cv::Mat& PA,  const cv::Mat& PB); 
	
	bool Find3DPoint(const TooN::Matrix<3,3>& RA2B, const TooN::Vector<3>& tA2B, const TooN::Vector<3>& v3A, const TooN::Vector<3>& v3B, TooN::Vector<3>& Point3D);
	
	CameraMatrixMap GetCameraMatrix() const { return m_mCamMat; }
	
private:
	TaylorCameraMap m_mCameraModels; // list of camerea models
	CameraMatrixMap m_mCamMat; // list of camera matrices
	ImageSizeMap m_mImageSizes; // list of images' original size
	ProjectMap m_mMapX, m_mMapY;
	std::string m_strDetectorType;
	std::string m_strDescriptorType;
	int m_nFeatureNumber;
	int m_nFeatureThreshold;
	
	cv::AdjusterAdapter* m_pFeatureDector;
	cv::Ptr<cv::DescriptorExtractor> m_pDescriptorExtractor;
	cv::DynamicAdaptedFeatureDetector* m_pDynamicDetector;
};

#endif
