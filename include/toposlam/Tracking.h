#ifndef TRACKING_
#define TRACKING_

#include <iostream>
#include <TooN/TooN.h>
#include <TooN/se3.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/nonfree/features2d.hpp>
#include <toposlam/MultiCameraSubscriber.h>
#include <toposlam/VocabTree.h>
#include <toposlam/ImageProcess.h>
#include <toposlam/ImageWindow.h>
#include <toposlam/TopoGraph.h>
#include <toposlam/Types.h>


//typedef std::map< std::string, std::vector<cv::KeyPoint> > KeyPointMap;
typedef std::map<std::string, CameraModel> TaylorCameraMap;
typedef unsigned int TI;
typedef VertexProperties<TI> TV;
typedef EdgeProperties TE;


class Tracking
{
public:    
    Tracking();
    Tracking(MultiCameraSubscriber* pCamSub, ImageWindow* pWindow, ImageProcess* pImgProc, VocabTree* pVocTree, TopoGraph<TV, TE, TI>* pGraph, bool bToRectify = true);
    ~Tracking();
    
    void InitTracking();
    bool LoadDescriptors(std::string filename);
    //void FASTExtractor(cv::Mat img_in, cv::Mat img_mask, std::vector<cv::KeyPoint>& keypoints, cv::Mat& descriptors, int threshold = 50);
    double OverlapRatio(int width, int height, cv::Mat homography);
    void Run();
    void Compute(ImageBWMap imagesBW);
    void SetMasks(ImageBWMap& maskMap);
    void SetCamMatInv(CameraMatrixMap mCamMat);
    void SaveScore(std::string filename = "/home/mingfeng/Scores.dat");
    TooN::SE3<> Odometry(const cv::Mat& camMatrix, const cv::Mat& desc_1, const cv::Mat& Points, const cv::Mat& desc_2, std::vector< cv::KeyPoint>& keypoints_2);
    void SetStereoNames(std::vector<std::string> vStereoNames);

//////////
	int m_nTrueFrameID;
	int m_nMatchedFrameID;
	geometry_msgs::Pose m_MatchedFramePose;
	geometry_msgs::Pose m_OdometryPose;
	geometry_msgs::Pose m_ReferencePose;

private:    
	KeyPointMap m_KeyPointList;

    MultiCameraSubscriber* m_pCamSub;
    ImageWindow* m_pWindow;
    ImageProcess* m_pImgProc;
    VocabTree*  m_pVocTree; 
    TopoGraph<TV, TE, TI>* m_pGraph;
    bool m_bToRectify;
    
    TaylorCameraMap m_mCameraModels; // list of camerea models
	CameraMatrixMap m_mCamMat; // list of camera matrices
	CameraMatrixMap m_mCamMatInv; // list of the inverse of camera matrices
	SE3Map m_mPoses; // camera poses w.r.t. the first camera
	
	ImageBWMap m_mMasks;
	TooN::Vector<3> m_v3ReferencePosition;	

	
	cv::Mat m_mScoreRecord;
	cv::Mat m_mDistanceRecord;
	
	std::vector<std::string> m_vStereoNames;

};


#endif

