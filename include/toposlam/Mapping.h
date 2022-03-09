#ifndef MAPPING_
#define MAPPING_

#include <TooN/TooN.h>
#include <TooN/se3.h>
#include <TooN/so3.h>
#include <iostream>
#include <iterator>     // std::distance
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/nonfree/features2d.hpp>
#include <toposlam/MultiCameraSubscriber.h>
#include <toposlam/ImageWindow.h>
#include <toposlam/ImageProcess.h>
#include <toposlam/VocabTree.h>
#include <toposlam/TopoGraph.h>
#include <toposlam/Types.h>


//typedef std::map< std::string, std::vector<cv::KeyPoint> > KeyPointMap;
typedef std::map<std::string, CameraModel> TaylorCameraMap;
typedef unsigned int TI;
typedef VertexProperties<TI> TV;
typedef EdgeProperties TE;


//enum MappingMode
//{
	//MODE_DISPLAY,
	//MODE_MAPPING,
	//MODE_CLUSTERING
//};

class Mapping
{
public:
    Mapping();
    Mapping(MultiCameraSubscriber* pCamSub, ImageWindow* pWindow, ImageProcess* pImgProc, VocabTree* pVocTree, TopoGraph<TV, TE, TI>* pGraph, bool bToRectify = true);
    ~Mapping();
    
    void InitMapping();
    
    void Compute(ImageBWMap imagesBW);
    void Run();
    void Clustering();
    void SaveDescriptors(std::string filename);
    //void ORBExtractor(cv::Mat img_in, cv::Mat img_mask, std::vector<cv::KeyPoint>& keypoints, cv::Mat& descriptors, int nFeatures = 150);
    //void FASTExtractor(cv::Mat img_in, cv::Mat img_mask, std::vector<cv::KeyPoint>& keypoints, cv::Mat& descriptors, int threshold = 50);
    //void DrawFrame(ImageBWMap imagesBW, KeyPointMap keypoint);
    //void DrawFrame(ImageBWMap imagesBW);
    //double OverlapRatio(int width, int height, cv::Mat homography);
    bool isKeyframe();
    
    void DecomposeEssential(const TooN::Matrix<3,3>& E, TooN::Matrix<3,3>& R1, TooN::Matrix<3,3>& R2, TooN::Vector<3>& b);
    void SVDEssential(const TooN::Matrix<3,3>& E, std::vector< TooN::Vector<3> >& kpused_last, std::vector< TooN::Vector<3> >& kpused, TooN::Matrix<3,3>& R, TooN::Vector<3>& b);
	
    //void CreateMasks();
    void SetMasks(ImageBWMap& maskMap);
    void SetCamMatInv(CameraMatrixMap mCamMat);
    
    void SaveImages(ImageBWMap imagesBW);
    
    void StereoSetup(std::string camA, std::string camB);
    void SetStereoNames(std::vector<std::string> vStereoNames);
    
    cv::Mat Stereo(std::string camA, std::string camB);
    void RecordReferencePose();
    
    //2D-2D motion estimation from 2D-2D coerespondences
    void getEssential(const std::vector<cv::Point2f>& vPoints_1, const std::vector<cv::Point2f>& vPoints_2, const TooN::Matrix<3> camMat_1, const TooN::Matrix<3> camMat_2, TooN::Matrix<3>& E, cv::Mat& mPointStatus);
    bool getRT(const TooN::Matrix<3>& E, std::vector< TooN::Matrix<3> > & R, TooN::Vector<3>& t);
    bool isPointInFront(const TooN::Matrix<3> R, const TooN::Vector<3> t, const TooN::Vector<3>  p1, const TooN::Vector<3> p2);
    int VerifyRT(const TooN::Matrix<3> R, const TooN::Vector<3> t, const std::vector< TooN::Vector<3> > imgPoints_1, const std::vector< TooN::Vector<3> > imgPoints_2);
    
private:
	unsigned m_nCameraNumber;
	
	std::vector<std::string> m_vStereoNames;

	KeyPointMap m_KeyPointsToDraw;
    KeyPointMap m_KeyPointList;
    KeyPointMap m_KeyPointList_Last;
    ImageBWMap m_DescriptorList;
    ImageBWMap m_DescriptorList_Last;
    ImageBWMap m_mMasks;
    
    std::map< std::string, std::vector<cv::Mat> > m_mImageHistory;
    
    cv::Mat m_DescriptorPool;
    cv::Mat m_DescriptorIndex;
    
    int m_nTotalMatches;
    int m_nDescriptors;
    int m_nDescriptors_Last;
    TI m_nKeyframeIndex;

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
	ImageBWMap m_mStereoPoses; // the pose of the stereo cameras, (transform from reference frame (camera 1) to image frame)
	//TooN::SE3<>  m_se3IMUPose;
	
	//MappingMode m_eMode;
	geometry_msgs::Pose m_ReferencePose;
	geometry_msgs::Pose m_ReferencePose_Last;
	TooN::Vector<3> m_v3ReferencePosition;
	TooN::Vector<3> m_v3ReferencePosition_Last;
	cv::Mat m_mPoseRecord;
};

#endif
