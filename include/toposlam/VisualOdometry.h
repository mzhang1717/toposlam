#ifndef __VISUAL_ODOMETRY__
#define __VISUAL_ODOMETRY__

#include <vector>
#include <map>
#include <iostream>
#include <iterator>     // std::distance

#include <TooN/TooN.h>
#include <TooN/se3.h>
#include <TooN/so3.h>

#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/nonfree/features2d.hpp>
#include <ctime>

class VisualOdometry
{
public:
	VisualOdometry();
	~VisualOdometry();
	
	// camera parameters (all are mandatory / need to be supplied)
	struct Calibration 
	{  
		double f;  // focal length (in pixels)
		double cu; // principal point (u-coordinate)
		double cv; // principal point (v-coordinate)
		TooN::Matrix<3> getK()// return camera matrix
		{
			TooN::Matrix<3> K = TooN::Zeros(3,3);
			K[0][0] = f; K[0][2] = cu;
			K[1][1] = f; K[1][2] = cv;
			K[2][2] = 1.0;
			
			return K;
		};

		cv::Mat getKMat()// return camera matrix
		{
			cv::Mat K  = (cv::Mat_<double>(3,3) << f, 0, cu, 0, f, cv, 0 , 0, 1.0);			
			return K;
		};
		
		Calibration() 
		{
			f  = 1;
			cu = 0;
			cv = 0;
		};
		
		Calibration(double _f, double _cu, double _cv): f(_f), cu(_cu), cv(_cv)
		{
		};
		
		Calibration(Calibration& calib)
		{
			f = calib.f;
			cu = calib.cu;
			cv = calib.cv;
		};
	};
	
	struct Parameters
	{
		bool bReweighting;
		int ransac_iters;
		double inlier_threshold;
        Parameters()
        {
            bReweighting = true;
            ransac_iters = 200;
            inlier_threshold = 2.0;
        }
        
        Parameters(bool _bR, int _ransac_iters, int _inlier_threshold)
        :bReweighting(_bR), ransac_iters(_ransac_iters), inlier_threshold(_inlier_threshold)
        {
        }
	};
	
	struct CrossMatches
	{
	    cv::Point2f vPointLeftPrevious;
	    cv::Point2f vPointRightPrevious;
	    cv::Point2f vPointLeftCurrent;
	    cv::Point2f vPointRightCurrent;
	    TooN::Vector<3> v3dPointPrevious;
	};

	struct StereoMatches
	{
	    cv::Point2f vPointsLeft;
	    cv::Point2f vPointsRight;
		TooN::Vector<3> v3dPoint;
	    
	    StereoMatches(){};
	    StereoMatches(cv::Point2f vPL,  cv::Point2f vPR)
	    {
			vPointsLeft = vPL; 
			vPointsRight = vPR;
		}
        
        StereoMatches(cv::Point2f vPL,  cv::Point2f vPR, TooN::Vector<3> v3P)
	    {
			vPointsLeft = vPL; 
			vPointsRight = vPR;
            v3dPoint = v3P;
		}
	};
	
	struct StereoFeatures
	{
		cv::Mat mDescLeft;
	    cv::Mat mDescRight;	

	    StereoFeatures(){};
	    StereoFeatures(cv::Mat mDL, cv::Mat mDR)
	    {
			mDL.copyTo(mDescLeft);
			mDR.copyTo(mDescRight);
		};	     	
	};
    
    struct JMatSize
    {
        int nRows;
        int nCols;
    };
	
	enum UpdateStatus {UPDATED, CONVERGED, FAILED};
	
	VisualOdometry(VisualOdometry::Calibration _leftCalib, VisualOdometry::Calibration _rightCalib, TooN::SE3<> _se3LeftPose, TooN::SE3<> _se3RightPose,  Parameters _param);
	
	/// set matched keypoints from previous stereo view
	void setPreviousKeypoints(std::vector<cv::Point2f> vPointLeftPrevious, cv::Mat mFeaturesLeftPrevious, std::vector<cv::Point2f>  vPointRightPrevious, cv::Mat mFeaturesRightPrevious);
	
	/// set matched keypoints from current stereo view
	void setCurrentKeypoints(std::vector<cv::Point2f> vPointLeftCurrent, cv::Mat mFeaturesLeftCurrent, std::vector<cv::Point2f>  vPointRightCurrent, cv::Mat mFeaturesRightCurrent);
	
	/// replace previous keypoints by the current keypoints
	void replaceKeypoints();
    
    /// match feature points from two images
	void matchFeatures(const cv::Mat& queryDesc, const cv::Mat& trainDesc, std::vector< cv::DMatch >& goodMatch);
	
	/// main 
	void process();
    
    /// 
    TooN::SE3<> getMotion();
    
    int countInliers();
    int countMatches();
    
    /// calculate rotation matrix R from euler angles and d(R)/d(euler)
	void getRotMatAndDerivative(const double& rx, const double& ry, const double& rz, TooN::Matrix<3>& Rot, TooN::Matrix<3>& Rx, TooN::Matrix<3>& Ry, TooN::Matrix<3>& Rz);
	
    /// m = K * Xc/Xc(3)
	TooN::Vector<2> Project(const TooN::Vector<3>& v3dPoints, VisualOdometry::Calibration& camCalib);
  
    Calibration m_sLeftCalib, m_sRightCalib;            // camera calibration parameters
    std::vector<VisualOdometry::CrossMatches> m_vCrossMatches; // feature points matched across four images in two stereo views
    
private:
	std::vector<VisualOdometry::StereoMatches>  m_vMatchesPrevious, m_vMatchesCurrent; // matched key points in each stereo
	
	VisualOdometry::StereoFeatures m_mFeaturesPrevious, m_mFeaturesCurrent; //matched feature points in each stereo
 
	
	TooN::Vector<TooN::Resizable> m_vPointsObserved; // vector of stacked 2d image points --- observed from measurements
	TooN::Vector<TooN::Resizable> m_vPointsPredicted; // vector of stacked 2d image points --- predicted from projections of 3D points
	TooN::Vector<TooN::Resizable> m_vResidual; // = m_vPointsObserved - m_vPointsPredicted
	TooN::Vector<TooN::Resizable> m_vWeightedResidual; // = weighting * m_vResidual
	TooN::Matrix<3, 6> m_mJacobian; // d (m_vPointsPredicted) d ([R | t])
	double* m_pJacobianData;
    JMatSize m_sJacobianSize;
	std::vector< TooN::Vector<3> > m_v3dPoints; // 3D points in left camera frame
	std::vector<int>  m_vInliers;    // inlier set
	
    TooN::Vector<6> m_vTr; // frame to frame motion

	TooN::SE3<> m_se3LeftPose, m_se3RightPose; // left and right camera poses
	TooN::SE3<> m_se3TL2R; //  transformation from left to right camera in a stereo
	Parameters m_sParam; // parameters
	
	cv::Mat m_mProjLeft, m_mProjRight; // projection matrices of left and right camera
	
	/// store Jacobian (as a TooN::Matrix) into an array
	void storeJacobianMatrix(TooN::Matrix<> mJacobian);
	
	/// retrieve Jacobian matrix into a TooN::Matrix from an array
	TooN::Matrix<> retrieveJacobianMatrix();
	
	/// match keypoints between previous and current views
	void crossMatching();

	/// stack all 2D image points from stereo into one single column vector
	void computeObservations(std::vector<VisualOdometry::CrossMatches> &p_matched, std::vector<int> &active); 
	
	/// J = d(X)/d(tr), X_tilte = X_observed - X_predicted
	void computeResidualsAndJacobian(TooN::Vector<6> &tr,std::vector<int> &active); 
	
	///
	void computeResiduals(TooN::Vector<6> &tr,std::vector<int> &active); 
	
	/// estimate the egomotion through RANSAC
	TooN::Vector<6> estimateMotion (std::vector<VisualOdometry::CrossMatches> &p_matched); 
	
	/// solve J * dp = Dx
	VisualOdometry::UpdateStatus updateParameters(std::vector<VisualOdometry::CrossMatches> &p_matched, std::vector<int> &active, TooN::Vector<6> &tr, double step_size, double eps);
	
	/// calculate the derivate of 3D to 2D homogeneous transformation, d (Xc/Xc(3)) /d (Xc)
	TooN::Matrix<2,3> getHomoDerivative(TooN::Vector<3>& v3dPoints);
	
	TooN::Matrix<3> getRotation(const double& rx, const double& ry, const double& rz);
	
	/// calculate the 3D coordinate of a point using its 2d projections on a pair of stereo images
	std::vector< TooN::Vector<3> > calTriangulation(cv::InputArray vPLeft, cv::InputArray vPRight);
			
	/// randomly select nSample points from nTotal points, return the indices of the selected points
	std::vector<int> getRandomSamples(const int nTotal, const int nSample);
	
	/// find the indices of the inliner matches 
	std::vector<int> getInlier(std::vector<VisualOdometry::CrossMatches> &p_matched, TooN::Vector<6> &tr); 
};

#endif
