#include <toposlam/VisualOdometry.h>
#include <assert.h>     /* assert */
#include <TooN/SVD.h>


VisualOdometry::VisualOdometry()
{
	srand (time(NULL)); /* initialize random seed: */
	
	m_pJacobianData = NULL;
}

VisualOdometry::VisualOdometry(VisualOdometry::Calibration _leftCalib, VisualOdometry::Calibration _rightCalib, TooN::SE3<> _se3LeftPose, TooN::SE3<> _se3RightPose,  Parameters _param)
:m_sLeftCalib(_leftCalib), m_sRightCalib(_rightCalib), m_se3LeftPose(_se3LeftPose), m_se3RightPose(_se3RightPose), m_sParam(_param) 
{
	srand (time(NULL)); /* initialize random seed: */
	
	m_pJacobianData = NULL;
    
    // generate the projection matrix (3x4) of the left camera
    TooN::Matrix<> rot = m_se3LeftPose.inverse().get_rotation().get_matrix();
	TooN::Vector<> tran = m_se3LeftPose.inverse().get_translation();
	m_mProjLeft = m_sLeftCalib.getKMat() * (cv::Mat_<double>(3,4) << rot(0,0), rot(0,1), rot(0,2), tran[0], rot(1,0), rot(1,1), rot(1,2), tran[1], rot(2,0), rot(2,1), rot(2,2), tran[2]);
    
    std::cout << "Left ProjMat: " << std::endl << m_mProjLeft << std::endl;
    
    // generate the projection matrix (3x4) of the right camera
    rot = m_se3RightPose.inverse().get_rotation().get_matrix();
	tran = m_se3RightPose.inverse().get_translation();
    m_mProjRight = m_sRightCalib.getKMat() * (cv::Mat_<double>(3,4) << rot(0,0), rot(0,1), rot(0,2), tran[0], rot(1,0), rot(1,1), rot(1,2), tran[1], rot(2,0), rot(2,1), rot(2,2), tran[2]);

    std::cout << "Right ProjMat: " << std::endl << m_mProjRight << std::endl;
    
    // calculate the transformation from left camera to right camera, p_r = m_se3TL2R * p_l
    m_se3TL2R = m_se3RightPose.inverse() * m_se3LeftPose;
    
    std::cout << "Relative camera pose: " << std::endl << m_se3TL2R << std::endl;
}

VisualOdometry::~VisualOdometry()
{	
	if(!m_pJacobianData)
		delete[] m_pJacobianData;
}
		
/// set matched keypoints and features from previous view
void VisualOdometry::setPreviousKeypoints(std::vector<cv::Point2f> vPointLeftPrevious, cv::Mat mFeaturesLeftPrevious, std::vector<cv::Point2f>  vPointRightPrevious, cv::Mat mFeaturesRightPrevious)
{
	assert(vPointLeftPrevious.size() == vPointRightPrevious.size()); // keypoints from left and right cameras are matched, so the sizes must be equal
	
	m_vMatchesPrevious.clear(); // clear the struct that stores all matched keypoints from previous view
    
    std::vector<TooN::Vector<3> > v3Ps = calTriangulation(vPointLeftPrevious, vPointRightPrevious); // calculate 3D points through triangulating each pair of matched points
	
    cv::Mat mFeatureLeftSelected, mFeatureRightSelected;
    
	for (size_t i = 0; i < vPointLeftPrevious.size(); i++)
	{
        if(v3Ps[i][2] > 0 && v3Ps[i][2] < 1000) // only keep 3D points that are within reasonable range
        {
            m_vMatchesPrevious.push_back(VisualOdometry::StereoMatches(vPointLeftPrevious[i], vPointRightPrevious[i], v3Ps[i]));

            if(mFeatureLeftSelected.empty())
                mFeaturesLeftPrevious.row(i).copyTo(mFeatureLeftSelected);
            else
                mFeatureLeftSelected.push_back(mFeaturesLeftPrevious.row(i));
                
            if(mFeatureRightSelected.empty())
                mFeaturesRightPrevious.row(i).copyTo(mFeatureRightSelected);
            else
                mFeatureRightSelected.push_back(mFeaturesRightPrevious.row(i));              
        }
	}
	
	m_mFeaturesPrevious = VisualOdometry::StereoFeatures(mFeatureLeftSelected, mFeatureRightSelected); // store valid, matched features from previous view  
}
	
/// set matched keypoints and features from current view
void VisualOdometry::setCurrentKeypoints(std::vector<cv::Point2f> vPointLeftCurrent, cv::Mat mFeaturesLeftCurrent, std::vector<cv::Point2f>  vPointRightCurrent, cv::Mat mFeaturesRightCurrent)
{
	assert(vPointLeftCurrent.size() == vPointRightCurrent.size()); // keypoints from left and right cameras are matched, so the sizes must be equal
	
	m_vMatchesCurrent.clear(); // clear the struct that stores all matched keypoints from current view
    
    std::vector<TooN::Vector<3> > v3Ps = calTriangulation(vPointLeftCurrent, vPointRightCurrent); // calculate 3D points through triangulating each pair of matched points
   
    cv::Mat mFeatureLeftSelected, mFeatureRightSelected;
    
	for(size_t i = 0; i < vPointLeftCurrent.size(); i++)
	{
        if(v3Ps[i][2]>0 && v3Ps[i][2] < 1000) // only keep 3D points that are within reasonable range
        {
            m_vMatchesCurrent.push_back(VisualOdometry::StereoMatches(vPointLeftCurrent[i], vPointRightCurrent[i], v3Ps[i]));
            
            if(mFeatureLeftSelected.empty())
                mFeaturesLeftCurrent.row(i).copyTo(mFeatureLeftSelected);
            else
                mFeatureLeftSelected.push_back(mFeaturesLeftCurrent.row(i));
                
            if(mFeatureRightSelected.empty())
                mFeaturesRightCurrent.row(i).copyTo(mFeatureRightSelected);
            else
                mFeatureRightSelected.push_back(mFeaturesRightCurrent.row(i));     
        }
	}	
	
	m_mFeaturesCurrent = VisualOdometry::StereoFeatures(mFeatureLeftSelected, mFeaturesRightCurrent); // store valid, matched features from current view  
    
    //for(size_t i = 0; i < m_vMatchesCurrent.size(); i++)
    //{
        //std::cout << "Stereo matches [" << i << "]: " << std::endl;
        //std::cout << "   -Left Point:  " << m_vMatchesCurrent[i].vPointsLeft.x << ", " << m_vMatchesCurrent[i].vPointsLeft.y  << ", ";
        //std::cout << "   -Right Point: " << m_vMatchesCurrent[i].vPointsRight.x << ", " << m_vMatchesCurrent[i].vPointsRight.y << ", ";
        //std::cout << "   -3D Point:    " << m_vMatchesCurrent[i].v3dPoint << std::endl;
    //}
}	

/// replace the previous keypoints and features by the current ones
void VisualOdometry::replaceKeypoints()
{
	m_vMatchesPrevious.clear();
	m_vMatchesPrevious = m_vMatchesCurrent;
	m_mFeaturesPrevious = m_mFeaturesCurrent;
}

/// temporal matching of keypoints from two consecutive images of the left camera
void VisualOdometry::crossMatching()
{
	std::vector< cv::DMatch > vLeftMatches; //, vRightMatches;
	
	// match left-to-left features
	matchFeatures(m_mFeaturesPrevious.mDescLeft, m_mFeaturesCurrent.mDescLeft, vLeftMatches);
	
    //std::cout << "Left matches size: " << vLeftMatches.size() << std::endl;
    
	// match right-to-right features
	//matchFeatures(m_mFeaturesPrevious.mDescRight, m_mFeaturesCurrent.mDescRight, vRightMatches);	
    
    m_vCrossMatches.clear();
    
	//std::map<int, int> mLeftIdx; 
	//std::map<int, int> mRightIdx;
	//std::map<int, int> mCrossIdx;
	
	for(size_t i = 0; i < vLeftMatches.size(); i++) 
	{
		//mLeftIdx[vLeftMatches[i].queryIdx] = vLeftMatches[i].trainIdx;
        
        //std::cout << vLeftMatches[i].queryIdx << " --- " << vLeftMatches[i].trainIdx << std::endl;
        
        VisualOdometry::CrossMatches quadMatch;
		quadMatch.vPointLeftPrevious = m_vMatchesPrevious[vLeftMatches[i].queryIdx].vPointsLeft;
		quadMatch.vPointRightPrevious = m_vMatchesPrevious[vLeftMatches[i].queryIdx].vPointsRight;
	    quadMatch.vPointLeftCurrent = m_vMatchesCurrent[vLeftMatches[i].trainIdx].vPointsLeft;
	    quadMatch.vPointRightCurrent = m_vMatchesCurrent[vLeftMatches[i].trainIdx].vPointsRight;
	    quadMatch.v3dPointPrevious = m_vMatchesPrevious[vLeftMatches[i].queryIdx].v3dPoint;
		m_vCrossMatches.push_back(quadMatch);
	}
}

/// perform feature matching using Flann/knn 
void VisualOdometry::matchFeatures(const cv::Mat& queryDesc, const cv::Mat& trainDesc, std::vector< cv::DMatch >& goodMatch)
{
	cv::FlannBasedMatcher FlannMatcher;
	
	std::vector< std::vector< cv::DMatch > > knnMatches;
	std::vector< cv::DMatch > matches_1;
	std::vector< cv::DMatch > matches_2;
	
	// matching: queryDesc -> trainDesc
	FlannMatcher.knnMatch(queryDesc, trainDesc, knnMatches, 2); // for each query descriptor, find two matching candidates from the train descriptor set
	
	for(std::vector< std::vector< cv::DMatch > >::iterator it = knnMatches.begin(); it != knnMatches.end(); ++it)
	{
		if(it->at(0).distance < (0.8*(it->at(1).distance))) // reject if the two matching candidates are very similar to the query feature
			matches_1.push_back(it->at(0));
	}
	
	// matching: trainDesc -> queryDesc
	knnMatches.clear();
	FlannMatcher.knnMatch(trainDesc, queryDesc, knnMatches, 2); // for each Train descriptor, find two matching candidates from the Query descriptor set
	
	for(std::vector< std::vector< cv::DMatch > >::iterator it = knnMatches.begin(); it != knnMatches.end(); ++it)
	{
		if(it->at(0).distance < (0.8*(it->at(1).distance))) // reject if the two matching candidates are very similar to the query feature
			matches_2.push_back(it->at(0));
	}	
	
	// only accept candidates that are matched in both orders
	for(std::vector< cv::DMatch >::iterator it_1 = matches_1.begin(); it_1 != matches_1.end(); ++it_1)
	{
		for(std::vector< cv::DMatch >::iterator it_2 = matches_2.begin(); it_2 != matches_2.end(); ++it_2)
		{
			if ((*it_1).queryIdx == (*it_2).trainIdx && (*it_2).queryIdx == (*it_1).trainIdx) 
			{
				// add symmetrical match
				goodMatch.push_back(
						cv::DMatch((*it_1).queryIdx,
						(*it_1).trainIdx,
						(*it_1).distance));
				break; // next match in image 1 -> image 2
			}
		}
	}
}


/// perform VO computation
void VisualOdometry::process()
{
    //std::cout << "Size_previous: " << m_vMatchesPrevious.size() << std::endl;
    //std::cout << "Size_current: " <<  m_vMatchesCurrent.size() << std::endl;
    
    clock_t tStart = clock();
    
	crossMatching(); //matching keypoints between previous and current views
    
    clock_t tElapsed = clock() - tStart;
    
    double tCrossMatching = (double)tElapsed / (CLOCKS_PER_SEC / 1000.0);
    
   
     
    tStart = clock();
    
	m_vTr = estimateMotion (m_vCrossMatches); // estimate the pose transformation from previous view to current view, and store it in a 6x1 vector
	
    tElapsed = clock() - tStart;
    
    double tRansac = (double)tElapsed / (CLOCKS_PER_SEC / 1000.0);
    
    std::cout << "Matches: " <<  m_vCrossMatches.size() ;
     
    std::cout << ", Inlier: " << countInliers() << ", Inlier rate: " << countInliers()*100.0/m_vCrossMatches.size() << "%" << std::endl;
    
    std::cout << "Cross matching time: " << tCrossMatching << " , RANSAC time: " << tRansac << std::endl;
    std::cout << "VO: " << m_vTr << std::endl;
}

/// generate an SE3 object (representing ego motion of the left camera) from Euler angles and translation
TooN::SE3<> VisualOdometry::getMotion()
{
    // extract motion parameters
    double rx = m_vTr[0]; double ry = m_vTr[1]; double rz = m_vTr[2];
    double tx = m_vTr[3]; double ty = m_vTr[4]; double tz = m_vTr[5];
    
    TooN::Matrix<3> Rot = getRotation(rx, ry, rz);
    TooN::Vector<3> Tran = TooN::makeVector(tx, ty, tz);
    
    TooN::SE3<> se3Tr(TooN::SO3<>(Rot), Tran); 
    
    return se3Tr;
}

/// select a random subset from a point set
std::vector<int> VisualOdometry::getRandomSamples(const int nTotal, const int nSample)
{
	assert(nTotal > nSample);
	
	// init sample and totalset
	std::vector<int> sample;
	std::vector<int> totalset;
	
	// create vector containing all indices
	for (int i = 0; i < nTotal; i++)
		totalset.push_back(i);
	
	// add nSample indices to current sample
	sample.clear();
	for (int i=0; i < nSample; i++) 
	{
		int j = rand()%totalset.size();
		sample.push_back(totalset[j]);
		totalset.erase(totalset.begin()+j);
	}
	
	// return sample
	return sample;	
}

// slove J*dp = e, and update pose hypothesis: p + step_size -> p
VisualOdometry::UpdateStatus VisualOdometry::updateParameters(std::vector<VisualOdometry::CrossMatches> &p_matched, std::vector<int> &active, TooN::Vector<6> &tr, double step_size, double eps)
{
    // we need at least 3 observations
    if (active.size()<3)
        return FAILED;
                
    std::clock_t tStart = std::clock();  
        
    computeObservations(p_matched, active);
    computeResidualsAndJacobian(tr, active);    

	TooN::Matrix<> mJacobian =  retrieveJacobianMatrix();
     
	TooN::SVD<> svdJ(mJacobian.T() * mJacobian);
	TooN::Vector<6> dp = svdJ.backsub(mJacobian.T() * m_vWeightedResidual);
    
    std::clock_t tElapsed =  std::clock() - tStart;
    
    double tSolver = (double) tElapsed / (CLOCKS_PER_SEC / 1000.0);
    
    if(tSolver > 0)
		std::cout << "Solver: " << tSolver << " ms"   << std::endl;
    
	tr += step_size * dp; // update pose hypothesis
 
	bool converged = true;
	for (int m = 0; m < 6; m++) 
	{
		if (fabs(dp[m] > eps))
		{
			converged = false;
			break;
		}
	}
	
	if (converged)
	  return CONVERGED;
	else
	  return UPDATED;
	//} 
	//else 
	//{
		//return FAILED;
	//}	 
}

/// find inlier points
std::vector<int> VisualOdometry::getInlier(std::vector<VisualOdometry::CrossMatches> &p_matched, TooN::Vector<6> &tr) 
{

	clock_t tS = clock();
	
	// mark all observations active
	std::vector<int> active;
	for (size_t i=0; i < p_matched.size(); i++)
		active.push_back(i);
	
	// extract observations and compute predictions
	computeObservations(p_matched, active);
	computeResiduals(tr, active);
	
	clock_t tE = clock() - tS;
	double tInlier = (double) tE / (CLOCKS_PER_SEC / 1000.0);
	
	if(tInlier > 0)
		std::cout << "                getInlier(): " << tInlier << std::endl;  
  
	// find inliers
	std::vector<int> inliers;
	
	for (size_t i=0; i < p_matched.size(); i++)
	{
		if (TooN::norm(m_vResidual.slice(4*i, 4)) < m_sParam.inlier_threshold)
			inliers.push_back(i);
	}

	return inliers;
}

/// put active (selected) observed points in a single column vector
void VisualOdometry::computeObservations(std::vector<VisualOdometry::CrossMatches> &p_matched, std::vector<int> &active) 
{
    m_vPointsObserved.resize(active.size()*4);

    // set all observations
    for (size_t i = 0; i < active.size(); i++) 
    {
        m_vPointsObserved[4*i+0] = p_matched[active[i]].vPointLeftCurrent.x; // u_left
        m_vPointsObserved[4*i+1] = p_matched[active[i]].vPointLeftCurrent.y; // v_left
        m_vPointsObserved[4*i+2] = p_matched[active[i]].vPointRightCurrent.x; // u_right
        m_vPointsObserved[4*i+3] = p_matched[active[i]].vPointRightCurrent.y; // v_right
    }
}

/// calculate rotation matrix from Euler angles
inline TooN::Matrix<3> VisualOdometry::getRotation(const double& rx, const double& ry, const double& rz)
{
    // precompute sine/cosine
    double sx = sin(rx); double cx = cos(rx); double sy = sin(ry);
    double cy = cos(ry); double sz = sin(rz); double cz = cos(rz);
    
	TooN::Matrix<3> Rot;
    
    // compute rotation matrix R
	Rot[0][0]    = +cy*cz;          Rot[0][1]    = cy*sz;          Rot[0][2]    = -sy;
	Rot[1][0]    = +sx*sy*cz-cx*sz; Rot[1][1]   = sx*sy*sz+cx*cz;  Rot[1][2]   = sx*cy;
	Rot[2][0]    = cx*sy*cz+sx*sz;  Rot[2][1]    = +cx*sy*sz-sx*cz; Rot[2][2]    = +cx*cy; 
    
    return Rot;   
}

/// compute rotation matrix from Euler angles and its derivatives w.r.t to Euler angles
inline void VisualOdometry::getRotMatAndDerivative(const double& rx, const double& ry, const double& rz, TooN::Matrix<3>& Rot, TooN::Matrix<3>& Rx, TooN::Matrix<3>& Ry, TooN::Matrix<3>& Rz)
{
    // precompute sine/cosine
    double sx = sin(rx); double cx = cos(rx); double sy = sin(ry);
    double cy = cos(ry); double sz = sin(rz); double cz = cos(rz);
	
    // compute rotation matrix R
	Rot[0][0]    = +cy*cz;          Rot[0][1]    = cy*sz;          Rot[0][2]    = -sy;
	Rot[1][0]    = +sx*sy*cz-cx*sz; Rot[1][1]   = sx*sy*sz+cx*cz;  Rot[1][2]   = sx*cy;
	Rot[2][0]    = cx*sy*cz+sx*sz;  Rot[2][1]    = +cx*sy*sz-sx*cz; Rot[2][2]    = +cx*cy;
	
	// compute dR/drx
	Rx =  TooN::Zeros(3,3);      
	Rx[1][0] = +cx*sy*cz+sx*sz; Rx[1][1] = cx*sy*sz-sx*cz; Rx[1][2] = cx*cy;
	Rx[2][0] = -sx*sy*cz+cx*sz; Rx[2][1] = -sx*sy*sz-cx*cz; Rx[2][2] = -sx*cy;
	
	// compute dR/dry
     Ry[0][0] = -sy*cz;           Ry[0][1] = -sy*sz;           Ry[0][2] = -cy;
     Ry[1][0] = +sx*cy*cz;        Ry[1][1] = +sx*cy*sz;        Ry[1][2] = -sx*sy;
     Ry[2][0] = cx*cy*cz;        Ry[2][1] = +cx*cy*sz;        Ry[2][2] = -cx*sy;
	
	// compute dR/drz
	Rz =  TooN::Zeros(3,3);
	Rz[0][0] = -cy*sz;          Rz[0][1] = cy*cz;
	Rz[1][0] = -sx*sy*sz-cx*cz; Rz[1][1] = sx*sy*cz-cx*sz;
	Rz[2][0] = -cx*sy*sz+sx*cz; Rz[2][1] = +cx*sy*cz+sx*sz;
    
	//// compute rotation matrix R
	//Rot[0][0]    = +cy*cz;          Rot[0][1]    = -cy*sz;          Rot[0][2]    = +sy;
	//Rot[1][0]    = +sx*sy*cz+cx*sz; Rot[1][1]   = -sx*sy*sz+cx*cz;  Rot[1][2]   = -sx*cy;
	//Rot[2][0]    = -cx*sy*cz+sx*sz; Rot[2][1]    = +cx*sy*sz+sx*cz; Rot[2][2]    = +cx*cy;
	
	//// compute dR/drx
	//Rx =  TooN::Zeros(3,3);      
	//Rx[1][0] = +cx*sy*cz-sx*sz; Rx[1][1] = -cx*sy*sz-sx*cz; Rx[1][2] = -cx*cy;
	//Rx[2][0] = +sx*sy*cz+cx*sz; Rx[2][1] = -sx*sy*sz+cx*cz; Rx[2][2] = -sx*cy;
	
	////// compute dR/dry
    //Ry[0][0] = -sy*cz;           Ry[0][1] = +sy*sz;           Ry[0][2] = +cy;
    //Ry[1][0] = +sx*cy*cz;        Ry[1][1] = -sx*cy*sz;        Ry[1][2] = +sx*sy;
    //Ry[2][0] = -cx*cy*cz;        Ry[2][1] = +cx*cy*sz;        Ry[2][2] = -cx*sy;
	
	////// compute dR/drz
	//Rz =  TooN::Zeros(3,3);
	//Rz[0][0] = -cy*sz;          Rz[0][1] = -cy*cz;
	//Rz[1][0] = -sx*sy*sz+cx*cz; Rz[1][1] = -sx*sy*cz-cx*sz;
	//Rz[2][0] = +cx*sy*sz+sx*cz; Rz[2][1] = +cx*sy*cz-sx*sz;
}

/// compute the derivate of a 2D image point w.r.t its 3D coordinate in the camera frame
inline TooN::Matrix<2,3> VisualOdometry::getHomoDerivative(TooN::Vector<3>& v3dPoints)
{
	double Z = v3dPoints[2];
	//assert(Z>0);
	
	double X = v3dPoints[0];
	double Y = v3dPoints[1];
	
	TooN::Matrix<2,3> der =TooN::Data(1.0/Z, 0, -X/(Z*Z), 0, 1.0/Z, -Y/(Z*Z));
	
	return der;
}

/// project a 3D point in camera frame to image plane
inline TooN::Vector<2> VisualOdometry::Project(const TooN::Vector<3>& v3dPoints, VisualOdometry::Calibration& camCalib)
{
	//assert(v3dPoints[2]>0);

	TooN::Vector<3> v3ImgPoint = camCalib.getK() * v3dPoints / v3dPoints[2];
	TooN::Vector<2> v2ImgPoint = v3ImgPoint.slice(0,2); 
	
	return v2ImgPoint;
}

/// compute residual between observed and predicted image points, and Jacobian matrix
void VisualOdometry::computeResidualsAndJacobian(TooN::Vector<6> &tr, std::vector<int> &active) 
{
	timespec tStart, tEnd;
	
    // extract motion parameters
    double rx = tr[0]; double ry = tr[1]; double rz = tr[2];
    double tx = tr[3]; double ty = tr[4]; double tz = tr[5];
    
    // compute rotation matrix from previous frame to current frame, and its derivatives w.r.t. euler angles (rx, ry, rz)
    TooN::Matrix<3>  Rot, Rx, Ry, Rz;
	getRotMatAndDerivative(rx, ry, rz, Rot, Rx, Ry, Rz);
    TooN::Vector<3>  Tran = TooN::makeVector(tx, ty, tz);
    
    // (guessed) SE3 pose transformation from previous (left) frame to current (left) frame
    TooN::SE3<> se3TPrev2Curr(TooN::SO3<>(Rot), Tran);
    
    TooN::Matrix<> mJacobian = TooN::Zeros(active.size()*4, 6);
    //TooN::Vector<> mWeighting = TooN::Zeros(active.size()*4);
    
    m_vPointsPredicted.resize(active.size()*4);
    
    //clock_t tStart =  clock();
    
    TooN::Matrix<3,6> temp = TooN::Zeros(3, 6);
	temp.T()[3] = TooN::makeVector(1.0, 0, 0); 
	temp.T()[4] = TooN::makeVector(0, 1.0, 0); 
	temp.T()[5] = TooN::makeVector(0, 0, 1.0); 
    
    int nClock = clock_gettime(CLOCK_MONOTONIC, &tStart);       
    
    // for all observations do
    for (size_t i=0; i < active.size(); i++) 
    {
        // get 3d point in previous (left) coordinate system
        TooN::Vector<3>  v3PointInPrevious = m_vCrossMatches[active[i]].v3dPointPrevious;

        // predict 3d point in current left & right coordinate system using guessed transformation
        TooN::Vector<3> v3PointInCurrentLeft = se3TPrev2Curr * v3PointInPrevious;
        TooN::Vector<3> v3PointInCurrentRight = m_se3TL2R * v3PointInCurrentLeft;
        
        // weighting
        //if (m_sParam.bReweighting)
        //{
            //mWeighting[4*i] = 1.0/(fabs(m_vPointsObserved[4*i+0] - m_sLeftCalib.cu)/fabs(m_sLeftCalib.cu) + 0.05);
            //mWeighting[4*i+1] = 1.0/(fabs(m_vPointsObserved[4*i+1] - m_sLeftCalib.cv)/fabs(m_sLeftCalib.cv) + 0.05);
            //mWeighting[4*i+2] = 1.0/(fabs(m_vPointsObserved[4*i+2] - m_sRightCalib.cu)/fabs(m_sRightCalib.cu) + 0.05);
            //mWeighting[4*i+3] = 1.0/(fabs(m_vPointsObserved[4*i+3] - m_sRightCalib.cv)/fabs(m_sRightCalib.cv) + 0.05);
		//}
        
        temp.T()[0] = Rx * v3PointInPrevious;
        temp.T()[1] = Ry * v3PointInPrevious;
        temp.T()[2] = Rz * v3PointInPrevious;

        
        TooN::Matrix<2,3> der = getHomoDerivative(v3PointInCurrentLeft);
        mJacobian.slice(4*i, 0, 2, 6) = m_sLeftCalib.f*der*temp;
        
        der = getHomoDerivative(v3PointInCurrentRight);
        mJacobian.slice(4*i+2, 0, 2, 6) = m_sRightCalib.f*der*temp;
            
        TooN::Vector<2> v2ImgPointLeft = Project(v3PointInCurrentLeft, m_sLeftCalib);    
        TooN::Vector<2> v2ImgPointRight = Project(v3PointInCurrentRight, m_sRightCalib);    
        
        // compute prediction (project via K)
        m_vPointsPredicted.slice(4*i, 4)  = TooN::makeVector(v2ImgPointLeft[0], v2ImgPointLeft[1], v2ImgPointRight[0], v2ImgPointRight[1]); 
    }
    
    nClock = clock_gettime(CLOCK_MONOTONIC, &tEnd);  
    double  tSecond = (tEnd.tv_sec - tStart.tv_sec) * 1000.0;
    double  tNano = (tEnd.tv_nsec -  tStart.tv_nsec) * 1.0 / 1e6; 
    std::cout << "Jacobian: " << tSecond + tNano << " ms, points size: " <<  active.size() <<  std::endl;
        
	storeJacobianMatrix(mJacobian); // store the Jacobian matrix in an array. TooN::Matrix does not allow for dynamic allocation, so an array is used to bypass this issue.
    
    m_vResidual.resize(active.size()*4);
    m_vWeightedResidual.resize(active.size()*4);
    
    // compute residuals
	m_vResidual = m_vPointsObserved - m_vPointsPredicted;
	m_vWeightedResidual = m_vResidual; //mWeighting.as_diagonal()*m_vResidual;
}

/// compute residual between observed and predicted image points
void VisualOdometry::computeResiduals(TooN::Vector<6> &tr, std::vector<int> &active) 
{
	timespec tStart, tEnd;
	
    // extract motion parameters
    double rx = tr[0]; double ry = tr[1]; double rz = tr[2]; // eular angles
    double tx = tr[3]; double ty = tr[4]; double tz = tr[5]; // translation
    
    // compute rotation matrix from previous frame to current frame, and its derivatives w.r.t. euler angles (rx, ry, rz)
    TooN::Matrix<3>  Rot, Rx, Ry, Rz;
	getRotMatAndDerivative(rx, ry, rz, Rot, Rx, Ry, Rz);
    TooN::Vector<3>  Tran = TooN::makeVector(tx, ty, tz);
    
    // (guessed) SE3 pose transformation from previous (left) frame to current (left) frame
    TooN::SE3<> se3TPrev2Curr(TooN::SO3<>(Rot), Tran);
    
    m_vPointsPredicted.resize(active.size()*4);
    
    int nClock = clock_gettime(CLOCK_MONOTONIC, &tStart);       
    // for all observations do
    for (size_t i=0; i < active.size(); i++) 
    {
        // get 3d point in previous (left) coordinate system
        TooN::Vector<3>  v3PointInPrevious = m_vCrossMatches[active[i]].v3dPointPrevious;

        // predict 3d point in current left & right coordinate system using guessed transformation
        TooN::Vector<3> v3PointInCurrentLeft = se3TPrev2Curr * v3PointInPrevious;
        TooN::Vector<3> v3PointInCurrentRight = m_se3TL2R * v3PointInCurrentLeft;
        
        // compute prediction (project via K)    
        TooN::Vector<2> v2ImgPointLeft = Project(v3PointInCurrentLeft, m_sLeftCalib);    
        TooN::Vector<2> v2ImgPointRight = Project(v3PointInCurrentRight, m_sRightCalib);    
        
        m_vPointsPredicted.slice(4*i, 4)  = TooN::makeVector(v2ImgPointLeft[0], v2ImgPointLeft[1], v2ImgPointRight[0], v2ImgPointRight[1]);         
    }
    
    nClock = clock_gettime(CLOCK_MONOTONIC, &tEnd);  
    double  tSecond = (tEnd.tv_sec - tStart.tv_sec) * 1000.0;
    double  tNano = (tEnd.tv_nsec -  tStart.tv_nsec) * 1.0 / 1e6; 
    
    std::cout << "Residual: " << tSecond + tNano << " ms, points size: " <<  active.size() <<  std::endl;
    
    m_vResidual.resize(active.size()*4);
    m_vWeightedResidual.resize(active.size()*4);
    
    // compute residuals
	m_vResidual = m_vPointsObserved - m_vPointsPredicted;
	m_vWeightedResidual = m_vResidual; //mWeighting.as_diagonal()*m_vResidual;
}

/// store Jacobian (as a TooN::Matrix) into an array because TooN::Matrix does not permit dynamic allocation
inline void VisualOdometry::storeJacobianMatrix(TooN::Matrix<> mJacobian)
{
	// release the memory of the array
	if(!m_pJacobianData)
		delete[] m_pJacobianData;
        
    //std::cout << "J size: " << mJacobian.num_rows() << "x" << mJacobian.num_cols() << std::endl;
    
    // the size of the Jacobian matrix is stored in order to put the array back into a matrix
	m_sJacobianSize.nRows = mJacobian.num_rows();
    m_sJacobianSize.nCols = mJacobian.num_cols();
    
	m_pJacobianData = new double[mJacobian.num_rows() * mJacobian.num_cols()];  // re-allocate the array  
    
    for(int nRow = 0; nRow < mJacobian.num_rows(); nRow++)
    {
		for(int nCol = 0; nCol < mJacobian.num_cols(); nCol++)
		{
			m_pJacobianData[nRow * mJacobian.num_cols() + nCol] = mJacobian[nRow][nCol];
            
            //std:: cout << mJacobian[nRow][nCol] << " " ;
		}
        
        //std::cout << std::endl;
	}
}
	
/// retrieve Jacobian matrix into a TooN::Matrix from an array
inline TooN::Matrix<> VisualOdometry::retrieveJacobianMatrix()
{
    TooN::Matrix<TooN::Dynamic, TooN::Dynamic, double, TooN::Reference::RowMajor> mJacobian(m_pJacobianData, m_sJacobianSize.nRows, m_sJacobianSize.nCols);
    
    return mJacobian;
}

/// calculate 3D point (expressed in left camera frame) from its projections on left and right iamges
std::vector<TooN::Vector<3> > VisualOdometry::calTriangulation(cv::InputArray vPLeft, cv::InputArray vPRight)
{
	/// void triangulatePoints(InputArray projMatr1, InputArray projMatr2, InputArray projPoints1, InputArray projPoints2, OutputArray points4D)
	//projMatr1 – 3x4 projection matrix of the first camera.
	//projMatr2 – 3x4 projection matrix of the second camera.
	//projPoints1 – 2xN array of feature points in the first image. In case of c++ version it can be also a vector of feature points or two-channel matrix of size 1xN or Nx1.
	//projPoints2 – 2xN array of corresponding points in the second image. In case of c++ version it can be also a vector of feature points or two-channel matrix of size 1xN or Nx1.
	//points4D – 4xN array of reconstructed points in homogeneous coordinates.
    
	cv::Mat mPoint4D;
	cv::triangulatePoints(m_mProjLeft, m_mProjRight, vPLeft, vPRight, mPoint4D);
    
    //std::cout << "3D points: " << std::endl;
    //std::cout << mPoint4D << std::endl;
    
    for (int i = 0; i < mPoint4D.rows; i++)
	{
		mPoint4D.row(i) = mPoint4D.row(i).mul(1.0/mPoint4D.row(3));
	}
    
    //std::cout << "Mat depth: " << mPoint4D.depth() << std::endl;
    
    //std::cout << "3D points: " << std::endl;
    //std::cout << mPoint4D << std::endl;
	
	std::vector<TooN::Vector<3> > v3dPoints;
	
    //std::cout << "3D: " << std::endl;
    
	for (int colIdx = 0; colIdx < mPoint4D.cols; colIdx++)
	{
		TooN::Vector<3> v3P = TooN::makeVector(mPoint4D.at<float>(0, colIdx), mPoint4D.at<float>(1, colIdx), mPoint4D.at<float>(2, colIdx));
		//std::cout << v3P << ", " << mPoint4D.at<float>(2, colIdx)<< std::endl;
        v3dPoints.push_back(v3P);
	}
    
	return  v3dPoints;
}

/// estimate pose change through 3-point RANSAC
TooN::Vector<6> VisualOdometry::estimateMotion(std::vector<VisualOdometry::CrossMatches>& p_matched) 
{
	// return value
	bool bSuccess = true;
	
	//// compute minimum distance for RANSAC samples
	//double width=0, height=0;
	//for (std::vector<VisualOdometry::StereoMatches>::iterator it=p_matched.begin(); it!=p_matched.end(); ++it) 
	//{
		//if (it->vPointLeftPrevious.pt.x > width)  width  = it->vPointLeftPrevious.pt.x;
		//if (it->vPointLeftPrevious.pt.y > height) height = it->vPointLeftPrevious.pt.y;
	//}
	//double min_dist = min(width,height)/3.0;
	
	// get number of matches
	int N  = p_matched.size();
	
	if (N < 6)
	{
		TooN::Vector<6> tr_0 = TooN::Zeros(6);
		return tr_0;
	}
	
	// loop variables
	TooN::Vector<6> tr_delta = TooN::Zeros;
	
	int nBestIdx = 0;;
    
	// clear parameter vector
	m_vInliers.clear();
	
    clock_t tStart = clock();
    
	// initial RANSAC estimate
	for (int k=0; k < m_sParam.ransac_iters; k++) 
	{
		
		clock_t tS = clock();
		
        TooN::Vector<6> tr_delta_curr = TooN::Zeros;
        // std::cout  << "RANSAC step: " << k << std::endl;
        
		// draw random sample point set
		std::vector<int> active = getRandomSamples(N,3);
		
		// minimize reprojection errors
		VisualOdometry::UpdateStatus result = UPDATED;
		int iter = 0;
		while (result==UPDATED) 
		{
			result = updateParameters(p_matched, active, tr_delta_curr, 1.0, 1e-6);
			if ( ++iter > 20 || result==CONVERGED)
				break;
		}
        
       // std::cout << "iter = " << iter << std::endl;
		clock_t tE = clock() - tS;
		double tRonce = (double)tE / (CLOCKS_PER_SEC / 1000.0);
		
		clock_t tIs = clock();
		
		// overwrite best parameters if we have more inliers
		if (result != FAILED) 
		{
			std::vector<int> inliers_curr = getInlier(p_matched, tr_delta_curr);
			if (inliers_curr.size() > m_vInliers.size()) 
			{
				m_vInliers = inliers_curr;
				tr_delta = tr_delta_curr;
                nBestIdx = k;
			}
            
            //std::cout << "Inlier number = " << m_vInliers.size() << " from step " << nBestIdx << std::endl;
           // std::cout << "tr = " << tr_delta << std::endl;
		}
		
		clock_t tIe = clock() - tIs;
		
		double tgetInlier = (double)tIe / (CLOCKS_PER_SEC / 1000.0);

		if(tRonce > 0 || tgetInlier > 0)
			std::cout << "RANSAC[" << k << "]: " << tRonce << "ms, " << "Get_Inlier: " << tgetInlier << "ms"<< std::endl;
	}
    
    clock_t tElapsed = clock() - tStart;
    
    std::cout << "RANSAC_total: " << (double)tElapsed / (CLOCKS_PER_SEC / 1000.0) << std::endl;
    
	
	// final optimization (refinement)
	if (m_vInliers.size() >= 6) 
	{
		int iter = 0;
		VisualOdometry::UpdateStatus result = UPDATED;
		while (result==UPDATED) 
		{     
			result = updateParameters(p_matched, m_vInliers, tr_delta, 1.0,1e-8);
			if (iter++ > 100 || result==CONVERGED)
				break;
		}
		
		// not converged
		if (result!=CONVERGED)
			bSuccess = false;
	
	// not enough inliers
	} 
	else 
	{
		bSuccess = false;
	}
	
	// parameter estimate succeeded?
	if (bSuccess) 
		return tr_delta;
	else         
		return TooN::Zeros(6);
}

/// count the number of inliers
int VisualOdometry::countInliers()
{
    return m_vInliers.size();
}

int VisualOdometry::countMatches()
{
	m_vCrossMatches.size();
}
