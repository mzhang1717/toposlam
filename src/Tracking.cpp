#include <algorithm>
#include <vector>
#include <iterator>     // std::distance
#include <toposlam/Tracking.h>
#include <toposlam/Utility.h>
//#include <opencv2/opencv.hpp>

//TooN::Matrix<> camMat = m_mCamMat.at("camera3");
cv::Mat camMatA; ////= (cv::Mat_<double>(3,3) << camMat(0, 0), camMat(0, 1), camMat(0, 2),camMat(1, 0),camMat(1, 1),camMat(1, 2),camMat(2, 0),camMat(2, 1),camMat(2, 2));
cv::Mat Pose1 = (cv::Mat_<double>(3,4) << 0.999827, -0.0180458, 0.00447629, 0.431835, 0.014944, 0.636741, -0.770933, -0.0261652, 0.0110619, 0.770867, 0.6369, 0.0306548);

cv::Mat R0 = (cv::Mat_<double>(3,3) <<  0.9741, 0.0753, 0.2134, -0.1156, 0.9763, 0.1828, -0.1946, -0.2028, 0.9597);
cv::Mat R0inv = (cv::Mat_<double>(3,3) <<  0.9741, -0.1156, -0.1946, 0.0753, 0.9763, -0.2028, 0.2134, 0.1828, 0.9597);
cv::Mat t0 = (cv::Mat_<double>(3,1) << -0.062, -0.153, -0.084);


Tracking::Tracking()
{
    // Read image from camera/bagfiles
    const char *sCamNames[] = {"camera1", "camera2", "camera3", "camera4"};
    std::vector<std::string> vCameraNames(sCamNames, sCamNames+4);
    m_pCamSub = new MultiCameraSubscriber(vCameraNames);
    
    InitTracking();
    
    m_pWindow = new ImageWindow();
    
    m_pVocTree = new VocabTree(6, 10); // 6 levels x 10 branches, 10e6 leaf nodes 
    
    m_bToRectify = true;
}

Tracking::Tracking(MultiCameraSubscriber* pCamSub, ImageWindow* pWindow, ImageProcess* pImgProc, VocabTree* pVocTree, TopoGraph<TV, TE, TI>* pGraph, bool bToRectify)
: m_pCamSub(pCamSub), m_pWindow(pWindow), m_pImgProc(pImgProc), m_pVocTree(pVocTree), m_pGraph(pGraph), m_bToRectify(bToRectify)
{
	// Get the pose of each camera w.r.t. to the first camera
	m_mPoses.clear();
	m_mPoses = m_pCamSub->GetPoses();
	m_mCamMat =  m_pImgProc->GetCameraMatrix();
	SetCamMatInv(m_mCamMat);	
}

Tracking::~Tracking()
{
    if (m_pCamSub != NULL) { delete m_pCamSub; }
    
    if (m_pWindow != NULL) { delete m_pWindow; }
    
    if (m_pImgProc != NULL) { delete m_pImgProc; }
    
    if (m_pVocTree != NULL) { delete m_pVocTree; }
	
	if (m_pGraph != NULL) { delete m_pGraph; }
}

void Tracking::InitTracking()
{
	// Get the pose of each camera w.r.t. to the first camera
	m_mPoses.clear();
	m_mPoses = m_pCamSub->GetPoses();
	
    //InfoMap camInfo = m_pCamSub->GetInfo();
	
    // Create camera models here
    ImageSizeMap mImageSizes = m_pCamSub->GetSizes();
	ParamMap mParams = m_pCamSub->GetParams();    
	m_mCameraModels.clear(); 
	cv::Size irImageSize;
	for(ImageSizeMap::iterator it = mImageSizes.begin(); it != mImageSizes.end(); ++it)
	{
		std::string camName = it->first;
		
		irImageSize = mImageSizes[camName];
		TooN::Vector<2> v2ImgSize = TooN::makeVector(irImageSize.width, irImageSize.height);
		TooN::Vector<9>& v9Params = mParams[camName];
		//std::cout << camName << ": size: " << v2ImgSize << std::endl << "Parameters: " << v9Params << std::endl;
		
		m_mCameraModels.insert(std::pair<std::string, CameraModel>(camName, CameraModel(v9Params, v2ImgSize)));
	}	
	
	m_pImgProc =  new ImageProcess(m_mCameraModels);
	//m_pImgProc->InitUndistort(mImageSizes);
	m_mCamMat =  m_pImgProc->GetCameraMatrix();
	SetCamMatInv(m_mCamMat);
}

bool Tracking::LoadDescriptors(std::string filename)
{
    cv::FileStorage fs;
    fs.open(filename, cv::FileStorage::READ);
    
    if (!fs.isOpened())
    {
        std::cerr << "Failed to open " << filename << std::endl;
        return false;
    }
    
    cv::Mat vDescriptorPool;
    cv::Mat vDescriptorIndex;
    
    fs["Descriptors"] >> vDescriptorPool;
    fs["Index"] >> vDescriptorIndex;
    
    m_pVocTree->LoadVOData("/home/mingfeng/VoData.dat");
    
	m_pGraph->LoadGraph("/home/mingfeng/Graph.dat");
    
    m_pVocTree->BuildBatch(vDescriptorPool, vDescriptorIndex); 
    
    return true;   
}

//void Tracking::FASTExtractor(cv::Mat img_in, cv::Mat img_mask, std::vector<cv::KeyPoint>& keypoints, cv::Mat& descriptors, int threshold)
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
//}

void Tracking::Run()
{
	ImageBWMap imagesBW, imagesNew;
    m_pCamSub->GetAndFillFrameBW(imagesBW);
    
    m_ReferencePose = m_pCamSub->GetReferencePose();
    m_v3ReferencePosition = TooN::makeVector(m_ReferencePose.position.x, m_ReferencePose.position.y, m_ReferencePose.position.z);
    
    if (imagesBW.empty())
    {
        std::cout << "No image is received..." << std::endl;
        return;
    }
    
    if (m_bToRectify)
		m_pImgProc->RemapToPerspective(imagesBW, imagesNew);
	else
		imagesNew = imagesBW;
	
	Compute(imagesNew); // where all the magic happens

	m_pWindow->DrawFrame(imagesNew, m_KeyPointList);  
}

// Receive frames and extract features from each frame and search each frame in the VocTree
void Tracking::Compute(ImageBWMap imagesBW)
{
    cv::Mat queryDesc;
    KeyPointMap KeyPointList;
    ImageBWMap DescriptorList;
    
    for (ImageBWMap::iterator it = imagesBW.begin(); it != imagesBW.end(); it++)
    {
        std::vector<cv::KeyPoint> keypoints;
        cv::Mat descriptors;

		//m_pImgProc->FASTExtractor(it->second, m_mMasks[it->first], keypoints, descriptors, 200, 70);
        m_pImgProc->FeatureExtractor(it->second, m_mMasks[it->first], keypoints, descriptors);

        //store descriptors 
        if (queryDesc.empty() == true)
        {
            descriptors.copyTo(queryDesc);
        }
        else
        {
            queryDesc.push_back(descriptors);
        }
        
        KeyPointList[it->first] = keypoints;
        
        descriptors.copyTo(DescriptorList[it->first]);
    }
    
    m_KeyPointList = KeyPointList;
    
    TooN::Vector<3> v3RPY = UTIL::QuatToRPY(m_ReferencePose.orientation);
    TooN::Vector<6> v6TruePose = TooN::makeVector(m_v3ReferencePosition[0], m_v3ReferencePosition[1], m_v3ReferencePosition[2], v3RPY[0], v3RPY[1], v3RPY[2]);
    TooN::SE3<> truePose = UTIL::PoseMsgToSE3(m_ReferencePose);  // Reference's true pose when the query image is taken.
    
    std::cout << "-----------------------------" << std::endl;
    std::cout << "Querying:               " << v6TruePose << std::endl;
    
    std::map<double, IDType> vMatchedFrames = m_pVocTree->QueryFrame(queryDesc);

    //std::cout << "Matched image ID: " << std::endl;
    std::vector<double> vdScore;
    std::vector<IDType> viIndex;
    std::vector<double> vdPos_vo;
    std::vector<double> vdRot_vo;
    std::vector<double> vdPose_vo;
    
    std::vector<double> vdPos_true;
    std::vector<double> vdRot_true;
    std::vector<double> vdPose_true;
    
    std::vector<double> vdSimilarity;
    
    std::vector< TooN::SE3<> > vse3MatchedPose;
    std::vector< TooN::SE3<> > vse3VOPose;
    
    for (std::map<double, IDType>::iterator it = vMatchedFrames.begin(); it != vMatchedFrames.end(); ++it)
    {
		vdScore.push_back(it->first);
		viIndex.push_back(it->second);
	}
	
    for (std::map<double, IDType>::iterator it = vMatchedFrames.begin(); it != vMatchedFrames.end(); ++it)
    {
		// retrieve the pose of one matching candidate
		VertexProperties<TI> matchedPlace = m_pGraph->GetVertexProperties(it->second);
		TooN::SE3<> se3MatchedPose = TooN::SE3<>::exp(matchedPlace.GetPose()); // one candidate that may match with the query image (current true pose)
		vse3MatchedPose.push_back(se3MatchedPose);
		
		TooN::Vector<6> v6MatchedPose = UTIL::SE3Tov6Pose(se3MatchedPose); //v6= [x, y, z, R, P, Y]
        std::cout << "Candidate: " << it->second << " (" << it->first << "), " << v6MatchedPose << std::endl;    
        
        
        // calculate the relative pose between the candidate frame and the querey frame
        TooN::SE3<> relativePose = se3MatchedPose.inverse() * truePose; // Pose of true frame expressed in candidate frame
        TooN::Vector<6> v6RelativePose = UTIL::SE3Tov6Pose(relativePose);
        std::cout << "  True dPose: " << v6RelativePose << std::endl;
          
        vdPos_true.push_back(TooN::norm(v6RelativePose.slice(0,3)));
        vdRot_true.push_back(TooN::norm(v6RelativePose.slice(3,3)));
        vdPose_true.push_back(TooN::norm(v6RelativePose));
		
		
		cv::Mat mDescRef = m_pVocTree->GetDescriptors(it->second);
		std::vector< cv::DMatch > good_matches;
		double dSimilarity = 0.0;
		m_pImgProc->FeatureMatching(queryDesc, mDescRef, good_matches, dSimilarity);
		
		vdSimilarity.push_back(dSimilarity);
		
		//cv::Mat m = (cv::Mat_<double>(1,4) << dPos, dRot, it->first, dSimilarity);
        //std::cout << "Score = " << m << std::endl;
        
        //if (m_mScoreRecord.empty())
			//m.copyTo(m_mScoreRecord);
		//else
			//m_mScoreRecord.push_back(m);
			
		//vScores.push_back(it->first);
		
		/////////////// VO ///////////////
		//VO data of the matched frame
		cv::Mat desc_1 = m_pVocTree->m_mVODescriptors.at(it->second);
		cv::Mat points = m_pVocTree->m_mVOPoints.at(it->second);
		
		//std::cout << "Original size: " << desc_1.rows << "x" << desc_1.cols << std::endl;
		//std::cout << "Original size: " << points.rows << "x" << points.cols << std::endl;
		
		//cv::Mat rMat, tVec;
		
		TooN::SE3<> se3VOPose = Odometry(camMatA, desc_1, points, DescriptorList["camera3"], KeyPointList["camera3"]); // rMat & tVec transforms 3D points of matched frame to camera frame
		
		//std::cout << "Pose: " << std::endl << rMat << std::endl << tVec << std::endl ;
		
		// se3RefPose
		
		
		
		//cv::Mat rot_3 = Pose1.colRange(0,3);
		//cv::Mat tran_3 = Pose1.colRange(3,4);
		//cv::Mat rot_inv = rMat.inv();
		
		//cv::Mat rot1 = rot_inv * rot_3;
		//cv::Mat tran1 = rot_inv * (tran_3 - tVec);
		
		////cv::Mat Rz = R0inv * rot1.inv() * R0;
		////cv::Mat tz = R0inv * rot1.inv() * (t0 - rot1 * t0 - tran1);
		
		//cv::Mat Rz = R0inv * rot1 * R0;
		//cv::Mat tz = R0inv * rot1 * t0 + R0inv * (tran1 - t0); // pose of true frame expressed in matched frame
		
		TooN::Vector<6> v6VOPose = UTIL::SE3Tov6Pose(se3VOPose);
		
		std::cout << "  VO dPose: "  << v6VOPose  << std::endl;		
		
		vse3VOPose.push_back(se3VOPose);
		
		vdPos_vo.push_back(TooN::norm(v6VOPose.slice(0,3)));
		vdRot_vo.push_back(TooN::norm(v6VOPose.slice(3,3)));	
		vdPose_vo.push_back(TooN::norm(v6VOPose) );	
    }
    
    int nTopCandidate = viIndex[0];
    
    // find the candidate with smallest VO pose change w.r.t the query frame
    std::vector<double>::iterator result = std::min_element(vdPose_vo.begin(), vdPose_vo.end());
    int nVoIndex = std::distance(vdPose_vo.begin(), result);
    int nVoMatched = viIndex[nVoIndex];
    double dpose_vo = *result;
    
    // find the candidate with the small true pose change  w.r.t the query frame
	result = std::min_element(vdPose_true.begin(), vdPose_true.end());
	int nTrueIndex = std::distance(vdPose_true.begin(), result);
    int nTrue = viIndex[nTrueIndex];
    double dpose_true = *result;
    
    // this candidate should be considered as the ground truth, unless the pose differsence > 1
    if (dpose_true > 1.0) { nTrue = -1; }
    
    // find the candidate with the highest similarity  w.r.t the query frame
	result = std::max_element(vdSimilarity.begin(), vdSimilarity.end());
	int nSimIndex = std::distance(vdSimilarity.begin(), result);
    int nSim = viIndex[nSimIndex];
	double dsim = *result;
	
	int nMix = -1;
	double estPose = -1.0;
	TooN::SE3<> se3MatchedPose = TooN::SE3<>::exp(TooN::makeVector(999.9, 999.9, 999.9, 0.0, 0.0, 0.0));
	TooN::SE3<> se3VOPose = TooN::SE3<>::exp(TooN::makeVector(0.0, 0.0, 0.0, 0.0, 0.0, 0.0));
		
	if (vdScore[0] <= 0.3 * vdScore[1])
	{
		nMix = nTopCandidate; // TopCandidate from VoC is selected 
		estPose = vdPose_true[0];

		se3MatchedPose = vse3MatchedPose[0]; 
	}
	else if (vdScore[0] <= 0.8 * vdScore[1])
	{
		nMix = nTopCandidate; // TopCandidate from VoC is selected 
		estPose = vdPose_true[0];

		se3MatchedPose = vse3MatchedPose[0]; 
		se3VOPose = vse3VOPose[0];
	}
	else if( dpose_vo <= 0.8)
	{
		nMix = nVoMatched; //candidate with lowest VO distance is selected
		estPose = vdPose_true[nVoIndex];

		se3MatchedPose = vse3MatchedPose[nVoIndex]; 
		se3VOPose = vse3VOPose[nVoIndex];
	}
	else if(dsim >= 0.4)
	{
		nMix = nSim; // candidate with highest similarity is selected
		estPose = vdPose_true[nSimIndex];
		
		se3MatchedPose = vse3MatchedPose[nSimIndex]; 
		se3VOPose = vse3VOPose[nSimIndex];
	}
	
	std::cout << "-True-Mix-TopCandidate-VoCandidate-Similarity" << std::endl;
	std::cout << "- " << nTrue << " - " << nMix << " - "  << nTopCandidate << " - "  << nVoMatched << " - "  << nSim << std::endl;
    
    if (nMix == -1)
		nTrueIndex = -1;
		
	cv::Mat m = (cv::Mat_<double>(1,6) << nTrueIndex, nTrue, nMix, nTopCandidate, nVoMatched, nSim);
	//std::cout << "Score = " << m << std::endl;
	cv::Mat n = (cv::Mat_<double>(1,6) << vdScore[0],  vdScore[0]/vdScore[1], estPose, dpose_true, dpose_vo, dsim);
	
	if (m_mScoreRecord.empty())
		m.copyTo(m_mScoreRecord);
	else
		m_mScoreRecord.push_back(m);

	if (m_mDistanceRecord.empty())
		n.copyTo(m_mDistanceRecord);
	else
		m_mDistanceRecord.push_back(n);		
    
    m_nTrueFrameID = nTrue;
    m_nMatchedFrameID = nMix;
	m_MatchedFramePose = UTIL::SE3ToPoseMsg(se3MatchedPose);
	m_OdometryPose = UTIL::SE3ToPoseMsg(se3MatchedPose * se3VOPose.inverse() ); //  * m_mPoses.at("camera3")
    
	//std::cout << std::endl;
    
    //cv::Mat mRatio = (cv::Mat_<double>(1,4) << vScores[0]/vScores[1],
			//2.0*vScores[0]/(vScores[1]+vScores[2]),
			//3.0*vScores[0]/(vScores[1]+vScores[2]+vScores[3]),
			//4.0*vScores[0]/(vScores[1]+vScores[2]+vScores[3]+vScores[4]) );
	//if (m_mScoreRatio.empty())
		//mRatio.copyTo(m_mScoreRatio);
	//else
		//m_mScoreRatio.push_back(mRatio);
		
    //std::cout << "Score ratio: " << mRatio << std::endl;
    
    //for (std::map<double, int>::iterator it = vMatchedFrames.begin(); it != vMatchedFrames.end(); ++it)
    //{
		//cv::Mat desc_1 = m_pVocTree->m_mVODescriptors.at(it->second);
		//cv::Mat points = m_pVocTree->m_mVOPoints.at(it->second);
		
		////std::cout << "Original size: " << desc_1.rows << "x" << desc_1.cols << std::endl;
		////std::cout << "Original size: " << points.rows << "x" << points.cols << std::endl;
		
		//cv::Mat rMat, tVec;
		
		//Odometry(camMatA, desc_1, points, DescriptorList["camera3"], KeyPointList["camera3"], rMat, tVec);
		
		////std::cout << "Pose: " << std::endl << rMat << std::endl << tVec << std::endl ;
		
		//cv::Mat rot_3 = Pose1.colRange(0,3);
		//cv::Mat tran_3 = Pose1.colRange(3,4);
		//cv::Mat rot_inv = rMat.inv();
		
		//cv::Mat rot1 = rot_inv * rot_3;
		//cv::Mat tran1 = rot_inv * (tran_3 - tVec);
		
		
		//cv::Mat Rz = R0inv * rot1.inv() * R0;
		//cv::Mat tz = R0inv * rot1.inv() * (t0 - rot1 * t0 - tran1);
		
		//TooN::Vector<3> euler = UTIL::MatToRPY(Rz);
		
		//std::cout << "Relative pose: "  << tz.t() << ", " << euler  << std::endl;
	//}     
}

void Tracking::SetMasks(ImageBWMap& maskMap)
{
	m_mMasks.clear();
	for(ImageBWMap::iterator it = maskMap.begin(); it != maskMap.end(); ++it)
	{
		it->second.copyTo(m_mMasks[it->first]);
	}
}

void Tracking::SetCamMatInv(CameraMatrixMap mCamMat)
{
	m_mCamMatInv.clear();
	for(CameraMatrixMap::iterator it = mCamMat.begin(); it != mCamMat.end(); ++it)
	{
		TooN::Matrix<3, 3> cameraMatrix = it->second;
		TooN::Matrix<3, 3> cameraMatInv = TooN::Data(1.0/cameraMatrix(0,0), 0.0, -cameraMatrix(0,2)/cameraMatrix(0,0),
													 0.0, 1.0/cameraMatrix(1,1), -cameraMatrix(1,2)/cameraMatrix(1,1),
													 0.0, 0.0, 1.0);
		m_mCamMatInv[it->first]	= cameraMatInv;										 
	}		
}

void Tracking::SaveScore(std::string filename)
{
	//std::string filename = "/home/mingfeng/Scores.dat";
    std::cout << "Writing tracking scores to a file: " << filename << std::endl;
    
    cv::FileStorage fs(filename, cv::FileStorage::WRITE);
        
    fs << "Scores" << m_mScoreRecord;
    fs << "Distance" << m_mDistanceRecord;
          
    fs.release(); 
    
    std::cout << "Write done!" << std::endl;	
}

TooN::SE3<>  Tracking::Odometry(const cv::Mat& camMatrix, const cv::Mat& desc_1, const cv::Mat& Points, const cv::Mat& desc_2, std::vector< cv::KeyPoint>& keypoints_2)
{
	std::vector< cv::DMatch > good_matches_cross;
	double dSimilarity;
	m_pImgProc->FeatureMatching(desc_1, desc_2, good_matches_cross, dSimilarity);	
	
	if (good_matches_cross.size() < 5 )//dSimilarity < 0.2)
	{
		//rMat = (cv::Mat_<double>(3,3) << -1.0, 0, 0, 0, -1.0, 0, 0, 0, -1.0);
		//tVec = (cv::Mat_<double>(3,1) << 99999.9, 99999.9, 99999.9 );
		return TooN::SE3<>::exp(TooN::makeVector(99999.9, 99999.9, 99999.9, 0.0, 0.0, 0.0 ));
	}
	
	cv::Mat Point4D_Matched;
	std::vector<cv::Point2f> kp_matched;
	for (size_t i = 0; i < good_matches_cross.size(); i++)
	{
		cv::Mat temp = Points.t();
		if(Point4D_Matched.empty())
			temp.row(good_matches_cross[i].queryIdx).copyTo(Point4D_Matched);
		else
			Point4D_Matched.push_back(temp.row(good_matches_cross[i].queryIdx));
		
		kp_matched.push_back(keypoints_2[good_matches_cross[i].trainIdx].pt);
	}	
	
	cv::Mat Point3DMatched;
	Point4D_Matched.colRange(0,3).copyTo(Point3DMatched);
	
	//std::cout << "Point size for VO: " << good_matches_cross.size() << std::endl;
	//std::cout << "Matched 3D Points: " << std::endl << Point3DMatched << std::endl;
	
	cv::Mat rvec, tvec;
	cv::solvePnPRansac(Point3DMatched, kp_matched, camMatrix, cv::Mat(),  rvec,  tvec);
	
	TooN::Vector<6> v6Pose = TooN::makeVector(tvec.at<double>(0), tvec.at<double>(1), tvec.at<double>(2), rvec.at<double>(0),  rvec.at<double>(1),  rvec.at<double>(2));
	
	return TooN::SE3<>::exp(v6Pose);
	//cv::Rodrigues(rvec, rMat);	
}

void Tracking::SetStereoNames(std::vector<std::string> vStereoNames)
{
	assert(vStereoNames.size() == 2);
	m_vStereoNames = vStereoNames;
	
	TooN::Matrix<> camMat = m_mCamMat.at(m_vStereoNames[0]);
	camMatA = (cv::Mat_<double>(3,3) << camMat(0, 0), camMat(0, 1), camMat(0, 2),camMat(1, 0),camMat(1, 1),camMat(1, 2),camMat(2, 0),camMat(2, 1),camMat(2, 2));
}
