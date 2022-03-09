#include <toposlam/Mapping.h>
#include <toposlam/Utility.h>
//#include <boost/geometry.hpp>
//#include <boost/geometry/geometries/point_xy.hpp>
//#include <boost/geometry/geometries/polygon.hpp>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include <tf/transform_datatypes.h>
#include <boost/foreach.hpp>
#include <deque>
#include <assert.h>     /* assert */
#include <TooN/SVD.h>

//typedef boost::geometry::model::polygon<boost::geometry::model::d2::point_xy<float> > polygon;
////typedef boost::geometry::model::point<float, 2, bg::cs::cartesian>
//typedef boost::geometry::model::d2::point_xy<float>  point;

//static const std::string OPENCV_WINDOW = "TOPO-SLAM";

Mapping::Mapping() : m_nKeyframeIndex(0) //, m_eMode(MODE_DISPLAY)
{
	// Read image from camera/bagfiles
    const char *sCamNames[] = {"camera1", "camera2", "camera3", "camera4"};
    std::vector<std::string> vCameraNames(sCamNames, sCamNames+4);
    m_pCamSub = new MultiCameraSubscriber(vCameraNames);
    
    InitMapping();
    m_pWindow = new ImageWindow();
    
    m_bToRectify = true;
}

Mapping::Mapping(MultiCameraSubscriber* pCamSub, ImageWindow* pWindow, ImageProcess* pImgProc, VocabTree*  pVocTree, TopoGraph<TV, TE, TI>* pGraph, bool bToRectify) 
: m_nKeyframeIndex(0),  m_pCamSub(pCamSub), m_pWindow(pWindow), m_pImgProc(pImgProc), m_pVocTree(pVocTree), m_pGraph(pGraph), m_bToRectify(bToRectify) 	
{
	// Get the pose of each camera w.r.t. to the first camera
	m_mPoses.clear();
	m_mPoses = m_pCamSub->GetPoses();
	
	// Get the number of cameras
	m_nCameraNumber = m_pCamSub->GetCameraNumber();
	
	// Get camera matrices
	m_mCamMat =  m_pImgProc->GetCameraMatrix();
	
	//for(CameraMatrixMap::iterator it = m_mCamMat.begin(); it != m_mCamMat.end(); ++it)
	//{
		//std::cout << it->first << " matrix: " << std::endl << it->second << std::endl;
		//std::cout << it->first << " pose: " << std::endl << m_mPoses.at(it->first) << std::endl;
	//}
	
	// calculate the inverse of each camera's matrix
	SetCamMatInv(m_mCamMat);	
	
	////position: 
	//TooN::Vector<3> imuInCam1_t = TooN::makeVector(-0.062, -0.153, -0.084);
	////orientation: 
	//TooN::Vector<4> imuInCam1_r = TooN::makeVector(-0.620, 0.656, -0.307, 0.301);
	
	//geometry_msgs::Pose imuPoseInCam1;
	
	  //// Fill in position directly
	//imuPoseInCam1.position.x = imuInCam1_t[0];
	//imuPoseInCam1.position.y = imuInCam1_t[1];
	//imuPoseInCam1.position.z = imuInCam1_t[2];
	
	//// Get quaternion values from Bullet quat
	//imuPoseInCam1.orientation.x = imuInCam1_r[0];
	//imuPoseInCam1.orientation.y = imuInCam1_r[1];
	//imuPoseInCam1.orientation.z = imuInCam1_r[2];
	//imuPoseInCam1.orientation.w = imuInCam1_r[3];		
		
	//TooN::SE3<> m_se3IMUPose = UTIL::PoseMsgToSE3(imuPoseInCam1);
}

Mapping::~Mapping()
{
	if (m_pCamSub != NULL) { delete m_pCamSub; }
	
	if (m_pWindow != NULL) { delete m_pWindow; }
	
	if (m_pImgProc != NULL) { delete m_pImgProc; }
	
	if (m_pVocTree != NULL) {delete m_pVocTree; }	
	
	if (m_pGraph != NULL) {delete m_pGraph; }
}

void Mapping::InitMapping()
{
	// Get the pose of each camera w.r.t. to the first camera
	m_mPoses.clear();
	m_mPoses = m_pCamSub->GetPoses();
	
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
	
	m_mCamMat =  m_pImgProc->GetCameraMatrix();
	SetCamMatInv(m_mCamMat);
}

void Mapping::Run()
{
	// capture a frame of images
    ImageBWMap imagesBW, imagesNew;
    m_pCamSub->GetAndFillFrameBW(imagesBW);
    
    m_ReferencePose = m_pCamSub->GetReferencePose();
    m_v3ReferencePosition = TooN::makeVector(m_ReferencePose.position.x, m_ReferencePose.position.y, m_ReferencePose.position.z);
      
    if (imagesBW.empty())
    {
        std::cout << "Mapping: No image is received ..." << std::endl;
        return;
    }
    
    if (m_bToRectify)	
		m_pImgProc->RemapToPerspective(imagesBW, imagesNew);	 
	else
		imagesNew = imagesBW;
	
	Compute(imagesNew); // where all the magic happens	 
	m_pWindow->DrawFrame(imagesNew, m_KeyPointsToDraw);  
}

void Mapping::Compute(ImageBWMap imagesNew)
{	 
	// extract features from multiple images in the current frame
    for (ImageBWMap::iterator it = imagesNew.begin(); it != imagesNew.end(); ++it)
    {
        std::vector<cv::KeyPoint> keypoints;
        cv::Mat descriptors;

        // use FASTX , 200 features/image, threshold: 70
        //m_pImgProc->FASTExtractor(it->second, m_mMasks[it->first], keypoints, descriptors, 200, 70);
        m_pImgProc->FeatureExtractor(it->second, m_mMasks[it->first], keypoints, descriptors);
            
        //store keypoints and their descriptors, tagged by camera names
        m_KeyPointList[it->first] = keypoints;
        descriptors.copyTo(m_DescriptorList[it->first]);
    }
    
    m_KeyPointsToDraw = m_KeyPointList;
    
    // if it is the first frame, it should be a keyframe and be added to the map
    if (m_DescriptorList_Last.empty() == true) 
    {
        std::cout << "Empty" << std::endl;

        // save descriptors and index to a pool
        m_nKeyframeIndex = 0;
        
        // save the descriptors and keypoints
        m_DescriptorList_Last = m_DescriptorList;
        m_KeyPointList_Last = m_KeyPointList; 
        
        RecordReferencePose();
        // save the Reference's pose
        m_ReferencePose_Last = m_ReferencePose;	
        m_v3ReferencePosition_Last = m_v3ReferencePosition;
           
        for (ImageBWMap::iterator it = m_DescriptorList.begin(); it != m_DescriptorList.end(); ++it)
        {
            if (m_DescriptorPool.empty() == true)
            {
                m_DescriptorPool = m_DescriptorList[it->first];
                cv::Mat temp(m_DescriptorList[it->first].rows, 1, CV_32S, cv::Scalar(m_nKeyframeIndex));
                m_DescriptorIndex = temp;
            }
            else
            {
                m_DescriptorPool.push_back(m_DescriptorList[it->first]);
                cv::Mat temp(m_DescriptorList[it->first].rows, 1, CV_32S, cv::Scalar(m_nKeyframeIndex));
                m_DescriptorIndex.push_back(temp);
            }
        }
		
		// Add a node to the graph
		//TooN::Vector<3> v3RPY = UTIL::QuatToRPY(m_ReferencePose.orientation);
		TooN::SE3<> se3Pose = UTIL::PoseMsgToSE3(m_ReferencePose);
        TooN::Vector<6> v6Pose = se3Pose.ln(); //TooN::makeVector(m_v3ReferencePosition[0], m_v3ReferencePosition[1], m_v3ReferencePosition[2], v3RPY[0], v3RPY[1], v3RPY[2]);
        
		TV node(m_nKeyframeIndex, v6Pose);
		TE edge_forward(1.0);
		TE edge_backward(1.0);
		bool bNode = m_pGraph->AddNode(node, edge_forward, edge_backward);
		
		//cv::Mat Point4D = Stereo(m_vStereoNames[0], m_vStereoNames[1]);
        
        return;
    }    
    
    m_nTotalMatches = 0;
    m_nDescriptors = 0;
    m_nDescriptors_Last = 0;
    double dMeanSimilarity = 0.0;    
    
    std::map<std::string, std::vector<cv::DMatch> > matchesMap;
    
	// Find matches between two consecutive frames in all sub-images   
    for (ImageBWMap::iterator it = m_DescriptorList.begin(); it != m_DescriptorList.end(); ++it)
    {
        std::cout << "------" << it->first << "------" <<  std::endl;
 
        m_nDescriptors += m_DescriptorList[it->first].rows;
        m_nDescriptors_Last += m_DescriptorList_Last[it->first].rows;


        std::vector< cv::DMatch > good_matches;
		double dSimilarity = 0.0;
		m_pImgProc->FeatureMatching(m_DescriptorList_Last[it->first], m_DescriptorList[it->first], good_matches, dSimilarity);
		dMeanSimilarity += dSimilarity;
		
		std::cout << "Similarity = " << dSimilarity << std::endl;
        std::cout << "Good matches size: " << good_matches.size() << std::endl;
		
		matchesMap[it->first] = good_matches;
        
        m_nTotalMatches += good_matches.size();        
    }

	dMeanSimilarity /= m_nCameraNumber;
	
	if (isKeyframe())
	//if(dMeanSimilarity < 0.6) // add a keyframe when the mean similarity is below 60%
	{
		SE3Map mCameraMotion;
		
		////visual odometry through essential matrix
		for(std::map<std::string, std::vector<cv::DMatch> >::iterator it = matchesMap.begin(); it != matchesMap.end(); ++it)
		{
			std::cout << "------" << it->first << "------" <<  std::endl;
			
			std::vector<cv::DMatch>& good_matches = it->second; // good matches
			
			//Get the keypoints from the good matches
		    std::vector<cv::Point2f> kpmatched_last, kpmatched;
		    
			for( size_t i = 0 ; i <  good_matches.size(); i++) 
			{
				kpmatched_last.push_back( m_KeyPointList_Last[it->first][ good_matches[i].queryIdx ].pt );
				kpmatched.push_back( m_KeyPointList[it->first][ good_matches[i].trainIdx ].pt );
			}
			
			TooN::Matrix<3> cameraMatrixInv = m_mCamMatInv.at(it->first);
			TooN::Matrix<3> cameraMatrix = m_mCamMat.at(it->first);
			
			TooN::Matrix<3> E;
			cv::Mat mInliersIndex;
			getEssential(kpmatched_last, kpmatched, cameraMatrix, cameraMatrix, E, mInliersIndex);
			
			std::vector< TooN::Vector<3> >  v3Points_1, v3Points_2;
			for(size_t ip = 0; ip < kpmatched_last.size(); ip++)
			{
				if (mInliersIndex.at<bool>(ip) == true)
				{
					cv::Point2f kpInlier = kpmatched_last[ip];
					TooN::Vector<3> v3Inlier = cameraMatrixInv * TooN::makeVector(kpInlier.x, kpInlier.y, 1.0);
					v3Points_1.push_back(v3Inlier);
					
					kpInlier = kpmatched[ip];
					v3Inlier = cameraMatrixInv * TooN::makeVector(kpInlier.x, kpInlier.y, 1.0);
					v3Points_2.push_back(v3Inlier);
				}
			}
			
			std::vector< TooN::Matrix<3> > rot;
			TooN::Vector<3> tran;
			
			bool bRT = getRT(E, rot, tran);
			
			std::cout << "E = " << E << std::endl;
			 
			
			int indexR[4] = {0, 1, 0, 1};
			int indexT[4] = {1, -1, -1, 1};
			int vInliers[4] = {0};
			int nMaxInliers =  0;
			int nMaxIndex = -1;
			
			for (int i = 0; i < 4; i++)
			{
				TooN::Matrix<3> R = rot[indexR[i]];
				TooN::Vector<3> t = indexT[i] * tran;
				
				vInliers[i] = VerifyRT(R, t, v3Points_1, v3Points_2);
				
				if ( vInliers[i] > nMaxInliers)
				{
					nMaxInliers = vInliers[i];
					nMaxIndex = i;
				}
				
				//std::cout << "R = " << R ;
				//std::cout << "t = " << t << std::endl;
				
				std::cout << "Solution[" << i << "] inliers: " << vInliers[i] << "/" << v3Points_1.size() << std::endl << std::endl;
			}
			
			if (nMaxIndex >= 0)
			{
				TooN::Matrix<3> R = rot[indexR[nMaxIndex]];
				TooN::Vector<3> t = indexT[nMaxIndex] * tran;
				
				TooN::SO3<> so3R(R);
				TooN::SE3<> se3RT(so3R, t);
				mCameraMotion[it->first] = se3RT;
				
				std::cout << "R = " << R;
				std::cout << "t = " << t << std::endl << std::endl;
			}
			else
			{
				std::cout << "No valide essential matrix found :(" << std::endl;
			}			
		}	
		
		if(mCameraMotion.size() >= 4)
		{
			std::vector< TooN::Vector<3> > vRelTrans, vMotionTrans;
			std::vector< TooN::Matrix<3> > vRelRots, vMotionRots;
			
			for (SE3Map::iterator iter =  mCameraMotion.begin(); iter != mCameraMotion.end(); ++iter)
			{
				TooN::SE3<> relPose = m_mPoses.at(iter->first); 
				vRelTrans.push_back(relPose.get_translation ());
				vRelRots.push_back(relPose.get_rotation().get_matrix());
				
				TooN::SE3<> motionPose = iter->second;
				vMotionTrans.push_back(motionPose.get_translation());
				vMotionRots.push_back(motionPose.get_rotation().get_matrix());
				
				TooN::SE3<> estPose = relPose.inverse() * motionPose * relPose;
				
				std::cout << "Reference camera estimated pose: " << std::endl;
				std::cout << estPose << std::endl;
			}
			
			TooN::Matrix<> A = TooN::Zeros(9,4);
			TooN::Vector<> B = TooN::Zeros(9);
			
			TooN::Vector<> t1 = vMotionTrans[0];
			for(int k = 1; k < 4; k++)
			{
				TooN::Vector<> a = vRelRots[k] * t1;
				TooN::Vector<> tmotion = vMotionTrans[k];
				TooN::Matrix<> Iden = TooN::Identity(3);
				TooN::Vector<> b = (vMotionRots[k] - Iden) * vRelTrans[k];
				
				for (int n = 0; n < 3; n++)
				{
					A[(k-1)*3+n][0] = a[n];
					A[(k-1)*3+n][k] = -tmotion[n];
				}
				
				B.slice((k-1)*3, 3) = b;
			}
			
			std::cout << "A = " << A << std::endl;
			std::cout << "B = " << B << std::endl;
 			
			TooN::SVD<9,4> svdA(A);
			
			TooN::Vector<> lambda = svdA.backsub(B);
			
			std::cout << "Scale is: " << lambda << std::endl;
		}
			//std::vector<cv::DMatch>& good_matches = it->second; // good matches
			
			////Get the keypoints from the good matches
		    //std::vector<cv::Point2f> kpmatched_last;
			//std::vector<cv::Point2f> kpmatched;
			//for( size_t i = 0 ; i <  good_matches.size(); i++) 
			//{
				//kpmatched_last.push_back( m_KeyPointList_Last[it->first][ good_matches[i].queryIdx ].pt );
				//kpmatched.push_back( m_KeyPointList[it->first][ good_matches[i].trainIdx ].pt );
			//}
		        
			//if (good_matches.size() >= 8)
			//{
				//// to calculate fundamental matrix
				//cv::Mat mPointStatus; // same size as kpmatched_last, 0: outlier, 1: inlier
				//cv::Mat fundamental_matrix = cv::findFundamentalMat(kpmatched_last, kpmatched, cv::FM_RANSAC, 3, 0.99, mPointStatus);
				////std::cout << "Fundamental matrix = " << fundamental_matrix << std::endl;
				////std::cout << "mPointStatus= " << mPointStatus << std::endl;
							
				//std::vector< TooN::Vector<3> > kpused_last;
				//std::vector< TooN::Vector<3> > kpused;
				//std::vector<cv::Point2f> kpinlier_last;
				//std::vector<cv::Point2f> kpinlier;
				//for(size_t ip = 0; ip < kpmatched_last.size(); ip++)
				//{
					//if (mPointStatus.at<bool>(ip) == true)
					//{
						////std::cout << "Point" << ip << " status: "   << mPointStatus.at<bool>(ip) << std::endl;
						//TooN::Vector<3> PointImg = TooN::makeVector(kpmatched_last[ip].x, kpmatched_last[ip].y, 1);			
						//TooN::Vector<3> PointCam_A = m_mCamMatInv.at(it->first) * PointImg;
				
						//PointImg = TooN::makeVector(kpmatched[ip].x, kpmatched[ip].y, 1);
						//TooN::Vector<3> PointCam_B = m_mCamMatInv.at(it->first) * PointImg;
						
						//kpused_last.push_back(PointCam_A);
						//kpused.push_back(PointCam_B);
						
						//kpinlier_last.push_back(kpmatched_last[ip]);
						//kpinlier.push_back(kpmatched[ip]);
					//}
				//}

				//fundamental_matrix = cv::findFundamentalMat(kpinlier_last, kpinlier, cv::FM_8POINT, 3, 0.99); //, mPointStatus);
				
				//TooN::Matrix<3,3> mfundamental; // = TooN::wrapMatrix(fundamental_matrix.data);
				//for (unsigned i = 0; i < 3; i++)
				//{
					//for (unsigned j = 0; j < 3; j++)
					//{
						//mfundamental(i,j) = fundamental_matrix.at<double>(i,j);
					//}
				//}
				
				//TooN::Matrix<3, 3> camMat = m_mCamMat.at(it->first);
				//TooN::Matrix<3,3> mEssential = camMat.T() * mfundamental * camMat;				
				
				
				//TooN::Matrix<3,3> R;
				//TooN::Vector<3> b;
				//SVDEssential(mEssential, kpused_last, kpused, R, b);
				//double det = TooN::determinant(R);
				//std::cout << "R = " << R;
				//std::cout << "b = " << b << std::endl;
				//std::cout << "det(R) = " << det << std::endl;
				////TooN::SE3<> se3ForwardTrans(TooN::SO3<>(R.T()), -R.T() * b);
				////TooN::SE3<> se3BackwardTrans(TooN::SO3<>(R), b);           
			//}
		//}		
				
		
		 m_nKeyframeIndex++; // index of a new place/node	
		 
        // Push the descriptors from the current keyframe to a pool (to be clustered)
        for (ImageBWMap::iterator it = m_DescriptorList.begin(); it != m_DescriptorList.end(); ++it)
        {
            if (m_DescriptorPool.empty() == true)
            {
                m_DescriptorPool = m_DescriptorList[it->first];
                cv::Mat temp(m_DescriptorList[it->first].rows, 1, CV_32S, cv::Scalar(m_nKeyframeIndex));
                m_DescriptorIndex = temp;
            }
            else
            {
                m_DescriptorPool.push_back(m_DescriptorList[it->first]);
                cv::Mat temp(m_DescriptorList[it->first].rows, 1, CV_32S, cv::Scalar(m_nKeyframeIndex));
                m_DescriptorIndex.push_back(temp);
            }
        }
        
        //TooN::Vector<3> v3RPY = UTIL::QuatToRPY(m_ReferencePose.orientation);
        //TooN::Vector<6> v6Pose = TooN::makeVector(m_v3ReferencePosition[0], m_v3ReferencePosition[1], m_v3ReferencePosition[2], v3RPY[0], v3RPY[1], v3RPY[2]);
		TooN::SE3<> se3Pose = UTIL::PoseMsgToSE3(m_ReferencePose);
        TooN::Vector<6> v6Pose = se3Pose.ln();
        
        TV node(m_nKeyframeIndex, v6Pose);
		TE edge_forward(1.0);
		TE edge_backward(1.0);
		bool bNode = m_pGraph->AddNode(node, edge_forward, edge_backward);
        
        
        //cv::Mat Point4D = Stereo(m_vStereoNames[0], m_vStereoNames[1]);
        //std::cout << "4D Points: " << std::endl << Point4D.t() << std::endl;
        
        // save record
        m_DescriptorList_Last = m_DescriptorList;
        m_KeyPointList_Last = m_KeyPointList;
       
        
        m_ReferencePose_Last = m_ReferencePose;	
        m_v3ReferencePosition_Last = m_v3ReferencePosition;
        
        RecordReferencePose();
        //SaveImages(imagesNew);
	}
    
    std::cout << "Detect frame: " << m_nKeyframeIndex << std::endl;	
}

void Mapping::Clustering()
{
	std::cout << "Index Size: " << m_DescriptorIndex.rows << "x" << m_DescriptorIndex.cols <<std::endl;
	std::cout << "Pool Size: " << m_DescriptorPool.rows << "x" << m_DescriptorPool.cols <<std::endl;  
	
	std::string filename = "/home/mingfeng/Descriptors.dat";
	
	SaveDescriptors(filename);
	
	filename = "/home/mingfeng/VoData.dat";
	
	m_pVocTree->SaveVOData(filename);
	
	filename = "/home/mingfeng/Graph.dat";
	
	m_pGraph->SaveGraph(filename);
	
	//m_pVocTree->m_mVODescriptors.clear();
	//m_pVocTree->m_mVOPoints.clear();
	
	//m_pVocTree->LoadVOData(filename);
	
	//MatIndexMap desc = m_pVocTree->m_mVODescriptors;
	//MatIndexMap poin = m_pVocTree->m_mVOPoints;
	
	//for(MatIndexMap::iterator it = desc.begin(); it != desc.end(); ++it)
	//{
		//std::cout << "Keyframe " << it->first << std::endl << it->second << std::endl;
	//}

	//for(MatIndexMap::iterator it = poin.begin(); it != poin.end(); ++it)
	//{
		//std::cout << "Points " << it->first << std::endl << it->second << std::endl;
	//}
	
	m_pVocTree->BuildBatch(m_DescriptorPool, m_DescriptorIndex);
	
	std::cout << "Clustering done!" << std::endl;
}

void Mapping::SaveDescriptors(std::string filename)
{
    std::cout << "Writing descriptos and index to a file: " << filename << std::endl;
    
    cv::FileStorage fs(filename, cv::FileStorage::WRITE);
        
    fs << "Descriptors" << m_DescriptorPool;
    fs << "Index" << m_DescriptorIndex;
    fs << "Pose" << m_mPoseRecord;
        
    fs.release(); 
    
    std::cout << "Write done!" << std::endl;
}

//double Mapping::OverlapRatio(int width, int height, cv::Mat homography)
//{
    //std::vector<cv::Point2f> old_corners(4);
    //std::vector<cv::Point2f> new_corners(4);
    
    //old_corners[0] = cv::Point2f( 0, 0 ); 
    //old_corners[1] = cv::Point2f( width, 0 );
    //old_corners[2] = cv::Point2f( width, height); 
    //old_corners[3] = cv::Point2f( 0, height );

    //cv::perspectiveTransform(old_corners, new_corners, homography);
    
    ////std::cout << "Rect old: " ;
    ////for (int j = 0; j < 4; j++)
    ////{
        ////std::cout <<  "(" << old_corners[j].x << ", " << old_corners[j].y << ")  ";
    ////}
    ////std::cout << std::endl;

    ////std::cout << "Rect new: " ;
    ////for (int j = 0; j < 4; j++)
    ////{
        ////std::cout <<  "(" << new_corners[j].x << ", " << new_corners[j].y << ")  ";
    ////}
    ////std::cout << std::endl;
    ////std::cout << "Homography = " << std::endl << homography << std::endl;
    
    
    //polygon oldRect;
    //polygon newRect;
    //oldRect.outer().push_back(point(0, 0));
    //oldRect.outer().push_back(point(width + 0.0f, 0));
    //oldRect.outer().push_back(point(width + 0.0f, -height + 0.0f));
    //oldRect.outer().push_back(point(0, -height + 0.0f));
    
    //newRect.outer().push_back(point(new_corners[0].x + 0.0f, new_corners[0].y + 0.0f));
    //newRect.outer().push_back(point(new_corners[1].x + 0.0f, new_corners[1].y + 0.0f));
    //newRect.outer().push_back(point(new_corners[2].x + 0.0f, -new_corners[2].y + 0.0f));
    //newRect.outer().push_back(point(new_corners[3].x + 0.0f, -new_corners[3].y + 0.0f));
    
    //std::deque<polygon> output;
    //boost::geometry::intersection(oldRect, newRect, output);

    //std::cout << "Area old: " << boost::geometry::area(oldRect) << std::endl;
    //std::cout << "Area new: " << boost::geometry::area(newRect) << std::endl;
    //double fRatio = 0.0;
    
    //if ( output.empty() != true )
    //{  
        //fRatio = boost::geometry::area(output[0])/width/height;
        
        ////int i = 0;
        //////std::cout << "Overlap: ";
        ////BOOST_FOREACH(polygon const& p, output)
        ////{
            //////std::cout << boost::geometry::wkt<polygon>(p) << std::endl;
            //////std::cout << i++ << ": " << boost::geometry::area(p) << std::endl;
            ////double ratio = boost::geometry::area(p)/width/height;
            ////fRatio += ratio;
            //////std::cout << "Overlap ratio: " << ratio << std::endl;
        ////}
        
    //}
    
    ////std::cout << "Overlap ratio: " << fRatio << std::endl;
    
    //return fRatio;
//}

void Mapping::DecomposeEssential(const TooN::Matrix<3,3>& E, TooN::Matrix<3,3>& R1, TooN::Matrix<3,3>& R2, TooN::Vector<3>& b)
{
	TooN::Matrix<3,3> BB = 0.5 * TooN::trace(E*E.T())*TooN::Identity(3) - E*E.T();
	
	double bd = 0;
	unsigned index = 0;
	for (unsigned i = 0; i < 3; i++)
	{
		if (BB(i,i) > bd)
		{
			bd = BB(i,i);
			index = i;
		}
	}
	b = TooN::makeVector(0, 0, 0);
	if(bd >0)
	{
		b = BB[index] / std::sqrt(BB(index,index));
	}
	
	R1 = TooN::Identity(3);
	R2 = TooN::Identity(3);
	
	TooN::Matrix<3,3> ECO, ET;
	ET = E.T();
	
	for (unsigned i = 0; i<3; i++)
	{
		ECO[i] = TooN::cross_product_matrix(ET[(i+1)%3]) * ET[(i+2)%3];
	}
	
	R1 = (ECO.T() - TooN::cross_product_matrix(b) * E) / (b * b);
	R2 = (ECO.T() - TooN::cross_product_matrix(-b) * E) / (b * b);
	
	//TooN::normalize(b);
}

void Mapping::SetCamMatInv(CameraMatrixMap mCamMat)
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

void Mapping::SVDEssential(const TooN::Matrix<3,3>& E, std::vector< TooN::Vector<3> >& kpused_last, std::vector< TooN::Vector<3> >& kpused, TooN::Matrix<3,3>& R, TooN::Vector<3>& b)
{
	TooN::SVD<>  svd(E);
	TooN::Matrix<3,3> U = svd.get_U();		
	TooN::Matrix<3,3> VT = svd.get_VT();
	TooN::Vector<> D = svd.get_diagonal();
	TooN::Matrix<3,3> W = TooN::Data(0, -1.0 , 0, 1.0, 0, 0, 0, 0, 1.0);
	
	TooN::Matrix<3,3> R_1 = U*W.T()*VT;
	TooN::Matrix<3,3> R_2 = U*W*VT;
	TooN::Matrix<3> t_skew = U*W*D.as_diagonal()*U.T();
	TooN::Vector<3> t =  TooN::makeVector(-t_skew(1,2), t_skew(0,2), -t_skew(0,1));
	
	//std::cout << "SVD: " << std::endl;
	//std::cout << "Rotation_1 = " << R_1; 
	//std::cout << "R1*R1'= " << R_1 * R_1.T();
	//std::cout << "Rotation_2 = " << R_2;
	//std::cout << "R2*R2'= " << R_2 * R_2.T();
	//std::cout << "Translation = (+/-) " << t << std::endl;
	//std::cout << "Diagonal D = " << D << std::endl;
	
	//std::cout << "E = " << messential << std::endl;
	//std::cout << "txR_1 = " << (TooN::cross_product_matrix(t) * R_1);
	//std::cout << "txR_2 = " << (TooN::cross_product_matrix(-t) * R_2);
	
	//DecomposeEssential(messential, R_1, R_2, t);
	//std::cout << "non-SVD: " << std::endl;
	//std::cout << "Rotation_1 = " << R_1; 
	//std::cout << "R1*R1'= " << R_1 * R_1.T();
	//std::cout << "Rotation_2 = " << R_2;
	//std::cout << "R2*R2'= " << R_2 * R_2.T();
	//std::cout << "Translation = (+/-)" << t << std::endl;
	
	//std::cout << "E = " << messential << std::endl;
	//std::cout << "txR_1 = " << (TooN::cross_product_matrix(t) * R_1);
	//std::cout << "txR_2 = " << (TooN::cross_product_matrix(-t) * R_2);
			
	std::vector< TooN::Matrix<3,3> > rot;
	std::vector< TooN::Vector<3> > tran;
	rot.push_back(R_1);
	rot.push_back(R_2);
	rot.push_back(R_1);
	rot.push_back(R_2);
	tran.push_back(t);
	tran.push_back(t);
	tran.push_back(-t);
	tran.push_back(-t);
	
	std::vector<int> pdcount(4, 0); // 4  counting index with initial value as zero
	for(size_t ip = 0; ip < kpused_last.size(); ip++)
	{
		//std::cout << "Points: " << PointCam_A << " : " << PointCam_B << std::endl;
		for (unsigned i = 0; i < 4; i++)
		{
			//std::cout << "R = " <<  rot[i] << ", t = " << tran[i] << std::endl;
			bool depthPositive = isPointInFront(rot[i], tran[i], kpused_last[i], kpused[i]);
			if (depthPositive)
			{
				pdcount[i] += 1;
			}
		}
	}
	
	//std::cout << "Size = " << mPointStatus.size() << "Type = " << mPointStatus.type() << std::endl;
	std::cout << "Positive depth test of " << kpused_last.size() << " inlier points: " ;
	for(unsigned i =0; i<4; i++)
	{
		std::cout<< pdcount[i] << " "  ;
	}
	   std::cout << std::endl;
	   
	int nTrans = std::distance(pdcount.begin(), std::max_element(pdcount.begin(), pdcount.end()));	
	
	R = rot[nTrans];
	b = tran[nTrans];
}
 
bool Mapping::isPointInFront(const TooN::Matrix<3> R, const TooN::Vector<3> t, const TooN::Vector<3>  p1, const TooN::Vector<3> p2)
{	
	// d_2*p2 = d_1*R*p1 + t
	// both d_1 & d_2 must be positive to ensure the intesection point is in front of both views
	
	TooN::Vector<3> eta_1 = TooN::makeVector(1.0, 0, -p2[0]);
	TooN::Vector<3> eta_2 = R * TooN::makeVector(1.0, 0, -p1[0]);
	
	double d1 = -eta_1 * t / (eta_1 * (R* p1));
	double d2 = eta_2 * t / (eta_2 * p2); 
	
	if ( d1 > 0 && d2 > 0 )
		return true;
	else
		return false;
}

//void Mapping::CreateMasks()
//{
    //ImageBWMap imagesBW, imagesNew;
    //m_pCamSub->GetAndFillFrameBW(imagesBW);
    
    //if (imagesBW.empty())
    //{
        //std::cout << "Mapping: No image is received ..." << std::endl;
        //return;
    //}	
    
    //static int nImages = 0;
    
    //m_pImgProc->RemapToPerspective(imagesBW, imagesNew);
    
    //for (ImageBWMap::iterator it = imagesNew.begin(); it != imagesNew.end(); ++it)
    //{
		//m_mImageHistory[it->first].push_back(it->second);		
    //}
    
    //nImages++;
    
    //if (nImages == 40)	
    //{
		//ImageBWMap ImgMasks;
		//for (std::map< std::string, std::vector<cv::Mat> >::iterator it = m_mImageHistory.begin(); it != m_mImageHistory.end(); ++it)
		//{
			//std::vector<cv::Mat> ImgDiff;
			//cv::Mat mask = cv::Mat::zeros(it->second[0].size(), it->second[0].type());			
			//for(int j = 0; j < (nImages/2); j++)
			//{
				//cv::Mat diff = it->second[j+(nImages/2)] - it->second[j];
				//cv::Mat dst0;
				//for ( int i = 1; i < 11; i = i + 2 )
				//{ 
					//cv::GaussianBlur( diff, dst0, cv::Size( i, i ), 0, 0 );
				//}
				//cv::Mat dst;
				//cv::threshold( dst0, dst, 3, 255, cv::THRESH_BINARY);
				////cv::adaptiveThreshold(dst0, dst, 255, cv::ADAPTIVE_THRESH_GAUSSIAN_C, cv::THRESH_BINARY, 7, 0);
				
				//ImgDiff.push_back(dst);
				//mask += (dst/(nImages/2.0f));
			//}
			//cv::Mat mask_out;
			////cv::fastNlMeansDenoising(mask, mask_out, 9); //, float h=3, int templateWindowSize=7, int searchWindowSize=21 )
			//for ( int i = 1; i < 3; i = i + 2 )
			//{ 
				//cv::GaussianBlur( mask, mask_out, cv::Size( i, i ), 0, 0 );
			//}
			//cv::Mat final;
			//cv::threshold( mask_out, final, 1, 255, 0 );
			//ImgMasks[it->first] = mask_out;
			//std::string filename = "/home/mingfeng/" + it->first + "_mask.jpg";
			//cv::imwrite( filename, mask );
			//filename = "/home/mingfeng/" + it->first + "_mask_out.jpg";
			//cv::imwrite( filename, final );
		//}
		
		//std::exit(0);
	//}
//}

void Mapping::SetMasks(ImageBWMap& maskMap)
{
	m_mMasks.clear();
	for(ImageBWMap::iterator it = maskMap.begin(); it != maskMap.end(); ++it)
	{
		it->second.copyTo(m_mMasks[it->first]);
	}
}

bool Mapping::isKeyframe()
{
	//return true;
	geometry_msgs::Point position_last  = m_ReferencePose_Last.position;
	geometry_msgs::Quaternion quat_last = m_ReferencePose_Last.orientation;
	
	geometry_msgs::Point position  = m_ReferencePose.position;
	geometry_msgs::Quaternion quat = m_ReferencePose.orientation;
	
	// Convert quaternion to RPY.
	TooN::Vector<3> rpy_last = UTIL::QuatToRPY(quat_last);
	TooN::Vector<3> rpy = UTIL::QuatToRPY(quat);
    double yaw_diff = rpy[2] - rpy_last[2];
    
    geometry_msgs::Point pos_diff;
    pos_diff.x = position.x - position_last.x;
    pos_diff.y = position.y - position_last.y;
    pos_diff.z = position.z - position_last.z;
    
    double dis_diff = std::sqrt(pos_diff.x * pos_diff.x + pos_diff.y * pos_diff.y + pos_diff.z * pos_diff.z);
    
    if (dis_diff >= 1 || fabs(yaw_diff) >= 0.4) // 0.5 , 0.2
		return true;
	else
		return false;
}

void Mapping::StereoSetup(std::string camA, std::string camB)
{
	TooN::Matrix<> camMat = m_mCamMat.at(camA);
	cv::Mat camMatA = (cv::Mat_<double>(3,3) << camMat(0, 0), camMat(0, 1), camMat(0, 2),camMat(1, 0),camMat(1, 1),camMat(1, 2),camMat(2, 0),camMat(2, 1),camMat(2, 2));

	TooN::SE3<> camPose = m_mPoses.at(camA); // * m_se3IMUPose; // X_cam = camPose * X_imu

	//TooN::Matrix<> rot = camPoseget_rotation().get_matrix();
	//TooN::Vector<> tran =  camPoseget_translation();
	//cv::Mat PA1 = camMatA * (cv::Mat_<double>(3,4) << rot(0,0), rot(0,1), rot(0,2), tran[0], rot(1,0), rot(1,1), rot(1,2), tran[1], rot(2,0), rot(2,1), rot(2,2), tran[2]);
	TooN::Matrix<> rot = camPose.get_rotation().get_matrix();
	TooN::Vector<> tran = camPose.get_translation();
	cv::Mat PA = camMatA * (cv::Mat_<double>(3,4) << rot(0,0), rot(0,1), rot(0,2), tran[0], rot(1,0), rot(1,1), rot(1,2), tran[1], rot(2,0), rot(2,1), rot(2,2), tran[2]);
	
	m_mStereoPoses[camA] = PA;

	camMat = m_mCamMat.at(camB);
	cv::Mat camMatB = (cv::Mat_<double>(3,3) << camMat(0, 0), camMat(0, 1), camMat(0, 2),camMat(1, 0),camMat(1, 1),camMat(1, 2),camMat(2, 0),camMat(2, 1),camMat(2, 2));
	
	//rot = m_mPoses.at(camB).get_rotation().get_matrix();
	//tran = m_mPoses.at(camB).get_translation();
	//cv::Mat PB1 = camMatB * (cv::Mat_<double>(3,4) << rot(0,0), rot(0,1), rot(0,2), tran[0], rot(1,0), rot(1,1), rot(1,2), tran[1], rot(2,0), rot(2,1), rot(2,2), tran[2]);
	
	camPose = m_mPoses.at(camB); // * m_se3IMUPose;
	
	rot = camPose.get_rotation().get_matrix();
	tran =  camPose.get_translation();
	cv::Mat PB =  camMatB * (cv::Mat_<double>(3,4) << rot(0,0), rot(0,1), rot(0,2), tran[0], rot(1,0), rot(1,1), rot(1,2), tran[1], rot(2,0), rot(2,1), rot(2,2), tran[2]);
	
	m_mStereoPoses[camB] = PB;
}

cv::Mat Mapping::Stereo(std::string camA, std::string camB)
{
	cv::Mat descA = m_DescriptorList.at(camA).clone();
	cv::Mat descB = m_DescriptorList.at(camB).clone();
	std::vector<cv::KeyPoint> kpA = m_KeyPointList.at(camA);
	std::vector<cv::KeyPoint> kpB = m_KeyPointList.at(camB);
	
	cv::Mat PA = m_mStereoPoses[camA];
	cv::Mat PB = m_mStereoPoses[camB];
	
	//std::cout << "Original size: " << descA.rows << "x" << descA.cols << std::endl;
	cv::Mat Point4D = m_pImgProc->Stereo(descA, descB, kpA, kpB, PA, PB);
	//std::cout << "New size: " << descA.rows << "x" << descA.cols << std::endl;
	
	for (int i = 0; i < Point4D.rows; i++)
	{
		Point4D.row(i) = Point4D.row(i).mul(1/Point4D.row(3));
	}
	
	//std::cout << "PA type = " << PA.type() << std::endl;
	//std::cout << "Point type = " << Point4D.type() << std::endl;
	cv::Mat Point4D_new;
	Point4D.convertTo(Point4D_new, PA.type());
	
	cv::Mat rePointA = PA * Point4D_new;
	cv::Mat rePointB = PB * Point4D_new;
	
	for (int i = 0; i < rePointA.rows; i++)
	{
		rePointA.row(i) = rePointA.row(i).mul(1/rePointA.row(2));
	}	

	for (int i = 0; i < rePointB.rows; i++)
	{
		rePointB.row(i) = rePointB.row(i).mul(1/rePointB.row(2));
	}	
		
	std::cout << rePointA.t() << std::endl;
	std::cout << rePointB.t() << std::endl;
	
	m_pVocTree->SetVOData(m_nKeyframeIndex, descA, Point4D);
	
	return Point4D;
}

void Mapping::SaveImages(ImageBWMap imagesBW)
{
	static int nFrame = 0;
	
	std::string indexFrame = UTIL::ToString(nFrame);
	
	int nSub =1;
	for (ImageBWMap::iterator it = imagesBW.begin(); it != imagesBW.end(); it++)
	{
		std::string indexSubImage = UTIL::ToString(nSub);
		std::string strFileName = std::string("../Img_")  + indexFrame + std::string("_") + indexSubImage + std::string(".jpg");
		
		//try 
		//{
			cv::imwrite(strFileName, it->second);
		//}
		//catch (runtime_error& ex) 
		//{
			//fprintf(stderr, "Exception converting image to JPG format: %s\n", ex.what());
			//return 1;
		//}
		
		nSub++;
	}
	
	nFrame++;
}

void Mapping::RecordReferencePose()
{
	TooN::SE3<> se3Pose = UTIL::PoseMsgToSE3(m_ReferencePose);
	TooN::Vector<6> v6Pose = UTIL::SE3Tov6Pose(se3Pose);
	
	cv::Mat temp = (cv::Mat_<double>(1, 7) << m_nKeyframeIndex, v6Pose[0], v6Pose[1], v6Pose[2], v6Pose[3], v6Pose[4], v6Pose[5]);
	if (m_mPoseRecord.empty())
		temp.copyTo(m_mPoseRecord);
	else
		m_mPoseRecord.push_back(temp);
}

void Mapping::SetStereoNames(std::vector<std::string> vStereoNames)
{
	assert(vStereoNames.size() == 2);
	m_vStereoNames = vStereoNames;
	StereoSetup(m_vStereoNames[0], m_vStereoNames[1]);
}

bool Mapping::getRT(const TooN::Matrix<3>& E, std::vector< TooN::Matrix<3> > & R, TooN::Vector<3>& t)
{
	bool bSuccess = true;
	
	TooN::SVD<3> svdE(E);
	
	TooN::Matrix<> U = svdE.get_U();
	TooN::Matrix<> VT = svdE.get_VT();
	
	if (TooN::determinant(U*VT.T()) < 0)
	{
		VT[2] = -VT[2];
	}
	
	TooN::Matrix<3> W = TooN::Data(0, -1.0, 0, 1.0, 0, 0, 0, 0, 1.0);
	
	R.push_back( U*W*VT );
	R.push_back( U*W.T()*VT);
	t = TooN::makeVector(U[0][2], U[1][2], U[2][2]); 
	
	return bSuccess;	
}

int Mapping::VerifyRT(const TooN::Matrix<3> R, const TooN::Vector<3> t, const std::vector< TooN::Vector<3> > imgPoints_1, const std::vector< TooN::Vector<3> > imgPoints_2)
{
	assert(imgPoints_1.size() == imgPoints_2.size());
	
	int nPointsInFront = 0;
	
	for (int i = 0; i < imgPoints_1.size(); i++)
	{
		TooN::Vector<3> p1 = imgPoints_1[i];
		TooN::Vector<3> p2 = imgPoints_2[i];
		
		if(isPointInFront(R, t, p1, p2))
			nPointsInFront++;
	}
	
	return nPointsInFront;
}

void Mapping::getEssential(const std::vector<cv::Point2f>& vPoints_1, const std::vector<cv::Point2f>& vPoints_2, const TooN::Matrix<3> camMat_1, const TooN::Matrix<3> camMat_2, TooN::Matrix<3>& E, cv::Mat& mPointStatus)
{
	assert(vPoints_1.size() == vPoints_2.size());
	
	//cv::Mat mPointStatus; // same size as kpmatched_last, 0: outlier, 1: inlier
	cv::Mat fundamental_matrix = cv::findFundamentalMat(vPoints_1, vPoints_2, cv::FM_RANSAC, 3, 0.99, mPointStatus);
    
    //std::cout << "F = " << fundamental_matrix << std::endl;
    
    std::vector<cv::Point2f> kpinlier_1, kpinlier_2;

	for(size_t ip = 0; ip < vPoints_1.size(); ip++)
	{
		if (mPointStatus.at<bool>(ip) == true)
		{
			kpinlier_1.push_back(vPoints_1[ip]);
			kpinlier_2.push_back(vPoints_2[ip]);
		}
	}
	
	fundamental_matrix = cv::findFundamentalMat(kpinlier_1, kpinlier_2, cv::FM_8POINT); 
	
	//std::cout << "F = " << fundamental_matrix << std::endl;;
 
	TooN::Matrix<3> F = UTIL::cvMat3x3ToTooNMat(fundamental_matrix);
	
	//std::cout << "F = " << F << std::endl;;
	
	//std::cout << "K = " << camMat_1 << std::endl;
	
	// essential matrix
	E = camMat_2.T() * F * camMat_1;
	
	//double sum_s = 0;
	
	//for (int i = 0; i < 3; i++)
	//{
		//for (int j= 0; j < 3; j++)
		//{
			//sum_s = sum_s + E[i][j] * E[i][j];
		//}
	//}
	
	//E = E / sqrt(sum_s);
}
