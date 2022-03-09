#include <toposlam/ImageProcess.h>
#include <assert.h>     /* assert */

ImageProcess::ImageProcess()
{
}

ImageProcess::ImageProcess(TaylorCameraMap CameraModels, std::string strDetector, std::string strDescriptor, int nFeatureNumber, int nThreshold)
: m_mCameraModels(CameraModels)
, m_strDetectorType(strDetector)
, m_strDescriptorType(strDescriptor)
, m_nFeatureNumber(nFeatureNumber)
, m_nFeatureThreshold(nThreshold)
{
	ROS_INFO_STREAM("Image processing: feature detector: " << m_strDetectorType << ", feature descriptor: " << m_strDescriptorType << ", feature number per image: " << m_nFeatureNumber);
	if (m_strDetectorType == "FAST" || m_strDetectorType == "fast")
	{
		m_pFeatureDector = new cv::FastAdjuster(m_nFeatureThreshold, true);
		m_pDynamicDetector = new cv::DynamicAdaptedFeatureDetector(m_pFeatureDector, (int)(m_nFeatureNumber*0.9), (int)(m_nFeatureNumber*1.1), 50);  
	}
	else if (m_strDetectorType == "CENSURE" || m_strDetectorType == "censure")
	{
		m_pFeatureDector = new cv::StarAdjuster(m_nFeatureThreshold);	     
		m_pDynamicDetector = new cv::DynamicAdaptedFeatureDetector(m_pFeatureDector, (int)(m_nFeatureNumber*0.9), (int)(m_nFeatureNumber*1.1), 50);     
	}
	else
	{
		ROS_ERROR_STREAM("Image Process: Feature detector must be: FAST or CENSURE ... ");
		exit(0);
	}
	
	if (m_strDescriptorType == "SIFT" || m_strDescriptorType == "sift" || m_strDescriptorType == "SURF" || m_strDescriptorType == "surf")
	{
		cv::initModule_nonfree();
		m_pDescriptorExtractor = cv::DescriptorExtractor::create(m_strDescriptorType);
		//std::cout << m_strDescriptorType  << " descripot defined!" << std::endl;
	}
	else
	{
		ROS_ERROR_STREAM("Image Process: Feature descriptor must be: SIFT or SURF ... ");
		exit(0);		
	}
	
	for (TaylorCameraMap::iterator it = m_mCameraModels.begin(); it != m_mCameraModels.end(); ++it)
	{
		cv::Size irImageSize;
		irImageSize.width = (it->second).GetSize()[0];
		irImageSize.height = (it->second).GetSize()[1];
		m_mImageSizes[it->first] = irImageSize;
	}
	
	InitUndistort();
}

ImageProcess::~ImageProcess()
{
	if (m_pFeatureDector !=  NULL) {delete m_pFeatureDector; }
	//if (m_pDescriptorExtractor !=  NULL) {delete m_pDescriptorExtractor; }
	if (m_pDynamicDetector !=  NULL) {delete m_pDynamicDetector; }
}

// find mapping matrices for a given size of the mapped image
void ImageProcess::InitUndistort( )
{
	assert(!m_mCameraModels.empty());
	
	Corner2DMap vPixelCorners;
	Corner3DMap vImgCorners;
	FindCameraMatrix(m_mImageSizes, vPixelCorners, vImgCorners, m_mCamMat);
	FindUndistortMapping(m_mCamMat, vImgCorners, m_mMapX, m_mMapY);
}

// Find the camera matrix to map a distorted image to a undistored image OF a specified resolution
// It also finds the corner of the mapped image
// Input: 
// cameraModel: models of each camera
// imgSize: specified size of undistored image
// vPixelCorners: pixel coordinates of the corners of the area of the original image that is being mapped
// vImgCorners: coordinate of the mapped image, one dimension may be smalled than the specified imgSize
// camMatMap: camera matrices for the specified mapping
void ImageProcess::FindCameraMatrix(ImageSizeMap mImageSizes, Corner2DMap& vPixelCorners, Corner3DMap& vImgCorners, CameraMatrixMap& camMatMap)
{
	FindCameraMatrix(m_mCameraModels, mImageSizes, vPixelCorners, vImgCorners, camMatMap);
}

void ImageProcess::FindCameraMatrix(TaylorCameraMap cameraModel, ImageSizeMap mImageSizes, Corner2DMap& vPixelCorners, Corner3DMap& vImgCorners, CameraMatrixMap& camMatMap)
{	
	for(TaylorCameraMap::iterator it = cameraModel.begin(); it != cameraModel.end(); ++it) 
	{
		std::string strCamName = it->first;

		std::cout << strCamName << ": "<< std::endl;
		
		cv::Size imgSize = mImageSizes.at(strCamName);
		
		TooN::Vector<2> srcCenter = it->second.GetCenter(); // get the center of the original image
		TooN::Vector<2> srcSize = it->second.GetSize(); // get the size of the original image
		TooN::Vector<2> offset = srcSize - srcCenter; 
		double roi = std::min(std::min(srcCenter[0], srcCenter[1]), std::min(double(offset[0]), offset[1]));  // find the shortest distance from the center to four edges of the original image
		
		if (roi < 0) { roi = 0; }
		
		//Four corners of ROI of the original image (on sensor)
		vPixelCorners.clear();
		
		TooN::Matrix<4, 2> pCorners; // find the largest square centered at the original center within the original image
		TooN::Matrix<4, 3> mCorners;
		double AoV = 2.5; // Angle of view of the mapped image
		
		while (AoV >= 2.0)
		{
			pCorners[0] = srcCenter + TooN::makeVector(-roi, -roi);
			pCorners[1] = srcCenter + TooN::makeVector(roi, -roi);
			pCorners[2] = srcCenter + TooN::makeVector(roi, roi);
			pCorners[3] = srcCenter + TooN::makeVector(-roi, roi);
			vPixelCorners[strCamName] = pCorners;
			
			//std::cout << "Pixel Coners: " << std::endl;
			//std::cout << pCorners;
			//std::cout << "--------------------" << std::endl;
			
			AoV = 0.0;
			for (unsigned i = 0; i < 4; i++)
			{
				TooN::Vector<3> rayInCam = it->second.UnProject(pCorners[i]);
				
				AoV += 2.0 * std::atan(std::sqrt(rayInCam[0]*rayInCam[0] + rayInCam[1]*rayInCam[1])/rayInCam[2]);
				
				//std::cout << "AOV= " << AoV << std::endl;
				
	            mCorners[i] = rayInCam;
	            
				//std:: cout << "From (" << pCorners[i] << ") to (" << rayInCam << ")" << std::endl;
	        
	            //TooN::Vector<2> pixelPoint = it->second.Project(rayInCam);
	            //std:: cout << "From (" << rayInCam << ") back to (" << pixelPoint << ")" << std::endl;
			}
			
			AoV /= 4.0; //average AoV of four corners
			roi -= 5; //shrink the square in the original image
		}
		
		double LU_x = std::min(mCorners(0, 0), mCorners(3, 0));
		double LU_y = std::min(mCorners(0, 1), mCorners(1, 1));
		double RB_x = std::max(mCorners(1, 0), mCorners(2, 0));
		double RB_y = std::max(mCorners(2, 1), mCorners(3, 1));
		double width = RB_x - LU_x;
		double height = RB_y - LU_y;
		
		//std::cout << "H:W = " << height << ":" << width << " =  "<< imgSize.height << ":" << imgSize.height*width/height << std::endl;
		double focal = std::min(imgSize.height / height, imgSize.width / width) * (mCorners(0, 2));
		
		TooN::Matrix<3, 3> cameraMatrix = TooN::Data(focal,     0, -focal*LU_x,
														  0, focal, -focal*LU_y,
														  0,	 0,	          1);
														 
		camMatMap[strCamName] = cameraMatrix;
		//std::cout << "Camera matrix: " << std::endl;
		//std::cout << cameraMatrix;
		//std::cout << "--------------------" << std::endl;
		
		mCorners(0, 0) = LU_x; mCorners(0, 1) = LU_y;
		mCorners(1, 0) = RB_x; mCorners(1, 1) = LU_y;
		mCorners(2, 0) = RB_x; mCorners(2, 1) = RB_y;
		mCorners(3, 0) = LU_x; mCorners(3, 1) = RB_y;
		
		mCorners = mCorners * cameraMatrix.T();
		
		//std::cout << "Image corners: " << std::endl;
		//std::cout << mCorners;
		//std::cout << "--------------------" << std::endl;
		
		vImgCorners[strCamName] = mCorners;
	}	
}

void ImageProcess::FindUndistortMapping(CameraMatrixMap camMatMap, Corner3DMap corners, ProjectMap& mapx, ProjectMap& mapy)
{
	FindUndistortMapping(m_mCameraModels, camMatMap, corners, mapx, mapy);
}

void ImageProcess::FindUndistortMapping(TaylorCameraMap cameraModel, CameraMatrixMap camMatMap, Corner3DMap corners, ProjectMap& mapx, ProjectMap& mapy)
{
	for(Corner3DMap::iterator it = corners.begin(); it != corners.end(); ++it)
	{
		int totalcols = static_cast<int> (it->second(1, 0) - it->second(0, 0));
		int totalRows = static_cast<int> (it->second(3, 1) - it->second(0, 1));
		
		cv::Mat map_x(totalRows, totalcols, CV_32FC1);
		cv::Mat map_y(totalRows, totalcols, CV_32FC1);
		
		TooN::Matrix<3, 3> cameraMatrix = camMatMap.at(it->first);
		TooN::Matrix<3, 3> cameraMatInv = TooN::Data(1.0/cameraMatrix(0,0), 0.0, -cameraMatrix(0,2)/cameraMatrix(0,0),
													 0.0, 1.0/cameraMatrix(1,1), -cameraMatrix(1,2)/cameraMatrix(1,1),
													 0.0, 0.0, 1.0);

		for (int nRows = 0; nRows < totalRows; nRows++)
		{
			for(int nCols = 0; nCols < totalcols; nCols++)
			{
				TooN::Vector<3> camPoint = cameraMatInv * TooN::makeVector(nCols, nRows, 1) ;
				TooN::Vector<2> piexlPoint = cameraModel.at(it->first).Project(camPoint);
				
				map_x.at<float>(nRows, nCols) = piexlPoint[0] ;
                map_y.at<float>(nRows, nCols) = piexlPoint[1] ;                     
			}
		}
		
		map_x.copyTo(mapx[it->first]);
		map_y.copyTo(mapy[it->first]); 
	}
}

void ImageProcess::FASTExtractor(cv::Mat img_in, cv::Mat img_mask, std::vector<cv::KeyPoint>& keypoints, cv::Mat& descriptors, int nDesc, int threshold)
{
    //will create a FAST detector that attempts to find
    //nDesc  Keypoints, and will at most run
    //FAST feature detection 50 times until that
    //number of keypoints are found
        
    cv::DynamicAdaptedFeatureDetector detector(new cv::FastAdjuster(threshold, true), (int)(nDesc*0.9), (int)(nDesc*1.1), 50);                          
    detector.detect(img_in, keypoints, img_mask);
    //std::cout << "image size: " << img_in.rows << " x " << img_in.cols << std::endl;
    //std::cout << "Feature size: " << keypoints.size() << std::endl;
	m_pDescriptorExtractor->compute(img_in, keypoints, descriptors);
}

void ImageProcess::STARExtractor(cv::Mat img_in, cv::Mat img_mask, std::vector<cv::KeyPoint>& keypoints, cv::Mat& descriptors, int nDesc, int threshold)
{    
    //will create a Star (CENSURE) detector that attempts to find
    //nDesc  Keypoints, and will at most run
    //CENSURE feature detection 50 times until that
    //number of keypoints are found
   
    cv::DynamicAdaptedFeatureDetector detector(new cv::StarAdjuster(threshold), (int)(nDesc*0.9), (int)(nDesc*1.1), 50);                       
    detector.detect(img_in, keypoints, img_mask);
    
	m_pDescriptorExtractor->compute(img_in, keypoints, descriptors);	
}

void ImageProcess::FeatureExtractor(cv::Mat img_in, cv::Mat img_mask, std::vector<cv::KeyPoint>& keypoints, cv::Mat& descriptors)
{
	m_pDynamicDetector->detect(img_in, keypoints, img_mask);
	m_pDescriptorExtractor->compute(img_in, keypoints, descriptors);
}

void ImageProcess::FeatureMatching(const cv::Mat& queryDesc, const cv::Mat& trainDesc, std::vector< cv::DMatch >& goodMatch, double& dSimilarity)
{
	cv::FlannBasedMatcher FlannMatcher;
	std::vector< std::vector< cv::DMatch > > knnMatches;
	std::vector< cv::DMatch > matches_1;
	std::vector< cv::DMatch > matches_2;
	
	FlannMatcher.knnMatch(queryDesc, trainDesc, knnMatches, 2); // for each query descriptor, find two matching featuers from the train descriptors
	
	for(std::vector< std::vector< cv::DMatch > >::iterator it = knnMatches.begin(); it != knnMatches.end(); ++it)
	{
		if(it->at(0).distance < (0.7*(it->at(1).distance)))
			matches_1.push_back(it->at(0));
	}
	
	knnMatches.clear();
	FlannMatcher.knnMatch(trainDesc, queryDesc, knnMatches, 2); // for each Train descriptor, find two matching featuers from the Query descriptors
	
	for(std::vector< std::vector< cv::DMatch > >::iterator it = knnMatches.begin(); it != knnMatches.end(); ++it)
	{
		if(it->at(0).distance < (0.7*(it->at(1).distance)))
			matches_2.push_back(it->at(0));
	}	
	
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
	
	// dSimilarity = 2.0 * goodMatch.size() /(queryDesc.rows + trainDesc.rows);
	dSimilarity =  double (goodMatch.size()) /std::min(queryDesc.rows, trainDesc.rows);
}

cv::Mat ImageProcess::Stereo(cv::Mat& DescA, cv::Mat& DescB, std::vector<cv::KeyPoint>& PointsA, std::vector<cv::KeyPoint>& PointsB, const cv::Mat& PA, const cv::Mat& PB) 
{
	std::vector< cv::DMatch > goodMatch;
	double dSimilarity;
	FeatureMatching(DescA, DescB, goodMatch, dSimilarity);

	cv::Mat desc_A, desc_B;
	std::vector<cv::KeyPoint> rekp_A, rekp_B;
	std::vector<cv::Point2f> PointsA_matched, PointsB_matched;
	
	for( size_t i = 0 ; i <  goodMatch.size(); i++) 
	{
		if(desc_A.empty())
			DescA.row(goodMatch[i].queryIdx).copyTo(desc_A);
		else
			desc_A.push_back(DescA.row(goodMatch[i].queryIdx));
			
		if(desc_B.empty())	
			DescB.row(goodMatch[i].trainIdx).copyTo(desc_B);
		else
			desc_B.push_back(DescB.row(goodMatch[i].trainIdx));				
		
		rekp_A.push_back(PointsA[ goodMatch[i].queryIdx ]); 
		rekp_B.push_back(PointsB[ goodMatch[i].trainIdx ]);
		
		cv::Point2f Point2A, Point2B;
		Point2A = PointsA[ goodMatch[i].queryIdx ].pt;
		Point2B = PointsB[ goodMatch[i].trainIdx ].pt;
		PointsA_matched.push_back(Point2A);
		PointsB_matched.push_back(Point2B);
	}	
	
	cv::Mat Point4D;
	cv::triangulatePoints(PA, PB, PointsA_matched, PointsB_matched, Point4D);
	
	DescA = desc_A;
	DescB = desc_B;
	
	PointsA.clear();
	PointsB.clear();
	PointsA = rekp_A;
	PointsB = rekp_B;
	
	return Point4D;
}	



//std::vector< TooN::Vector<3> > ImageProcess::Stereo(const cv::Mat& DescA, const cv::Mat& DescB, std::vector<cv::KeyPoint>& PointsA, std::vector<cv::KeyPoint>& PointsB, const TooN::Matrix<3,3>& RA2B, const TooN::Vector<3>& tA2B, const TooN::Matrix<3,3>& mCamMatInvA, const TooN::Matrix<3,3>& mCamMatInvB)
//{
	//std::vector< cv::DMatch > goodMatch;
	//double dSimilarity;
	//FeatureMatching(DescA, DescB, goodMatch, dSimilarity);
	
	//std::vector<cv::Point2f> PointsA_matched, PointsB_matched;
	//std::vector< TooN::Vector<3> > Points3A, Points3B;
	//for( size_t i = 0 ; i <  goodMatch.size(); i++) 
	//{
		//cv::Point2f Point2A, Point2B;
		//Point2A = PointsA[ goodMatch[i].queryIdx ].pt;
		//Point2B = PointsB[ goodMatch[i].trainIdx ].pt;
		//PointsA_matched.push_back(Point2A);
		//PointsB_matched.push_back(Point2B);
		
		//TooN::Vector<3> PointImg = TooN::makeVector(Point2A.x, Point2A.y, 1);			
		//TooN::Vector<3> PointCam_A = mCamMatInvA * PointImg;
		//PointImg = TooN::makeVector(Point2B.x, Point2B.y, 1);			
		//TooN::Vector<3> PointCam_B = mCamMatInvB * PointImg;	
		//Points3A.push_back(PointCam_A);
		//Points3B.push_back(PointCam_B);	
	//}
	
	//std::vector< TooN::Vector<3> > vPoints;
	
	//for (size_t i = 0; i < Points3A.size(); i++)
	//{
		//TooN::Vector<3> Point3D;
		//if ( Find3DPoint(RA2B, tA2B, Points3A[i], Points3B[i], Point3D) )
		//{
			//vPoints.push_back(Point3D);
		//}
	//}
	
	//return vPoints;
//}

bool ImageProcess::Find3DPoint(const TooN::Matrix<3,3>& RA2B, const TooN::Vector<3>& tA2B, const TooN::Vector<3>& v3A, const TooN::Vector<3>& v3B, TooN::Vector<3>& Point3D)
{
	TooN::Vector<3> u = RA2B * v3A;
	TooN::Vector<3> v = v3B;
	
	double a = u * u;
	double b = u * v;
	double c = v * v;
	double d = u * tA2B;
	double e = v * tA2B;
	
	double dDenom = (-a*c + b*b);
	
	if (dDenom == 0)
	{
		std::cout << "dDenom = " << dDenom << std::endl;
		return false;
	}
	//assert(dDenom != 0);
	//if(dDenom == 0)
		//return std::numeric_limits<double>::infinity();
	
	double ku = (c*d - b*e)/dDenom;
	double kv = (b*d - a*e)/dDenom;
    
	//std::cout << "dDenom = " << dDenom << std::endl;
    //std::cout << "ku = " << ku << ", kv = " << kv  << std::endl;
	//std::cout << "Intersection = " << ku*v3A << " : " << kv * v << std::endl;
	
	Point3D = ku*v3A;
	
	if ( ku > 0 && kv > 0 )
		return true;
	else
		return false;
}
