#include <toposlam/VisualOdometry.h>
#include <opencv2/nonfree/features2d.hpp>
#include <opencv2/nonfree/nonfree.hpp>
//#include <time.h>       /* time */
#include <stdio.h>
#include <iostream>
#include <time.h>

int nFeatureThreshold = 70;
int nFeatureNumber = 300;
std::string strDescriptorType = "SIFT"; //SURF

//TooN::Vector<3> getRandomPoint()
//{
    //int x = rand()%100 - 50;
    //int y = rand()%100 - 50;
    //int z = rand()%20 + 30;
    
    //TooN::Vector<3> v3dP = TooN::makeVector(double(x), double(y), double(z));
    
    //return v3dP;
//};

//int main(int argc, char **argv)
//{
    //srand (time(NULL)); /* initialize random seed: */
    
    //VisualOdometry::Calibration leftCalib(645.24, 635.96, 194.13); // f, cu, cv
    //VisualOdometry::Calibration rightCalib(645.24, 635.96, 194.13); // f, cu, cv
    //TooN::Matrix<3> rot = TooN::Identity(3);
    //TooN::SE3<> se3LeftCam = TooN::SE3<> (TooN::SO3<>(rot), TooN::makeVector(0, 0, 0));
    //TooN::SE3<> se3RightCam = TooN::SE3<> (TooN::SO3<>(rot), TooN::makeVector(0.5707, 0, 0));
    
    //VisualOdometry::Parameters param(true, 100, 2.0); // bReweighting, RANSAC_iter, inliner_threshold
    
	//VisualOdometry vo(leftCalib, rightCalib, se3LeftCam, se3RightCam, param);
    
    ////transformation from left to right
    //TooN::SE3<> se3TL2R(TooN::SO3<>(rot), TooN::makeVector(-0.5707, 0, 0));
    
    
    //double rx = 0.1;
    //double ry = -0.1; 
    //double rz = 0.2;
    
    //TooN::Matrix<3> Rot, Rx, Ry, Rz;
    
    //vo.getRotMatAndDerivative(rx, ry, rz, Rot, Rx, Ry, Rz);
    
    
    ////transformation from previous to current
    //TooN::SE3<> se3TP2C(TooN::SO3<>(Rot), TooN::makeVector(0.3, -0.2, 0.5));
    
    //std::vector< TooN::Vector<3> > v3dPInPreviousLeft;
    //std::vector< TooN::Vector<3> > v3dPInPreviousRight;
    
    //std::vector< TooN::Vector<3> > v3dPInCurrentLeft;
    //std::vector< TooN::Vector<3> > v3dPInCurrentRight;
    
    //std::vector< TooN::Vector<2> > v2dPInPreviousLeft;
    //std::vector< TooN::Vector<2> > v2dPInPreviousRight;
    
    //std::vector< TooN::Vector<2> > v2dPInCurrentLeft;
    //std::vector< TooN::Vector<2> > v2dPInCurrentRight;
    
    //for(int i = 0; i < 100; i++)
    //{
        //TooN::Vector<3> v3dP_PL = getRandomPoint(); // previous left
        
        //std::cout << "p[" << i << "] = " << v3dP_PL << std::endl;
        
        //TooN::Vector<3> v3dP_PR = se3TL2R * v3dP_PL; // previous right
        
        //TooN::Vector<3> v3dP_CL = se3TP2C * v3dP_PL; // current right
        //TooN::Vector<3> v3dP_CR = se3TL2R * v3dP_CL; // current right
        
        //v3dPInPreviousLeft.push_back(v3dP_PL);
        //v3dPInPreviousRight.push_back(v3dP_PR);
        //v3dPInCurrentLeft.push_back(v3dP_CL);
        //v3dPInCurrentRight.push_back(v3dP_CR);
        
        //TooN::Vector<2> v2dP_PL = vo.Project(v3dP_PL, vo.m_sLeftCalib);
        //TooN::Vector<2> v2dP_PR = vo.Project(v3dP_PR, vo.m_sRightCalib);
        
        //TooN::Vector<2> v2dP_CL = vo.Project(v3dP_CL, vo.m_sLeftCalib);
        //TooN::Vector<2> v2dP_CR = vo.Project(v3dP_CR, vo.m_sRightCalib);       
        
        //v2dPInPreviousLeft.push_back(v2dP_PL);
        //v2dPInPreviousRight.push_back(v2dP_PR);
        //v2dPInCurrentLeft.push_back(v2dP_CL);
        //v2dPInCurrentRight.push_back(v2dP_CR); 
        
        //VisualOdometry::CrossMatches quadMatch;
		//quadMatch.vPointLeftPrevious.x = v2dP_PL[0];
        //quadMatch.vPointLeftPrevious.y = v2dP_PL[1];
		//quadMatch.vPointRightPrevious.x = v2dP_PR[0];
        //quadMatch.vPointRightPrevious.y = v2dP_PR[1];
	    //quadMatch.vPointLeftCurrent.x = v2dP_CL[0];
        //quadMatch.vPointLeftCurrent.y = v2dP_CL[1];
	    //quadMatch.vPointRightCurrent.x = v2dP_CR[0];
        //quadMatch.vPointRightCurrent.y = v2dP_CR[1];
        
	    //quadMatch.v3dPointPrevious = v3dP_PL;
		//vo.m_vCrossMatches.push_back(quadMatch);
    //}
    
    //vo.process();
    
    //return 0;
//}

//int main(int argc, char **argv)
//{
    //VisualOdometry::Calibration leftCalib(645.24, 635.96, 194.13); // f, cu, cv
    //VisualOdometry::Calibration rightCalib(645.24, 635.96, 194.13); // f, cu, cv
    //TooN::Matrix<3> rot = TooN::Identity(3);
    //TooN::SE3<> se3LeftCam = TooN::SE3<> (TooN::SO3<>(rot), TooN::makeVector(0, 0, 0));
    //TooN::SE3<> se3RightCam = TooN::SE3<> (TooN::SO3<>(rot), TooN::makeVector(0.5707, 0, 0));
    ////TooN::SE3<> se3TL2R(TooN::SO3<>(rot), TooN::makeVector(-0.5707, 0, 0));
    //VisualOdometry::Parameters param(true, 100, 2.0); // bReweighting, RANSAC_iter, inliner_threshold
    
	//VisualOdometry vo(leftCalib, rightCalib, se3LeftCam, se3RightCam, param);
    
    //cv::AdjusterAdapter* pFeatureDector = new cv::StarAdjuster(nFeatureThreshold);	     
    //cv::DynamicAdaptedFeatureDetector* pDynamicDetector = new cv::DynamicAdaptedFeatureDetector(pFeatureDector, (int)(nFeatureNumber*0.9), (int)(nFeatureNumber*1.1), 50);     
    
    //cv::initModule_nonfree();
    //cv::Ptr<cv::DescriptorExtractor> pDescriptorExtractor;
    
    //pDescriptorExtractor = cv::DescriptorExtractor::create(strDescriptorType);
    
    //std::string strImgDir = "/home/mingfeng/catkin_ws/src/toposlam/image/";
    //// default names of cameras
    //const char *sImgNames[] = {"I1_000040", "I2_000040", "I1_000041", "I2_000041"};
	//std::vector<std::string> vImgNames(sImgNames, sImgNames+4);
    
    //std::string strLeftImgName = strImgDir + vImgNames[0] + ".png";
    //std::string strRightImgName = strImgDir + vImgNames[1] + ".png";
    //cv::Mat img_left = cv::imread(strLeftImgName, CV_LOAD_IMAGE_GRAYSCALE);
    //cv::Mat img_right = cv::imread(strRightImgName, CV_LOAD_IMAGE_GRAYSCALE);
    
    //std::vector<cv::KeyPoint> keypoints_left, keypoints_right;
    //cv::Mat descriptors_left, descriptors_right;
    
    //pDynamicDetector->detect(img_left, keypoints_left, cv::Mat());
    //pDescriptorExtractor->compute(img_left, keypoints_left, descriptors_left);
    
    //pDynamicDetector->detect(img_right, keypoints_right, cv::Mat());
    //pDescriptorExtractor->compute(img_right, keypoints_right, descriptors_right);
    
    //std::vector< cv::DMatch > goodMatch;
    //vo.matchFeatures(descriptors_left, descriptors_right, goodMatch);
    
    //std::vector<cv::Point2f> matched_kp_left, matched_kp_right;
    //cv::Mat mateched_desc_left, matched_desc_right;
    
    ////std::cout << "image 2d points" << std::endl;
    ////std::cout << "Left image       Right image" << std::endl;
    //for(std::vector< cv::DMatch >::iterator it = goodMatch.begin(); it != goodMatch.end(); ++it)
    //{
        //matched_kp_left.push_back(keypoints_left[it->queryIdx].pt);
        //if(mateched_desc_left.empty())
            //descriptors_left.row(it->queryIdx).copyTo(mateched_desc_left);
        //else
            //mateched_desc_left.push_back(descriptors_left.row(it->queryIdx));
            
        //matched_kp_right.push_back(keypoints_right[it->trainIdx].pt);
        //if(matched_desc_right.empty())
            //descriptors_right.row(it->trainIdx).copyTo(matched_desc_right);
        //else
            //matched_desc_right.push_back(descriptors_right.row(it->trainIdx));
            
        ////std::cout << "(" << keypoints_left[it->queryIdx].pt.x << ", " << keypoints_left[it->queryIdx].pt.y <<
                ////")  (" << keypoints_right[it->trainIdx].pt.x << ", " <<  keypoints_right[it->trainIdx].pt.y << ")" << std::endl;
    //}
    
    ////std::cout << std::endl;
    
    //cv::Mat img_matches;
    //cv::drawMatches(img_left, keypoints_left, img_right, keypoints_right,
               //goodMatch, img_matches, cv::Scalar::all(-1), cv::Scalar::all(-1),
               //std::vector<char>(), cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );
               
     ////-- Show detected matches
    //cv::imshow( "Good Matches & Object detection", img_matches );

    //cv::waitKey(0);
    
    //vo.setCurrentKeypoints(matched_kp_left, mateched_desc_left, matched_kp_right, matched_desc_right);
    
    //vo.replaceKeypoints();
    
    //strLeftImgName = strImgDir + vImgNames[2] + ".png";
    //strRightImgName = strImgDir + vImgNames[3] + ".png";
    //cv::Mat new_img_left = cv::imread(strLeftImgName, CV_LOAD_IMAGE_GRAYSCALE);
    //cv::Mat new_img_right = cv::imread(strRightImgName, CV_LOAD_IMAGE_GRAYSCALE);
    
    //keypoints_left.clear();
    //keypoints_right.clear();
    //descriptors_left.release();
    //descriptors_right.release();
    
    //pDynamicDetector->detect(new_img_left, keypoints_left, cv::Mat());
    //pDescriptorExtractor->compute(new_img_left, keypoints_left, descriptors_left);
    
    //pDynamicDetector->detect(new_img_right, keypoints_right, cv::Mat());
    //pDescriptorExtractor->compute(new_img_right, keypoints_right, descriptors_right);
    
    //std::cout << "left desc size = " << descriptors_left.size() << ", right desc size = " << descriptors_right.size() << std::endl;
    //goodMatch.clear();
    //vo.matchFeatures(descriptors_left, descriptors_right, goodMatch);
    
    
    //std::cout << "Match size: " << goodMatch.size() << std::endl;
    
    ////std::vector<cv::Point2f> matched_kp_left, matched_kp_right;
    ////cv::Mat mateched_desc_left, matched_desc_right;
    
    //matched_kp_left.clear();
    //matched_kp_right.clear();
    
    //mateched_desc_left.release();
    //matched_desc_right.release();
    
    ////std::cout << "image 2d points" << std::endl;
    ////std::cout << "Left image       Right image" << std::endl;
    //for(std::vector< cv::DMatch >::iterator it = goodMatch.begin(); it != goodMatch.end(); ++it)
    //{
        //matched_kp_left.push_back(keypoints_left[it->queryIdx].pt);
        //if(mateched_desc_left.empty())
            //descriptors_left.row(it->queryIdx).copyTo(mateched_desc_left);
        //else
            //mateched_desc_left.push_back(descriptors_left.row(it->queryIdx));
            
        //matched_kp_right.push_back(keypoints_right[it->trainIdx].pt);
        //if(matched_desc_right.empty())
            //descriptors_right.row(it->trainIdx).copyTo(matched_desc_right);
        //else
            //matched_desc_right.push_back(descriptors_right.row(it->trainIdx));
            
        ////std::cout << "(" << keypoints_left[it->queryIdx].pt.x << ", " << keypoints_left[it->queryIdx].pt.y <<
                ////")  (" << keypoints_right[it->trainIdx].pt.x << ", " <<  keypoints_right[it->trainIdx].pt.y << ")" << std::endl;
    //}
    
    
    //std::cout << "Size: " << matched_kp_left.size() << std::endl;
    
    //vo.setCurrentKeypoints(matched_kp_left, mateched_desc_left, matched_kp_right, matched_desc_right);
    
    //vo.process();

    //if(!pFeatureDector)
        //delete pFeatureDector;
        
    //if(!pDynamicDetector)
        //delete pDynamicDetector;
        
    ////if(!pDescriptorExtractor)
        ////delete pDescriptorExtractor;
        
	//return 0;
//}
timespec tIni, tEnd;

int main(int argc, char **argv)
{
    VisualOdometry::Calibration leftCalib(645.24, 635.96, 194.13); // f, cu, cv
    VisualOdometry::Calibration rightCalib(645.24, 635.96, 194.13); // f, cu, cv
    TooN::Matrix<3> rot = TooN::Identity(3);
    TooN::SE3<> se3LeftCam = TooN::SE3<> (TooN::SO3<>(rot), TooN::makeVector(0, 0, 0));
    TooN::SE3<> se3Pose = se3LeftCam;
    
    TooN::SE3<> se3RightCam = TooN::SE3<> (TooN::SO3<>(rot), TooN::makeVector(0.5707, 0, 0));
    //TooN::SE3<> se3TL2R(TooN::SO3<>(rot), TooN::makeVector(-0.5707, 0, 0));
    VisualOdometry::Parameters param(true, 100, 2.0); // bReweighting, RANSAC_iter, inliner_threshold
    
	VisualOdometry vo(leftCalib, rightCalib, se3LeftCam, se3RightCam, param);
    
    cv::AdjusterAdapter* pFeatureDector = new cv::StarAdjuster(nFeatureThreshold);	     
    cv::DynamicAdaptedFeatureDetector* pDynamicDetector = new cv::DynamicAdaptedFeatureDetector(pFeatureDector, (int)(nFeatureNumber*0.9), (int)(nFeatureNumber*1.1), 50);     
    
    cv::initModule_nonfree();
    cv::Ptr<cv::DescriptorExtractor> pDescriptorExtractor;
    
    pDescriptorExtractor = cv::DescriptorExtractor::create(strDescriptorType);
    
    std::string strImgDir = "/home/mingfeng/catkin_ws/src/toposlam/image/";
    // default names of cameras
    const char *sImgNames[] = {"I1_", "I2_"};
	std::vector<std::string> vImgNames(sImgNames, sImgNames+2);
    
    double aveTime = 0;
    double maxTime = 0;
    int nTotalImg = 373;
    
    for(int nImg = 0; nImg < nTotalImg; nImg++)
    {
        char base_name[256]; 
        sprintf(base_name, "%06d.png", nImg);
        
        std::string strLeftImgName = strImgDir + vImgNames[0] + base_name;
        std::string strRightImgName = strImgDir + vImgNames[1] + base_name;
        
        cv::Mat img_left = cv::imread(strLeftImgName, CV_LOAD_IMAGE_GRAYSCALE);
        cv::Mat img_right = cv::imread(strRightImgName, CV_LOAD_IMAGE_GRAYSCALE);
        
        std::vector<cv::KeyPoint> keypoints_left, keypoints_right;
        cv::Mat descriptors_left, descriptors_right;
        
        pDynamicDetector->detect(img_left, keypoints_left, cv::Mat());
        pDescriptorExtractor->compute(img_left, keypoints_left, descriptors_left);
        
        pDynamicDetector->detect(img_right, keypoints_right, cv::Mat());
        pDescriptorExtractor->compute(img_right, keypoints_right, descriptors_right);
        
        std::vector< cv::DMatch > goodMatch;
        vo.matchFeatures(descriptors_left, descriptors_right, goodMatch);
        
        std::vector<cv::Point2f> matched_kp_left, matched_kp_right;
        cv::Mat mateched_desc_left, matched_desc_right;
        
        //std::cout << "image 2d points" << std::endl;
        //std::cout << "Left image       Right image" << std::endl;
        for(std::vector< cv::DMatch >::iterator it = goodMatch.begin(); it != goodMatch.end(); ++it)
        {
            matched_kp_left.push_back(keypoints_left[it->queryIdx].pt);
            if(mateched_desc_left.empty())
                descriptors_left.row(it->queryIdx).copyTo(mateched_desc_left);
            else
                mateched_desc_left.push_back(descriptors_left.row(it->queryIdx));
                
            matched_kp_right.push_back(keypoints_right[it->trainIdx].pt);
            if(matched_desc_right.empty())
                descriptors_right.row(it->trainIdx).copyTo(matched_desc_right);
            else
                matched_desc_right.push_back(descriptors_right.row(it->trainIdx));
                
            //std::cout << "(" << keypoints_left[it->queryIdx].pt.x << ", " << keypoints_left[it->queryIdx].pt.y <<
                    //")  (" << keypoints_right[it->trainIdx].pt.x << ", " <<  keypoints_right[it->trainIdx].pt.y << ")" << std::endl;
        }
        
        //std::cout << std::endl;
        
        //cv::Mat img_matches;
        //cv::drawMatches(img_left, keypoints_left, img_right, keypoints_right,
                   //goodMatch, img_matches, cv::Scalar::all(-1), cv::Scalar::all(-1),
                   //std::vector<char>(), cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );
                   
         ////-- Show detected matches
        //cv::imshow( "Good Matches & Object detection", img_matches );
    
        //cv::waitKey(0);
        if (nImg  == 0)
        {
            vo.setCurrentKeypoints(matched_kp_left, mateched_desc_left, matched_kp_right, matched_desc_right);
        }

        vo.replaceKeypoints();
        vo.setCurrentKeypoints(matched_kp_left, mateched_desc_left, matched_kp_right, matched_desc_right);

        std::cout << "----- Image " << nImg << " -----" << std::endl;
        
       // clock_t tStart = clock();
        int nClock = clock_gettime(CLOCK_MONOTONIC, &tIni);
        vo.process();
        nClock = clock_gettime(CLOCK_MONOTONIC, &tEnd);
        
        //clock_t tElapsed = clock() - tStart;
        
       // double tComputation = (double)tElapsed / (CLOCKS_PER_SEC/1000.0);
        
        //aveTime += tComputation;
        //if(tComputation > maxTime)
            //maxTime = tComputation;
        double  tSecond = (tEnd.tv_sec - tIni.tv_sec) * 1000.0;
        double  tNano = (tEnd.tv_nsec -  tIni.tv_nsec) * 1.0 / 1e6; 
        std::cout << "Compuation time: " << tSecond + tNano << " ms" << std::endl;
        
        TooN::SE3<> se3Tr = vo.getMotion();
        se3Pose = se3Pose * se3Tr.inverse();
        
        std::cout << "Pose: " << se3Pose << std::endl;
    }
    
    //std::cout << "Average computation time: " << aveTime/nTotalImg << std::endl;
    //std::cout << "Maximum computation time: " << maxTime << std::endl;

    if(!pFeatureDector)
        delete pFeatureDector;
        
    if(!pDynamicDetector)
        delete pDynamicDetector;
        
    //if(!pDescriptorExtractor)
        //delete pDescriptorExtractor;
        
	return 0;
}

