#include <toposlam/System.h>
#include <toposlam/TrackerInfo.h>


System::System() : m_eMode(MODE_DISPLAY), m_eModeLast(MODE_NONE), m_NodeHandlePriv("~")
{
	// default names of cameras
    const char *sCamNames[] = {"camera1", "camera2", "camera3", "camera4"};
	std::vector<std::string> vNameDefault(sCamNames, sCamNames+4);
	
	std::vector<std::string> vCameraNames;	
	
	// if cameras's names are not provided, use the default names
	if(!m_NodeHandlePriv.getParam("camera_name_list", vCameraNames))
	{
		vCameraNames = vNameDefault;
	}
	
	// load names of stereos pair
	std::vector<std::string> vStereoNames;
	m_NodeHandlePriv.getParam("stereo_name_list", vStereoNames);
	
	// load strings related to the name of image and camera info topics
	std::string strImgTopic, strInfoTopic, strCamPrefix;
	m_NodeHandlePriv.param<std::string>("image_topic", strImgTopic, "image_raw");
	m_NodeHandlePriv.param<std::string>("info_topic", strInfoTopic, "camera_info");
	m_NodeHandlePriv.param<std::string>("camera_prefix", strCamPrefix, "");
	
	bool bToRectify;
	m_NodeHandlePriv.param<bool>("image_to_rectify", bToRectify, true);

	// load parameters of the vocabulary tree
	int nVocTreeLevel, nVocTreeBranch, nVocTreeLeafSize;    
	m_NodeHandlePriv.param<int>("voctree_level", nVocTreeLevel, 6);
	m_NodeHandlePriv.param<int>("voctree_branch", nVocTreeBranch, 10);
    m_NodeHandlePriv.param<int>("voctree_leafsize", nVocTreeLeafSize, 200);
    
    // load settings for feature detection
    std::string strFeatureDetector, strFeatureDescriptor;
    int nFeatureNumber, nThreshold;
    m_NodeHandlePriv.param<std::string>("feature_detector", strFeatureDetector, "CENSURE");
    m_NodeHandlePriv.param<std::string>("feature_descriptor", strFeatureDescriptor, "SIFT");
    m_NodeHandlePriv.param<int>("feature_number", nFeatureNumber, 200);
    m_NodeHandlePriv.param<int>("feature_threshold", nThreshold, 70);
    
    // subscribe to images of multiple cameras
    m_pCamSub = new MultiCameraSubscriber(vCameraNames, strImgTopic, strInfoTopic, strCamPrefix);	
    
    // Get the pose of each camera w.r.t. to the first camera
	//m_mPoses.clear();
	//m_mPoses = m_pCamSub->GetPoses();
	
    //InfoMap camInfo = m_pCamSub->GetInfo();
	
    // Create camera models here
    ImageSizeMap mImageSizes = m_pCamSub->GetSizes();
	ParamMap mParams = m_pCamSub->GetParams();   	 
	TaylorCameraMap mCameraModels; // list of camerea models
	cv::Size irImageSize;
	
	for(ImageSizeMap::iterator it = mImageSizes.begin(); it != mImageSizes.end(); ++it)
	{
		std::string camName = it->first;
			
		irImageSize = mImageSizes[camName];
		TooN::Vector<2> v2ImgSize = TooN::makeVector(irImageSize.width, irImageSize.height);
		TooN::Vector<9>& v9Params = mParams[camName];
		//std::cout << camName << ": size: " << v2ImgSize << std::endl << "Parameters: " << v9Params << std::endl;
		
		mCameraModels.insert(std::pair<std::string, CameraModel>(camName, CameraModel(v9Params, v2ImgSize)));
	}	
	
	m_pImgProc =  new ImageProcess(mCameraModels, strFeatureDetector, strFeatureDescriptor, nFeatureNumber, nThreshold);
	
	m_pWindow = new ImageWindow();
	
	m_pVocTree = new VocabTree(nVocTreeLevel, nVocTreeBranch, nVocTreeLeafSize); // 6 levels x 10 branches, 10e6 leaf nodes 
	m_pGraph =  new TopoGraph<TV, TE, TI>();
	
	m_pMapMaker = new Mapping(m_pCamSub, m_pWindow, m_pImgProc, m_pVocTree, m_pGraph, bToRectify);
	m_pTracker = new Tracking(m_pCamSub, m_pWindow, m_pImgProc, m_pVocTree, m_pGraph, bToRectify); 
	m_pMapMaker->SetStereoNames(vStereoNames);
	m_pTracker->SetStereoNames(vStereoNames);
	
	// load image masks for each camera
	ImageBWMap maskMaps = LoadMasks();
	m_pMapMaker->SetMasks(maskMaps);  
	m_pTracker->SetMasks(maskMaps);  
	
	
	mTrackerInfoPub = m_NodeHandlePriv.advertise<toposlam::TrackerInfo>("t_pose", 1);
}

System::~System()
{
	if (m_pCamSub != NULL) { delete m_pCamSub; }
	
	if (m_pWindow != NULL) { delete m_pWindow; }
	
	if (m_pImgProc != NULL) {delete m_pImgProc; }
	
	if (m_pMapMaker != NULL) {delete m_pMapMaker; }
		
	if (m_pTracker != NULL) {delete m_pTracker; }
	
	if (m_pVocTree != NULL) {delete m_pVocTree; }	
	
	if (m_pGraph != NULL) {delete m_pGraph; }	
}

void System::Init()
{

}

void System::Run()
{
	while (ros::ok())
	{
		//if(m_eMode != m_eModeLast) // if there is a mode change, may need to reset the size of the display window
			//m_pWindow->RequestResize(true);
			
	    //std::cout << "Keyboard command = " << m_Window.GetKeyCommand() << std::endl;
	    
	    switch((char)m_pWindow->GetKeyCommand())
	    {
			case 'd':
				m_eMode = MODE_DISPLAY;
				m_pWindow->RequestResize(true);
				break;
			case 'm': 
				m_eMode = MODE_MAPPING;
				m_pWindow->RequestResize(true);
				break;
			case 'c':
				m_eMode = MODE_CLUSTERING;
				m_pWindow->RequestResize(true);
				break;
			case 't':
				m_eMode = MODE_TRACKING;
				m_pWindow->RequestResize(true);
				break;	
			case 'v':
				m_eMode = MODE_TRACKWITHMAP;
				m_pWindow->RequestResize(true);
				break;
			case 'q':
				m_eMode = MODE_QUIT;
			default:
				break;		
		}				
					
	    if (m_eMode == MODE_DISPLAY)
	    {
			ImageBWMap imagesBW, imagesNew;
		    m_pCamSub->GetAndFillFrameBW(imagesBW);
		    
		    if (imagesBW.empty())
		    {
		        std::cout << "Mapping: No image is received ..." << std::endl;
		        return;
		    }	
			m_pWindow->DrawFrame(imagesBW);
		}
		else if (m_eMode == MODE_MAPPING)
		{
			m_pMapMaker->Run();   
		}
		else if (m_eMode == MODE_CLUSTERING)
		{
			m_pMapMaker->Clustering();
			m_eMode = MODE_DISPLAY;
		}
		else if (m_eMode == MODE_TRACKING)
		{
			static bool bMapAvailable = false;
			if(!bMapAvailable)
			{
				m_pMapMaker->Clustering();
				bMapAvailable = true;
			}			
			m_pTracker->Run();
			PublishPose();
		}
		else if (m_eMode == MODE_TRACKWITHMAP)
		{
			static bool bMapAvailable = false;
			if(!bMapAvailable)
			{
				m_pTracker->LoadDescriptors("/home/mingfeng/Descriptors.dat");
				bMapAvailable = true;
			}			
			m_pTracker->Run();	
			PublishPose();		
		}
		else if (m_eMode == MODE_QUIT)
		{
			m_pTracker->SaveScore("/home/mingfeng/Scores.dat");
			std::exit(0);
		}
		else
		{
			std::cout << "Undefined command! Exit mapping!!" << std::endl;
			return;
		}
	     
		m_eModeLast = m_eMode;
		
		ros::spinOnce();
	}
}

ImageBWMap System::LoadMasks()
{
	std::map<std::string, std::string> vMaskFiles;
    //if(!m_NodeHandlePriv.getParam("masks", vMaskFiles)) 
    if(!m_NodeHandlePriv.getParam("camera_mask_list", vMaskFiles)) 
	{
		ROS_ERROR_STREAM("System: could not load image masks ... ");	
	}
	
    std::string strMaskDir;	
	m_NodeHandlePriv.getParam("masks_dir", strMaskDir);
	
	ImageBWMap masks;
	for(std::map<std::string, std::string>::iterator it = vMaskFiles.begin(); it != vMaskFiles.end(); ++it)
	{
		std::string filename = strMaskDir + "/"+ it->second;
		std::cout << it->first << "'s mask: " << filename << std::endl;
	
		cv::Mat img = cv::imread(filename, CV_LOAD_IMAGE_GRAYSCALE);
		masks[it->first] = img;
	}
	
	return masks;
}

void System::PublishPose()
{
	toposlam::TrackerInfo tInfo;
	
	tInfo.trueNodeID = m_pTracker->m_nTrueFrameID;
	tInfo.matchedNodeID = m_pTracker->m_nMatchedFrameID;
	tInfo.matchedNodePose = m_pTracker->m_MatchedFramePose;
	tInfo.odometryPose =  m_pTracker->m_OdometryPose;
	tInfo.truePose = m_pTracker->m_ReferencePose;
	
	mTrackerInfoPub.publish(tInfo);
}
