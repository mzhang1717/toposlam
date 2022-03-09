#include <toposlam/MultiCameraSubscriber.h>

MultiCameraSubscriber::MultiCameraSubscriber(std::vector<std::string> vCameraNames, std::string strImgTopic, std::string strInfoTopic, std::string strCamPrefix)
: mNodeHandlePriv("~")
, mvCameraNames(vCameraNames) ///< camera names
, sImageTopic(strImgTopic)         ///< The image topic for each camera
, sInfoTopic(strInfoTopic)          ///< The info topic for each camera
, sCameraPrefix(strCamPrefix)       ///< Prefix added on to each camera name	
, mpSync(new message_filters::Synchronizer<ApproxTimePolicy>(ApproxTimePolicy(5)))  // queue size of 5
{
	mNodeHandle.setCallbackQueue(&mCallbackQueue);
    mpImageTransport = new image_transport::ImageTransport(mNodeHandle);
    
    //mvCameraNames = vCameraNames; // cameras' names
    mNumCams = mvCameraNames.size(); // number of cameras
    
    std::string strNamelist;
    for(size_t i = 0; i < mNumCams; i++)
    {
		strNamelist = strNamelist + mvCameraNames[i] + " ";
	}
    
    ROS_INFO_STREAM("MultiCameraSubscriber: Loading " << mNumCams << " cameras (" << strNamelist << ")");
    
  
    // Maximum of 8 cameras in a group. Unlikely to have more since we'll have
    // bandwidth issues with all those images coming in together. Also, need
    // to specifiy the synchronizer policy at compile time so we can't use
    // the runtime variable mNumCams
    
    //std::cout << "Number of cameras: " << mNumCams << std::endl;
    
    if(mNumCams < 1)
    {
        ROS_FATAL("MultiCameraSubscriber: No camera provided ...");
        ros::shutdown();
        return;
    }
    else if(mNumCams > 8)
    {
        ROS_FATAL_STREAM("MultiCameraSubscriber: Maximum 8 cameras, provided " << mNumCams);
        ros::shutdown();
        return;
    }
    
    // Create space for our image subscribers
    mvpImageSubs.resize(mNumCams);
    for(unsigned i=0; i < mNumCams; ++i)
    {
        //std::cout << mvCameraNames[i] << std::endl;
        // Subscribe to the appropriate image topic
        mvpImageSubs[i] = new image_transport::SubscriberFilter();
        
        std::string strCamName = sCameraPrefix + "/" + mvCameraNames[i] + "/" + sImageTopic;
        //std::cout << strCamName << std::endl;
        
        mvpImageSubs[i]->subscribe(*mpImageTransport, strCamName, 2); 
    }
    
    // Connect the synchronizer to the subscribers
    // If there are fewer than 8 cameras, the unfilled slots are connected
    // back to the first subscriber (no performance hit)
    switch(mNumCams)
    {
        case 1:
        {
            mpSync->connectInput(*mvpImageSubs[0],*mvpImageSubs[0],*mvpImageSubs[0],*mvpImageSubs[0],
            *mvpImageSubs[0],*mvpImageSubs[0],*mvpImageSubs[0],*mvpImageSubs[0]);
            break;
        }
        case 2:
        {
            mpSync->connectInput(*mvpImageSubs[0],*mvpImageSubs[1],*mvpImageSubs[0],*mvpImageSubs[0],
            *mvpImageSubs[0],*mvpImageSubs[0],*mvpImageSubs[0],*mvpImageSubs[0]);
            break;
        }
        case 3:
        {
            mpSync->connectInput(*mvpImageSubs[0],*mvpImageSubs[1],*mvpImageSubs[2],*mvpImageSubs[0],
            *mvpImageSubs[0],*mvpImageSubs[0],*mvpImageSubs[0],*mvpImageSubs[0]);
            break;
        }
        case 4:
        {
            mpSync->connectInput(*mvpImageSubs[0],*mvpImageSubs[1],*mvpImageSubs[2],*mvpImageSubs[3],
            *mvpImageSubs[0],*mvpImageSubs[0],*mvpImageSubs[0],*mvpImageSubs[0]);
            break;
        }
        case 5:
        {
            mpSync->connectInput(*mvpImageSubs[0],*mvpImageSubs[1],*mvpImageSubs[2],*mvpImageSubs[3],
            *mvpImageSubs[4],*mvpImageSubs[0],*mvpImageSubs[0],*mvpImageSubs[0]);
            break;
        }
        case 6:
        {
            mpSync->connectInput(*mvpImageSubs[0],*mvpImageSubs[1],*mvpImageSubs[2],*mvpImageSubs[3],
            *mvpImageSubs[4],*mvpImageSubs[5],*mvpImageSubs[0],*mvpImageSubs[0]);
            break;
        }
        case 7:
        {
            mpSync->connectInput(*mvpImageSubs[0],*mvpImageSubs[1],*mvpImageSubs[2],*mvpImageSubs[3],
            *mvpImageSubs[4],*mvpImageSubs[5],*mvpImageSubs[6],*mvpImageSubs[0]);
            break;
        }
        case 8:
        {
            mpSync->connectInput(*mvpImageSubs[0],*mvpImageSubs[1],*mvpImageSubs[2],*mvpImageSubs[3],
            *mvpImageSubs[4],*mvpImageSubs[5],*mvpImageSubs[6],*mvpImageSubs[7]);
            break;
        }
        default:
        {
            ROS_ERROR("MultiCameraSubscriber: Couldn't resolve mpSync connectInput switch block, should never happen");
            return;
        }
    }
    
    // Connect the callback to the synchronizer
    mpSync->registerCallback(&MultiCameraSubscriber::ImageCallback, this);
    
	// Subscribe to the camera info and pose topics
	mvInfoSubs.resize(mNumCams);
	for(unsigned i=0; i < mNumCams; ++i)
	{
		std::string strTopicName = sCameraPrefix + "/" + mvCameraNames[i] + "/" + sInfoTopic;
		mvInfoSubs[i] = mNodeHandle.subscribe<sensor_msgs::CameraInfo>(strTopicName, 1, boost::bind(&MultiCameraSubscriber::InfoCallback, this, _1, mvCameraNames[i])); 
	}
	
	ROS_INFO_STREAM("MultiCameraSubscriber: Waiting for camera info (including pose) from all cameras");
	
	while(mmSavedInfos.size() < mNumCams && ros::ok())  // loop while we haven't collected all the info yet
	{
		mCallbackQueue.callAvailable();
		ros::WallDuration(0.2).sleep();
	}
	
	for(unsigned i=0; i < mNumCams; ++i)
		mvInfoSubs[i].shutdown();  // stop subscribers to CameraInfo once recieved one set of CameraInfo
	
	//ROS_INFO("MultiCameraSubscriber: Got info of all cameras");
	
	// Extract camera info from the first set of images
	InfoMap mInfo = GetInfo();
	for(InfoMap::iterator it = mInfo.begin(); it != mInfo.end(); ++it)
	{
		RecordInfo(it->second, it->first);
	}

	ROS_INFO_STREAM("MultiCameraSubscriber: Got ino of all cameras");

	ImagePtrMap mpImages = GetNewImage(); // get a set of images so we can extract image size info
	while(mpImages.size() < mNumCams && ros::ok())  // loop while we haven't collected all the info yet
	{
		mpImages = GetNewImage();
	}

	for(ImagePtrMap::iterator it = mpImages.begin(); it != mpImages.end(); ++it)
	{
		std::string camName = it->first;
		mmSizes[camName] =  it->second->image.size();
		
		//CVD::ImageRef irBinning = mmBinnings[camName];
		//mmFullScaleSizes[camName] = CVD::ImageRef(it->second->image.cols * irBinning.x,it->second->image.rows * irBinning.y);
		
		// Prepare workspace variables
		//mmWorkspaceBW[camName] = CVD::Image<CVD::byte>();
		//mmWorkspaceRGB[camName] = CVD::Image<CVD::Rgb<CVD::byte> >();
	}
	
	ROS_INFO("MultiCameraSubscriber: Got first set of images");	
	
	mReferencePoseSub = mNodeHandle.subscribe("/RefPose", 1,  &MultiCameraSubscriber::ReferencePoseCallback, this); 
	
	mReferencePose.position.z = -999999.0;
}

MultiCameraSubscriber::~MultiCameraSubscriber()
{
    // Need to delete this first because it will try to disconnect from the
    // mvpImageSubs in the destructor
    delete mpSync;
    
    for(unsigned i=0; i < mNumCams; ++i)
    {
        mvpImageSubs[i]->unsubscribe();
        delete mvpImageSubs[i];
    }
    
    mvpImageSubs.clear();
    //mvInfoSubs.clear();
    
    delete mpImageTransport;
}

void MultiCameraSubscriber::ImageCallback(const sensor_msgs::ImageConstPtr& msg1, const sensor_msgs::ImageConstPtr& msg2, const sensor_msgs::ImageConstPtr& msg3, const sensor_msgs::ImageConstPtr& msg4,
                                          const sensor_msgs::ImageConstPtr& msg5, const sensor_msgs::ImageConstPtr& msg6, const sensor_msgs::ImageConstPtr& msg7, const sensor_msgs::ImageConstPtr& msg8)
{
    ROS_INFO("MultiCameraSubscriber: ImageCallback is called!");
    
  // Remeber that if the number of cameras is less than 8, the messages in the unused slots will point back to the first image message
  try
  {
    if(mNumCams > 0)
      mmLastImages[mvCameraNames[0]] = cv_bridge::toCvCopy(msg1);
      
    if(mNumCams > 1)
      mmLastImages[mvCameraNames[1]] = cv_bridge::toCvCopy(msg2);
      
    if(mNumCams > 2)
      mmLastImages[mvCameraNames[2]] = cv_bridge::toCvCopy(msg3);
      
    if(mNumCams > 3)
      mmLastImages[mvCameraNames[3]] = cv_bridge::toCvCopy(msg4);
      
    if(mNumCams > 4)
      mmLastImages[mvCameraNames[4]] = cv_bridge::toCvCopy(msg5);
      
    if(mNumCams > 5)
      mmLastImages[mvCameraNames[5]] = cv_bridge::toCvCopy(msg6);
      
    if(mNumCams > 6)
      mmLastImages[mvCameraNames[6]] = cv_bridge::toCvCopy(msg7);
      
    if(mNumCams > 7)
      mmLastImages[mvCameraNames[7]] = cv_bridge::toCvCopy(msg8);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("MultiCameraSubscriber: cv_bridge exception: %s", e.what());
    return;
  }
}

// This will be called by the ROS subscriber whenever a CameraInfo message arrives from any of the cameras
void MultiCameraSubscriber::InfoCallback(const sensor_msgs::CameraInfo::ConstPtr& infoMsg, std::string cameraName)
{
	ROS_INFO("MultiCameraSubscriber: InfoCallback is called!");
	
	// Save to the appropriate slot
	mmSavedInfos[cameraName] = *infoMsg;
}

void MultiCameraSubscriber::ReferencePoseCallback(const geometry_msgs::PoseWithCovarianceStamped& poseMsg)	
{
	ROS_DEBUG("MultiCameraSubscriber: in ReferencePoseCallback");
	mReferencePose = poseMsg.pose.pose;
	//std::cout << "Reference position: (" <<  mReferencePose.position.x << ", "  << mReferencePose.position.y << ", " << mReferencePose.position.z << ")" << std::endl;
	//std::cout << "Reference orientation: (" <<  mReferencePose.orientation.x << ", " << mReferencePose.orientation.y << ", " << mReferencePose.orientation.z << ", " << mReferencePose.orientation.w << ")" << std::endl;
}

// Grab a new set of images from all the cameras in the group
ImagePtrMap MultiCameraSubscriber::GetNewImage( )
{
  mmLastImages.clear();
  
  ros::WallRate rate(200);
  
  // Wait until mmLastImages has been filled and we don't want to bail
  while(mmLastImages.size() < mNumCams && ros::ok() )
  {
    mCallbackQueue.callAvailable();   // This will trigger any waiting callbacks
    rate.sleep();
  }
  
  if(mmLastImages.size() < mNumCams)
    return ImagePtrMap(); // aborted early, so return empty map
  else // otherwise return good set of images
    return mmLastImages;  
}

// Acquires a new set of images from the camera group whose data arrives first, outputs it in greyscale format
bool MultiCameraSubscriber::GetAndFillFrameBW(ImageBWMap &imBW)
{
	ImagePtrMap mpTempImages = GetNewImage();
	
	if (mpTempImages.empty())
		return false;
		
    imBW.clear();
    
    for(ImagePtrMap::iterator it = mmLastImages.begin(); it != mmLastImages.end(); it++)
    {
        if (it->second->image.channels() == 3)
            cv::cvtColor(it->second->image, imBW[it->first], cv::COLOR_BGR2GRAY);
        else
            imBW[it->first] = it->second->image;
    }
    
    return true;
}



// Extract data from the CameraInfo message and fill out member variables
void MultiCameraSubscriber::RecordInfo(sensor_msgs::CameraInfo infoMsg, std::string cameraName)
{
	if(infoMsg.distortion_model != "taylor")
	{
		ROS_ERROR_STREAM(cameraName<<" not calibrated with Taylor distortion model!");
	//ros::shutdown();
	}
	
	if(infoMsg.binning_x == 0)
		infoMsg.binning_x = 1;
	
	if(infoMsg.binning_y == 0)
		infoMsg.binning_y = 1;
	
	// The parameters are:
	// 0 - a0 coefficient
	// 1 - a2 coefficient
	// 2 - a3 coefficient
	// 3 - a4 coefficient
	// 4 - image center xc
	// 5 - image center yc
	// 6 - affine transform param c
	// 7 - affine transform param d
	// 8 - affine transform param e
	
	double c = infoMsg.K[0];
	double d = infoMsg.K[1];
	double e = infoMsg.K[3];
	
	double xc = infoMsg.K[2];
	double yc = infoMsg.K[5];
	
	double a0 = infoMsg.D[0];
	double a2 = infoMsg.D[1];
	double a3 = infoMsg.D[2];
	double a4 = infoMsg.D[3];
	
	mmParams[cameraName] = TooN::makeVector(a0,a2,a3,a4,xc,yc,c,d,e);
	//mmBinnings[cameraName] = CVD::ImageRef(infoMsg.binning_x, infoMsg.binning_y);  
	//mmCalibSizes[cameraName] = CVD::ImageRef(infoMsg.width, infoMsg.height);
	
	// pose is located in the info message, so extract it
	TooN::Matrix<3> m3Rot;
	for(int j=0; j < 3; ++j)
	{
		for(int i=0; i < 3; ++i)
		{
			m3Rot(j,i) = infoMsg.R[j*3 + i];
		}
	} 
	TooN::Vector<3> v3Trans = TooN::makeVector(infoMsg.P[3], infoMsg.P[7], infoMsg.P[11]);
	
	// Check the rotation matrix to see if it is a proper rotation
	TooN::Matrix<3> m3Result = m3Rot * m3Rot.T();  // this should be close to identity
	TooN::Vector<3> v3Ones = TooN::makeVector(1,1,1);
	TooN::Vector<3> v3Diff = v3Ones - m3Result*v3Ones;  // should be close to zero
	
	if(v3Diff * v3Diff > 1e-4)
	{
		m3Rot = TooN::Identity;
		v3Trans = TooN::Zeros;
		ROS_WARN_STREAM("MultiCameraSubscriber: The rotation matrix inside the CameraInfo message of " << cameraName << " is invalid!");
		//ROS_WARN_STREAM("VideoSourceMulti: Perhaps you meant to get the camera pose separately but forgot to set the get_pose_separately parameter to true?");
		ROS_WARN_STREAM("MultiCameraSubscriber: Defaulting to IDENTITY transformation.");
		//ROS_WARN_STREAM("VideoSourceMulti: If you are calibrating the camera, continue without worry. If you are not, things will be messed up.");
	}
	
	TooN::SE3<> se3Pose(TooN::SO3<>(m3Rot), v3Trans);
	
	// Need to invert the pose before storing since it is stored in the "usual" format in the
	// info message (ie pose of the camera IN another reference frame, such that x_r = P_c * x_c
	// where x_r is point in reference frame, P_c is camera's pose, x_c is point in camera frame)
	// whereas the KeyFrames where these poses end up store the poses in the inverse manner
	// (ie x_c = P_c * x_r)
	mmPoses[cameraName] = se3Pose.inverse();
}

void MultiCameraSubscriber::SetCameraTopics(std::string strImgTopic, std::string strInfoTopic, std::string strCamPrefix)
{
	sImageTopic = strImgTopic;         ///< The image topic for each camera
	sInfoTopic = strInfoTopic;          ///< The info topic for each camera
	sCameraPrefix = strCamPrefix;       ///< Prefix added on to each camera name	
}
