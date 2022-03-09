#ifndef MULTI_CAMERA_SUBSCRIBER
#define MULTI_CAMERA_SUBSCRIBER

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/nonfree/features2d.hpp>
#include <TooN/TooN.h>
#include <TooN/se3.h>
#include <toposlam/Types.h>

//typedef std::map<std::string, cv_bridge::CvImageConstPtr> ImagePtrMap;
//typedef std::map<std::string, sensor_msgs::CameraInfo> InfoMap;
//typedef std::map<std::string, geometry_msgs::Pose> PoseMap;
//typedef std::map<std::string, cv::Mat> ImageBWMap;
//typedef std::map<std::string, cv::Size > ImageSizeMap;
//typedef std::map<std::string, TooN::SE3<> > SE3Map;
//typedef std::map<std::string, TooN::Vector<9> > ParamMap;

class MultiCameraSubscriber
{
public:
    MultiCameraSubscriber(std::vector<std::string> vCameraNames, std::string strImgTopic = "image_raw", std::string strInfoTopic = "camera_info", std::string strCamPrefix = "");
    ~MultiCameraSubscriber();
    
    ImagePtrMap GetNewImage();
    bool GetAndFillFrameBW(ImageBWMap &imBW);
    
	/** @brief Get the internal calibration from all the cameras
	* 
	*  This function returns a saved CameraInfo message it got during initialization, so if the camera starts sending 
	*  a different calibration after the constructor has been called, it won't be reflected in the output of this function.
	*  @return A map of camera names => CameraInfo, from which camera parameters can be extracted.  */
	InfoMap GetInfo(){ return mmSavedInfos; }
	
	// Info getter functions
	/** @brief Get the total size of all camera images, tiled side by side
	* 
	*  Makes the assumption that the tiling should be 2 columns. Could be changed if necessary.
	*  @param bFullSize If true, uses the full scale size of the camera images, otherwise uses the current size
	*  @return The width and height of the area taken up by the tiled images */
	//CVD::ImageRef GetTotalSize(bool bFullSize);
	
	/** @brief Get the individual sizes of the current camera images
	*  @return A map of camera names => image sizes */
	ImageSizeMap GetSizes(){ return mmSizes; }
	
	/** @brief Get the individual sizes of the current camera images times any binning being applied
	*  @return A map of camera names => image sizes */
	//ImageRefMap GetFullScaleSizes(){ return mmFullScaleSizes; }
	
	/** @brief Get the individual sizes of the camera images that were used to calibrate the cameras
	*  @return A map of camera names => image sizes */
	//ImageRefMap GetCalibSizes(){ return mmCalibSizes; }
	
	/** @brief Get the coordinates of the top left of each image necessary to recreate the tiling layout used in GetTotalSize
	*  @return A map of camera names => coordinates */
	//ImageRefMap GetDrawOffsets(){ return mmDrawOffsets; }
	
	/** @brief Get the calibrated parameters of each camera
	*  @return A map of camera names => parameter vectors */
	ParamMap GetParams(){ return mmParams; }
	
	/** @brief Get the calibrated pose of each camera
	*  @return A map of camera names => poses */
	SE3Map GetPoses(){ return mmPoses; }
	
	/** @brief Call the service to set the camera poses in the respective camera_infos 
	*  @param mPoses A map of the extrinsic poses for the cameras
	*  @return Successfully saved all poses */
	//bool SavePoses(const SE3Map& mPoses);
	
	geometry_msgs::Pose GetReferencePose() {return mReferencePose; }
	
	unsigned GetCameraNumber() const {return mNumCams;}
	
	void SetCameraTopics(std::string strImgTopic, std::string strInfoTopic, std::string strCamPrefix);
   
protected:
    void ImageCallback(const sensor_msgs::ImageConstPtr& msg1, const sensor_msgs::ImageConstPtr& msg2, const sensor_msgs::ImageConstPtr& msg3, const sensor_msgs::ImageConstPtr& msg4,
                    const sensor_msgs::ImageConstPtr& msg5, const sensor_msgs::ImageConstPtr& msg6, const sensor_msgs::ImageConstPtr& msg7, const sensor_msgs::ImageConstPtr& msg8);
                    
	/** @brief This will be called by the ROS subscriber whenever a CameraInfo message arrives from any of the cameras
	* 
	*  Saves the message into the savedInfo map.
	*  @param infoMsg The actual CameraInfo message
	*  @param cameraName The camera that this message came from */
	void InfoCallback(const sensor_msgs::CameraInfo::ConstPtr& infoMsg, std::string cameraName);    
	
	// /** @brief This will be called by the ROS subscriber whenever a Pose message arrives from any of the cameras
	//* 
	//*  Saves the message into the savedPose map.
	//*  @param poseMsg The actual Pose message
	//*  @param cameraName The camera that this message came from */
	
	//void PoseCallback(const geometry_msgs::Pose::ConstPtr& poseMsg, std::string cameraName);	            
    
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::Image,
                                                          sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::Image> ApproxTimePolicy;  
                                                          
    /** @brief Extract data from the CameraInfo message and fill out member variables
	*  @param infoMsg The calibration message from the camera
	*  @param cameraName The name of the camera that sent the message */
	void RecordInfo(sensor_msgs::CameraInfo infoMsg, std::string cameraName);
	
	void ReferencePoseCallback(const geometry_msgs::PoseWithCovarianceStamped& poseMsg);	                                       

    //bool NewAcquistion(ros::WallDuration timeout, std::string camName = "")
    
    ros::NodeHandle mNodeHandle;  ///< ROS node handle with global namespace
    ros::NodeHandle mNodeHandlePriv;  ///< ROS node handle with private namespace
    image_transport::ImageTransport* mpImageTransport;      ///< The image transport, used to enable image compression
    
    unsigned mNumCams;                       ///< The number of cameras in this group
    std::vector<std::string> mvCameraNames;   ///< Vector of camera names in this group
    std::string sImageTopic;         ///< The image topic for each camera
	std::string sInfoTopic;          ///< The info topic for each camera
	std::string sCameraPrefix;       ///< Prefix added on to each camera name
  
    message_filters::Synchronizer<ApproxTimePolicy>* mpSync;   ///< The message synchronizer, using the ApproxTimePolicy we defined 
    std::vector< image_transport::SubscriberFilter* > mvpImageSubs;   ///< Vector of image subscribers (one per camera) as SubscriberFilter pointers so they can be chained with the synchronizer
	std::vector< ros::Subscriber > mvInfoSubs;    ///< Vector of regular subscribers (one per camera) for getting camera info, these don't need to be synchronized
	ros::Subscriber mReferencePoseSub; /// subscribe to the Reference  pose (position & orientation), which is needed to tag each image
	ros::CallbackQueue mCallbackQueue;         ///< Custom callback queue so we can spin just for our own callbacks instead of a node-wide spin
	  
    ImagePtrMap mmLastImages;    ///< A map of camera name => CvImageConstPtr from the last acquisition  
    //bool mbWaitActive;          ///< Setting to false will terminate acquisition threads
    
    InfoMap mmSavedInfos;        ///< A map of camera name => CameraInfo that we saved from the initialization
	//PoseMap mmSavedPoses;        ///< A map of camera name => Pose that we saved from the initialization
	
	ImageSizeMap mmSizes;
	SE3Map mmPoses;
	ParamMap mmParams;
	
	geometry_msgs::Pose mReferencePose;
};


#endif
