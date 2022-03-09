#ifndef VIDEO_SOURCE_MULTI
#define VIDEO_SOURCE_MULTI

#include <ros/ros.h>
#include <opencv2/core/core.hpp>
#include <MultiCameraSubscriber.h>

typedef std::map<std::string, cv::Mat> ImageBWMap;
typedef std::map<std::string, cv_bridge::CvImageConstPtr> ImagePtrMap;

class VideoSourceMulti
{
public:
    VideoSourceMulti(bool bGetPoseSeparately = false);
    ~VideoSourceMulti();
  
    bool GetAndFillFrameBW(ros::WallDuration timeout, ImageBWMap &mImBW, ros::Time& timestamp);

protected:
    void WaitForGroup(int nGroupIdx, bool* pThreadActive);
    bool NewAcquistion(ros::WallDuration timeout, std::string camName = "");    
    
    std::vector<std::string> mvCamNames;                ///< The names of all the cameras we're getting images from
    MultiCameraSubscriber* mvpCamGroups;   ///< Vector of camera groups
    
    ImagePtrMap mmpLastImages;         ///< %Map of images most recently received
    bool mbWaitActive;          ///< Setting to false will terminate acquisition threads
    
};

#endif
