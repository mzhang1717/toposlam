#include <VidoeSourceMulti.h>

VideoSourceMulti::VideoSourceMulti(bool bGetPoseSeparately)
{
    ROS_INFO("VideoSourceMulti: Initializing");
    const char *strCamName[] = {"camera1","camera2","camera3","camera4"};
    
    mvCamNames.assign(strCamName,std::end(strCamName));
}

VideoSourceMulti::~VideoSourceMulti()
{
    
}

// Acquires a new set of images from the camera group whose data arrives first, outputs it in greyscale format
bool VideoSourceMulti::GetAndFillFrameBW(ros::WallDuration timeout, ImageBWMap &imBW, ros::Time& timestamp)
{
    imBW.clear();
    
    bool success = NewAcquistion(timeout);
    
    if(!success)
        return false;
    
    timestamp = mLastTimestamp;
    
    for(ImagePtrMap::iterator it = mmpLastImages.begin(); it != mmpLastImages.end(); it++)
    {
        util::ConversionBW(it->second->image, mmWorkspaceBW[it->first]);
        imBW[it->first] = mmWorkspaceBW[it->first];
    }
    
    return true;
}

bool NewAcquistion(ros::WallDuration timeout, std::string camName = "")
{
    
    mbWaitActive = true;
    
    ImagePtrMap mpTempImages = mvpCamGroups->GetNewImage(&mbWaitActive);
    
    if(!mpTempImages.empty())
    {
        mmpLastImages.clear();
        boost::mutex::scoped_lock lock(mImageMutex);
        mmpLastImages = mpTempImages;
        //mLastTimestamp = mvpCamGroups->GetLastTimestamp();
    } 
    
    if(camName != "")
        return mmpLastImages.count(camName);
    else
        return !mmpLastImages.empty();    
}

void ConversionBW
