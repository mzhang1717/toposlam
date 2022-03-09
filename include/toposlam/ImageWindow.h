#ifndef IMAGEWINDOW_
#define IMAGEWINDOW_

#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include <toposlam/Types.h>

//typedef std::map<std::string, cv::Mat> ImageBWMap;
//typedef std::map<std::string, std::vector<cv::KeyPoint> > KeyPointMap;
//typedef std::map<std::string, cv::Size> ImageSizeMap;
//typedef std::map<std::string, cv::Rect> ROIMap;

class ImageWindow
{
public:
    ImageWindow(std::string strWindowName = "TOPO-SLAM", int nKey = -1);
    void CreateWindow();
    ~ImageWindow();
        
    void DrawFrame(ImageBWMap imagesBW);
    void DrawFrame(ImageBWMap imagesBW, KeyPointMap keypoint);
    void GetImageSizes(ImageBWMap imagesBW);
    int GetKeyCommand();
    void Init();
    void RequestResize(bool bResize);
    
private:
    std::string m_strWindowName;
    cv::Size m_TotalSize;
    cv::Mat m_CurrentFrame;
    std::vector<cv::Mat> m_roiImage;
    ROIMap m_roiMap;
    bool m_bToResize;
    int m_keyCommand;
};

#endif
