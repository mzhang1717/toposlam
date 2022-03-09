#ifndef TOPO_SYSTEM_H_
#define TOPO_SYSTEM_H_

#include <ros/ros.h>
#include <TooN/TooN.h>
#include <TooN/se3.h>
#include <iostream>
#include <iterator>     // std::distance
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/nonfree/features2d.hpp>
#include <toposlam/MultiCameraSubscriber.h>
#include <toposlam/VocabTree.h>
#include <toposlam/ImageWindow.h>
#include <toposlam/ImageProcess.h>
#include <toposlam/Mapping.h>
#include <toposlam/Tracking.h>
#include <toposlam/VocabTree.h>
#include <toposlam/TopoGraph.h>
#include <toposlam/Types.h>

typedef unsigned int TI;
typedef VertexProperties<TI> TV;
typedef EdgeProperties TE;


enum SystemMode
{
	MODE_DISPLAY,
	MODE_MAPPING,
	MODE_CLUSTERING,
	MODE_TRACKING,
	MODE_TRACKWITHMAP,
	MODE_QUIT,
	MODE_NONE
};


class System
{
public:
	System();
	~System();
	void Init();
	void Run();
	ImageBWMap LoadMasks();
	void PublishPose();
private:
    MultiCameraSubscriber* m_pCamSub;
    ImageWindow* m_pWindow;
    ImageProcess* m_pImgProc;
	Mapping* m_pMapMaker;
	Tracking* m_pTracker;
	VocabTree*  m_pVocTree; 
	TopoGraph<TV, TE, TI>* m_pGraph;
	
	SystemMode m_eMode;
	SystemMode m_eModeLast;
	//bool m_bResetWindow;
	
	ros::NodeHandle m_NodeHandle;
	ros::NodeHandle m_NodeHandlePriv;
	//ros::Rate m_SysLoopRate;
	
	ros::Publisher mTrackerInfoPub;   ///< Publisher for tracker pose (array of cameras)
};

#endif
