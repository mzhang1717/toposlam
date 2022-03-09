#ifndef TYPES_
#define TYPES_

#include <TooN/TooN.h>
#include <TooN/se3.h>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <geometry_msgs/Pose.h>
#include <cv_bridge/cv_bridge.h>
#include <toposlam/CameraModel.h>

#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>
#include <boost/graph/adj_list_serialize.hpp>
#include <boost/serialization/vector.hpp>
#include <boost/serialization/map.hpp>
#include <boost/archive/archive_exception.hpp>
#include <boost/foreach.hpp>
#include <boost/serialization/split_free.hpp>
#include <boost/serialization/vector.hpp>

//typedef std::map<std::string, CameraModel> TaylorCameraMap;
typedef std::map<std::string, TooN::Matrix<4, 3> > Corner3DMap;
typedef std::map<std::string, TooN::Matrix<4, 2> > Corner2DMap;
typedef std::map<std::string, TooN::Matrix<3, 3> > CameraMatrixMap;
typedef std::map<std::string, cv::Mat> ProjectMap;

typedef std::map<std::string, std::vector<cv::KeyPoint> > KeyPointMap;
typedef std::map<std::string, cv::Rect> ROIMap;

typedef std::map<std::string, cv_bridge::CvImageConstPtr> ImagePtrMap;
typedef std::map<std::string, sensor_msgs::CameraInfo> InfoMap;
typedef std::map<std::string, geometry_msgs::Pose> PoseMap;
typedef std::map<std::string, cv::Mat> ImageBWMap;
typedef std::map<unsigned int, cv::Mat> MatIndexMap;
typedef std::map<std::string, cv::Size > ImageSizeMap;
typedef std::map<std::string, TooN::SE3<> > SE3Map;
typedef std::map<std::string, TooN::Vector<9> > ParamMap;


 
BOOST_SERIALIZATION_SPLIT_FREE(::cv::Mat)
namespace boost {
  namespace serialization {
 
    /** Serialization support for cv::Mat */
    template<class Archive>
    void save(Archive & ar, const ::cv::Mat& m, const unsigned int version)
    {
      size_t elem_size = m.elemSize();
      size_t elem_type = m.type();
 
      ar & m.cols;
      ar & m.rows;
      ar & elem_size;
      ar & elem_type;
 
      const size_t data_size = m.cols * m.rows * elem_size;
      ar & boost::serialization::make_array(m.ptr(), data_size);
    }
 
    /** Serialization support for cv::Mat */
    template<class Archive>
    void load(Archive & ar, ::cv::Mat& m, const unsigned int version)
    {
      int cols, rows;
      size_t elem_size, elem_type;
 
      ar & cols;
      ar & rows;
      ar & elem_size;
      ar & elem_type;
 
      m.create(rows, cols, elem_type);
 
      size_t data_size = m.cols * m.rows * elem_size;
      ar & boost::serialization::make_array(m.ptr(), data_size);
    }
 
  }
};

//namespace boost {
//namespace serialization {

//template<class Archive>
//void serialize(Archive & ar, MatIndexMap & mIndex, const unsigned int version)
//{
	//typedef std::pair<const unsigned int, cv::Mat>& pr;
	//BOOST_FOREACH(pr p, mIndex)
	//{
		//ar & p.first & p.second;
	//}

//}

//} // namespace serialization
//}; // namespace boost

#endif 
