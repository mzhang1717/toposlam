#ifndef UTILITY_TOPO_
#define UTILITY_TOPO_


#include <TooN/TooN.h>
#include <TooN/se3.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Quaternion.h>
#include <tf/transform_datatypes.h>
#include <XmlRpcValue.h>

namespace UTIL
{
	template<typename A, typename B>
	std::pair<B,A> flip_pair(const std::pair<A,B> &p)
	{
	    return std::pair<B,A>(p.second, p.first);
	}
	
	template<typename A, typename B>
	std::map<B,A> flip_map(const std::map<A,B> &src)
	{
	    std::map<B,A> dst;
	    std::transform(src.begin(), src.end(), std::inserter(dst, dst.begin()), 
	                   flip_pair<A,B>);
	    return dst;
	}
	
	inline TooN::Vector<3> QuatToRPY(geometry_msgs::Quaternion quat)
	{
		tf::Quaternion q;
	    double roll, pitch, yaw;
	    tf::quaternionMsgToTF(quat, q);
	    tf::Matrix3x3(q).getRPY(roll, pitch, yaw);	
	    
	    return TooN::makeVector(roll, pitch, yaw);
	}
	
	inline TooN::Vector<3> MatToRPY(cv::Mat& rot)
	{
		CV_Assert(rot.depth() != sizeof(double));
		tf::Matrix3x3 m(rot.at<double>(0, 0), rot.at<double>(0, 1), rot.at<double>(0, 2),
						rot.at<double>(1, 0), rot.at<double>(1, 1), rot.at<double>(1, 2),
						rot.at<double>(2, 0), rot.at<double>(2, 1), rot.at<double>(2, 2));
						
		double roll, pitch, yaw;				
		m.getRPY(roll, pitch, yaw);	
	    
	    return TooN::makeVector(roll, pitch, yaw);
	}
	
	template <typename T>
	inline std::string ToString(T tX)
	{
	    std::ostringstream oStream;
	    oStream << tX;
	    return oStream.str();
	}

/** @brief Convert a geometry_msgs::Pose message into TooN::SE3<>
 *  @param pose The pose to convert
 *  @return The same pose in SE3 format */
inline TooN::SE3<> PoseMsgToSE3(geometry_msgs::Pose pose)
{
	TooN::SE3<> se3Transform;
	
	// Fill in translation directly
	se3Transform.get_translation() = TooN::makeVector(pose.position.x, pose.position.y, pose.position.z);
	
	// The rotation component of a Pose message is in a quaternion, so instead of messing around
	// with quaternion math myself I just use the Bullet classes included in ROS tf package
	
	// Convert orientation message to Bullet quaternion
	tf::Quaternion btQuat(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);
	
	// Convert to Bullet 3x3 matrix
	tf::Matrix3x3 btMat(btQuat);
	
	// Copy data into a TooN 3-Matrix
	TooN::Matrix<3> m3PoseRot;
	for(unsigned j=0; j < 3; j++)
	{
	for(unsigned i=0; i < 3; i++)
	{
	  m3PoseRot[i][j] = btMat[i][j];
	}
	}
	
	// Set the rotation and we're done
	se3Transform.get_rotation() = m3PoseRot;
	return se3Transform;
}

/** @brief Convert a TooN::SE3<> message into geometry_msgs::Pose
 *  @param se3Transform The SE3 pose to convert
 *  @return The same pose in geometry_msg format */
inline geometry_msgs::Pose SE3ToPoseMsg(TooN::SE3<> se3Transform)
{
	TooN::Vector<3> v3Pos = se3Transform.get_translation();
	TooN::Matrix<3> m3Rot = se3Transform.get_rotation().get_matrix();
	
	// Similar to PoseMsgToSE3(..), use Bullet 3x3 matrix and quaternion as intermediaries
	
	// Initialize 3x3 matrix directly from TooN 3-matrix
	tf::Matrix3x3 btMat(m3Rot(0,0),m3Rot(0,1),m3Rot(0,2),m3Rot(1,0),m3Rot(1,1),m3Rot(1,2),m3Rot(2,0),m3Rot(2,1),m3Rot(2,2));
	tf::Quaternion btQuat;
	btMat.getRotation(btQuat);
	
	geometry_msgs::Pose pose;
	
	// Fill in position directly
	pose.position.x = v3Pos[0];
	pose.position.y = v3Pos[1];
	pose.position.z = v3Pos[2];
	
	// Get quaternion values from Bullet quat
	pose.orientation.x = btQuat.x();
	pose.orientation.y = btQuat.y();
	pose.orientation.z = btQuat.z();
	pose.orientation.w = btQuat.w();
	
	return pose;
}


inline TooN::Vector<6> SE3Tov6Pose(TooN::SE3<> se3Transform)
{
	TooN::Vector<3> v3Pos = se3Transform.get_translation();
	TooN::Matrix<3> m3Rot = se3Transform.get_rotation().get_matrix();
	
	// Similar to PoseMsgToSE3(..), use Bullet 3x3 matrix and quaternion as intermediaries
	
	// Initialize 3x3 matrix directly from TooN 3-matrix
	tf::Matrix3x3 btMat(m3Rot(0,0),m3Rot(0,1),m3Rot(0,2),m3Rot(1,0),m3Rot(1,1),m3Rot(1,2),m3Rot(2,0),m3Rot(2,1),m3Rot(2,2));
	
	double roll, pitch, yaw;				
	btMat.getRPY(roll, pitch, yaw);	
	    
	return TooN::makeVector(v3Pos[0], v3Pos[1], v3Pos[2], roll, pitch, yaw);
}

/** @brief Parses a ROS param XmlRpcValue into a vector of string vectors
 * 
 *  This is called by code that needs to load a ROS param indicating camera grouping
 *  @param camNameList The XmlRpcValue containing camera names, grouped using parentheses. For example,  [[cam1, cam2],[cam3]] indicates
 *                      two camera groups, one with the cameras cam1 and cam2, and one with the camera cam3.
 *  @param [out] camNameStrings A vector of string vectors, the outer vector indicating grouping
 *  @return Did the parse succeed? */
inline bool Parse(XmlRpc::XmlRpcValue camNameList, std::vector<std::string> & camNameStrings)
{
	camNameStrings.clear();
	
	// input needs to be the right type
	if(camNameList.getType() != XmlRpc::XmlRpcValue::TypeArray)
	{
		ROS_ERROR("CameraNameParser: Camera parameters should be an array of camera names");
		return false;
	}

	for(size_t i=0; i < camNameList.size(); i++)
	{
		  ROS_INFO_STREAM( "CameraNameParser: Adding camera "<<std::string(camNameList[i]) );
		  camNameStrings.push_back(std::string(camNameList[i]));
	}
	
	return true;
}


inline TooN::Matrix<3> cvMat3x3ToTooNMat(cv::Mat mat)
{
	assert(mat.size() == cv::Size(3,3));
	
	TooN::Matrix<3,3> m; 
	// = TooN::wrapMatrix(fundamental_matrix.data);
	for (unsigned i = 0; i < 3; i++)
	{
		for (unsigned j = 0; j < 3; j++)
		{
			m(i,j) = mat.at<double>(i,j);
		}
	}
	
	return m;
}

}
#endif
