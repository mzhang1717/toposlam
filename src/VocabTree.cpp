#include <assert.h>
#include <toposlam/VocabTree.h>
#include <toposlam/Utility.h>
#include <fstream>
#include <boost/serialization/vector.hpp>
#include <boost/serialization/map.hpp>
#include <boost/archive/archive_exception.hpp>
#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>

VocabTree::VocabTree(int nDepth, int nBranch, int nLeafSize, ScoreScheme eScore): m_nDepth(nDepth), m_nBranch(nBranch), m_nLeafSize(nLeafSize), m_enumScore(eScore)
{
	ROS_INFO_STREAM("Create Vocabulary Tree: " << m_nDepth << " levels and "<< m_nBranch << " branches" << " with leaf size: " << nLeafSize);
}

VocabTree::~VocabTree()
{
    if (m_pNodeRoot != NULL)
        delete m_pNodeRoot;
}

void VocabTree::BuildBatch(cv::Mat& vDescriptorPool, cv::Mat& vFrameIndexOfDescriptor)
{
    assert(vDescriptorPool.rows == vFrameIndexOfDescriptor.rows); // make sure each descriptor's frame index is recorded
    assert(vFrameIndexOfDescriptor.cols == 1);
    
    CopyDescriptorPool(vDescriptorPool, vFrameIndexOfDescriptor);
    
    m_nTotoalFrames = vFrameIndexOfDescriptor.at<int>(vFrameIndexOfDescriptor.rows - 1) + 1; // find the number of frames
    
    std::cout << "Building vocabulary tree from " << vDescriptorPool.rows << " features in " << m_nTotoalFrames << " frames ..." << std::endl;
    std::cout << "Depth: " << m_nDepth << ", Branching factor: " << m_nBranch << std::endl;
    
    m_pNodeRoot = new VocabTreeInteriorNode();
    
    int nDepthCurrent = 0;
    
    m_pNodeRoot->BuildRecursive(vDescriptorPool, vFrameIndexOfDescriptor, m_nTotoalFrames, nDepthCurrent, m_nDepth, m_nBranch, m_nLeafSize);
    
    return;
}

void VocabTree::VisualizeTree()
{
   // std::cout << "Total number of nodes: " << m_pNodeRoot->GetNumberOfNodes() << std::endl;
    
   // std::cout << "ID of nodes: " << std::endl;
    
}

std::map<double, IDType> VocabTree::QueryFrame(cv::Mat& vDescriptors, int nCandidate)
{
    assert(nCandidate > 0); // make sure the number of requested candidates > 0
    
    int nCountFeatures = vDescriptors.rows; // get the number of features
    
	std::vector<IDType> vWordID; // ID of words found in the query image
    std::map<IDType, int> mWordTF; // term frequency of a word found in the query image, i.e., the number of features in a query image that are matched with the same one word in VocTree
    std::vector<double> vWordIDF;  // weight of words found in the query image, such as inverse document frequency  
    
    std::vector<double> vQueryVOC; // VOC representation of the query image
    std::map< IDType, std::vector<double> > mFrameVOC; // VOC representation of each image in the VocTree

    for (int i=0; i < nCountFeatures; i++)
    {
        VocabTreeNode *pMatchedNode;
        cv::Mat vFeature;
        
        //Find the matching node (visual word) for each query feature
        vDescriptors.row(i).copyTo(vFeature); // get the ith feature
        m_pNodeRoot->SearchFeature(vFeature, pMatchedNode); // find the matching word for ith feature
        IDType nMatchedID = pMatchedNode->GetID(); // get the ID of the matched word
        
        if(mWordTF.count(nMatchedID) == 0) //  if the matched word has not been matched with other features of the query image
        {
			//std::cout << "Matched ID = " << pMatchedNode.GetID() << std::endl;
 			mWordTF[nMatchedID] = 1; // add a record of this newly matched word
	        vWordID.push_back(nMatchedID); // save the ID of this matched word (node)
	        vWordIDF.push_back(pMatchedNode->GetWordWeight());// get weight of the matched word
	        
	        for(std::map< IDType, std::vector<double> >::iterator it = mFrameVOC.begin(); it != mFrameVOC.end(); ++it)
	        {
				it->second.push_back(0.0);
			}
	        
	        //record the weight of each frame associated with matched nodes
	        std::map<IDType, double>::iterator it, end;
	        for (it = pMatchedNode->m_mTFIDF.begin(), end = pMatchedNode->m_mTFIDF.end(); it != end; ++it)
	        {
				IDType nFrameID = it->first;
				if (mFrameVOC.count(nFrameID) > 0) // the td-idf/weight vector of each matched frame
				{
					mFrameVOC.at(nFrameID).pop_back();
				}
				
				double fFrameWeight = pMatchedNode->GetFrameWeight(nFrameID, m_enumScore); //get TF-IDF
				
				mFrameVOC[nFrameID].push_back(fFrameWeight); 
	        }	        
		}
		else // multiple features matched with the same word
		{
			mWordTF.at(nMatchedID) += 1; // keep tracking the number of features that are matched with the same word
		}
     }

	// Generate the BoW representation for the query image : vQueryVOC
	switch (m_enumScore)
	{
		case TF:
		{
			for (std::map<IDType, int>::iterator it = mWordTF.begin(); it != mWordTF.end(); ++it)
			{
				vQueryVOC.push_back(it->second);
			}
			break;
		}
		case TFIDF:
		{
			int i ;
			std::map<IDType, int>::iterator it;
			for (i = 0, it = mWordTF.begin(); it != mWordTF.end(); ++it)
			{
				vQueryVOC.push_back(vWordIDF[i] * (it->second));
				i++;
			}			
			break;
		}
		case IDF:
		{
			vQueryVOC = vWordIDF;
			break;
		}
		case BINARY:
		{
			for (std::map<IDType, int>::iterator it = mWordTF.begin(); it != mWordTF.end(); ++it)
			{
				vQueryVOC.push_back(1.0);
			}				
			break;
		}
		default:
		{
			int i ;
			std::map<IDType, int>::iterator it;
			for (i =0,  it = mWordTF.begin(); it != mWordTF.end(); ++it)
			{
				vQueryVOC.push_back(vWordIDF[i] * (it->second));
				i++;
			}			
			break;
		}
	}
 
    std::map<double, IDType> vfFrameWeight_Sorted; // = UTIL::flip_map(vfFrameWeight);
    //std::cout << "Diff = ";
    for(std::map< IDType, std::vector<double> >::iterator it = mFrameVOC.begin(); it != mFrameVOC.end(); ++it)
    {
		std::vector<double> vMatchedFrame = it->second;
		double diff = 0.0;
		
		for(size_t i = 0; i < vMatchedFrame.size(); i++)
		{
			double e = vQueryVOC[i] - vMatchedFrame[i];
			//diff += e * e; // L2
			//diff = std::max(diff, e); //L_infinity
			diff += fabs(e);
			//std::cout << vWeight[i] << " ~ "<< vWordWeightQuery[i] << std::endl;
		}
		
		//diff = sqrt(diff);		
		//std::cout << diff << ", ";
		
		vfFrameWeight_Sorted[diff] = it->first;
	} 
    
    //std::cout << std::endl;
    
    //std::cout  << "Matched image ID: " ;
    
    std::map<double, IDType>::iterator it;
    it = vfFrameWeight_Sorted.begin();
    
    std::map<double, IDType> vMatchedFrames;
    
    for (int i = 0; i < std::min((size_t)nCandidate, vfFrameWeight_Sorted.size()); i++)
    {       
        //std::cout << it->second << " (" <<it->first << "), ";
        vMatchedFrames[it->first] = it->second;
		it++;    
    }
    //std::cout << std::endl;
    
    return vMatchedFrames;
}

void VocabTree::CopyDescriptorPool(const cv::Mat& mDescPool, const cv::Mat& mDescIndex)
{
    mDescPool.copyTo(m_mDescriptorPool);
    mDescIndex.copyTo(m_mDescriptorIndex);	
}

cv::Mat VocabTree::GetDescriptors(int nFrameIndex)
{
	int i=0;
	int nBegin;
	int nEnd;
	
	while (i < m_mDescriptorIndex.rows)
	{
		if (m_mDescriptorIndex.at<int>(i) == nFrameIndex)
		{
			nBegin = i;
			break;
		}
		i++;
	}
	
	while (i <= m_mDescriptorIndex.rows)
	{
		if (m_mDescriptorIndex.at<int>(i) != nFrameIndex || i == m_mDescriptorIndex.rows)
		{
			nEnd = i-1;
			break;
		}
		i++;        
	}
	
	//std::cout << "Query image ID: " << nFrameIndex << "(" << nBegin << " - " << nEnd << ");     "  ;
	cv::Mat mDescInFrame = m_mDescriptorPool.rowRange(nBegin, nEnd);	
	
	return mDescInFrame;
}

void VocabTree::SetVOData(unsigned int nIndex, const cv::Mat& desc, const cv::Mat& points)
{
	m_mVODescriptors[nIndex] = desc.clone();
    m_mVOPoints[nIndex] = points.clone();	
}

void VocabTree::SaveVOData(std::string filename)
{
	std::ofstream fout(filename.c_str());
	
	if (!fout)
	{
		std::cerr << filename << " could not be opened for writing!" << std::endl;
		return;
	}
	
	boost::archive::text_oarchive ar(fout);	
	try {	
		ar << BOOST_SERIALIZATION_NVP(m_mVODescriptors); 
		ar << BOOST_SERIALIZATION_NVP(m_mVOPoints);
	}
	catch (const boost::archive::archive_exception& oor) {
		std::cerr << "Exception in SaveVOData(): " << oor.what() << std::endl;
	}
}

void VocabTree::LoadVOData(std::string filename)
{
	std::ifstream fin(filename.c_str());
	
	if (!fin)
	{
		std::cerr << filename << " could not be opened for reading!" << std::endl;
		return;
	}
	
	boost::archive::text_iarchive ar(fin);
	try {
		ar >> BOOST_SERIALIZATION_NVP(m_mVODescriptors);
		ar >> BOOST_SERIALIZATION_NVP(m_mVOPoints);
	}
	catch (const boost::archive::archive_exception& oor) {
		std::cerr << "Exception in LoadVOData(): " << oor.what() << std::endl;
	}
}
