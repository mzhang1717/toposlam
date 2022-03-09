#include <math.h>
#include <toposlam/VocabTreeNode.h>

unsigned long VocabTreeNode::m_snCountNodes = 0;

VocabTreeNode::VocabTreeNode()
{
    GenerateID();
}

VocabTreeNode::~VocabTreeNode()
{
}

void VocabTreeNode::SetClusterCenter(cv::Mat vClusterCenter)
{
    vClusterCenter.copyTo(m_vClusterCenter);
    return;
}

/// Calculagte the number of frames (TF, term frequency) that contain this node (word)
/// vFrameIndexOfDescriptor is a list of ID of frames that contain this node (word)
/// count the number of each ID and store it in m_mTF
void VocabTreeNode::CalculateTF(cv::Mat& vFrameIndexOfDescriptor)
{
    cv::MatIterator_<int> it, end;
    for (it = vFrameIndexOfDescriptor.begin<int>(), end = vFrameIndexOfDescriptor.end<int>(); it != end; ++it)
    {
        if (m_mTF.empty() || m_mTF.count(*it) == 0)// if the ID is not the m_mTF
        {
            m_mTF.insert(std::pair <int, int>(*it, 1));
        }
        else // increase the count if the ID is already in m_nTF
        {
            m_mTF[*it]++;
        }
    }
    
    return;
}

/// calculate the inverse document frequency of each node/word
/// idf =  lg (number of images in the tree / number of images that contain this node/word)
void VocabTreeNode::CalculateIDF(cv::Mat& vFrameIndexOfDescriptor, int nTotalFrames)
{
    CalculateTF(vFrameIndexOfDescriptor);
    
    int nCountFrames = m_mTF.size();
    
    //assert(nCountFrames != 0);
    
    //std::cout << "TF-IDF: " <<  nCountFrames << "/" << nTotalFrames << std::endl;
    if (nCountFrames > 0)
        m_fIDF = log((double)nTotalFrames/nCountFrames);
    else
        m_fIDF = 0.0;
        
    CalculateTFIDF();    
}

/// Calculate the TF-IDF for each frame that contains this node (word)
/// m_mTFIDF.second = m_mTF.second * m_fIDF
void VocabTreeNode::CalculateTFIDF()
{
    std::map<IDType, int>::iterator it, end;
    for (it = m_mTF.begin(), end = m_mTF.end(); it != end; ++it)
    {
        m_mTFIDF[it->first] = m_fIDF * (it->second);
    }
} 

//void VocabTreeNode::CalculateFrameWeight()
//{
    ////std::map<int, int>::iterator it, end;
    ////for (it = m_mTF.begin(), end = m_mTF.end(); it != end; ++it)
    ////{
        ////m_mTFIDF[it->first] = m_fIDF * (it->second);
    ////}
//} 

//double VocabTreeNode::GetWordWeight() const
//{
	//return m_fIDF;
//}

double VocabTreeNode::GetFrameWeight(IDType nFrameId, ScoreScheme type) const
{
	switch (type)
	{
		case BINARY:
			return 1.0;
			break;
		case TF:
			return m_mTF.at(nFrameId);
			break;
		case TFIDF:
			return m_mTFIDF.at(nFrameId);
			break;
		case IDF:
			return m_fIDF;
			break;
		default:
			return m_mTFIDF.at(nFrameId);
			std::cerr << "Warning: No frame weight type is specified! TFIDF is used by default!" << std::endl;
			break;
	}
}

/////////////////////////////////////////////////
VocabTreeInteriorNode::~VocabTreeInteriorNode()
{
    for (size_t i = 0; i< m_pChildNodes.size(); i++)
	{
		if (m_pChildNodes[i] != NULL) delete m_pChildNodes[i];
	}
}

void VocabTreeInteriorNode::BuildRecursive(cv::Mat& vDescriptorPool, cv::Mat& vFrameIndexOfDescriptor, int nTotalFrames, int nDepth, int nDepthMax, int nBranch, int nLeafSize)
{
    std::cout << "Clustering at level: " << nDepth << std::endl;
    
    cv::Mat labels;
    cv::Mat centers;
    cv::kmeans(vDescriptorPool, nBranch, labels,
               cv::TermCriteria( CV_TERMCRIT_EPS+CV_TERMCRIT_ITER, 50, 1.0),
               3, cv::KMEANS_PP_CENTERS, centers);

    //TermCriteria::TermCriteria(int type, int maxCount, double epsilon)
    
    std::vector< cv::Mat > vFrameIndexOfDescInCluster(nBranch); //store the index of descriptors for each cluster center
    std::vector< cv::Mat > vDescPoolInCluster(nBranch);
        
    for (int i = 0; i < labels.rows; i++)
    {
        if (!vFrameIndexOfDescInCluster[labels.at<int>(i)].empty())
        {
            cv::Mat temp(1, 1, CV_32S, cv::Scalar(vFrameIndexOfDescriptor.at<int>(i)));
            vFrameIndexOfDescInCluster[labels.at<int>(i)].push_back(temp);
            vDescPoolInCluster[labels.at<int>(i)].push_back(vDescriptorPool.row(i)); // + 0 to mimic copyto()
        }
        else
        {
            cv::Mat temp(1, 1, CV_32S, cv::Scalar(vFrameIndexOfDescriptor.at<int>(i)));
            vFrameIndexOfDescInCluster[labels.at<int>(i)] = temp;
            vDescPoolInCluster[labels.at<int>(i)] = vDescriptorPool.row(i); // + 0 to mimic copyto()
        }
    } 
              
    for (int i = 0; i < nBranch; i++)
    {
        VocabTreeNode* newNode;
        
        if (nDepth < nDepthMax && vDescPoolInCluster[i].rows > nLeafSize)
        {
            newNode = new VocabTreeInteriorNode();
        }
        else
        {
            std::cout << "Operating at the bottom level (" << nDepth << "): " ;
            //create leaf nodes for each center
            newNode = new VocabLeafNode();
        } 
          
        // set cluster center
        newNode->SetClusterCenter(centers.row(i));
        
        // collect descriptors for the current node, 
        // if the size is larger than a threshold, apply kmean to this node recursively
        //if (vDescPoolInCluster[i].rows > 200)
        //{
            newNode->BuildRecursive(vDescPoolInCluster[i], vFrameIndexOfDescInCluster[i], nTotalFrames, nDepth + 1, nDepthMax, nBranch, nLeafSize);
            
        //}
        
        newNode->CalculateIDF(vFrameIndexOfDescInCluster[i], nTotalFrames); // calculate the weight of a word (node)
        m_pChildNodes.push_back(newNode);
        std::cout << "Total nodes: " << GetNumberOfNodes() << " ID: " << newNode->GetID() <<  " Weight: " << newNode->GetWordWeight() << std::endl;
    }
    
    return;
}

void VocabTreeInteriorNode::SearchFeature(cv::Mat& vFeature, VocabTreeNode *pMatchedNode)
{
    //std::cout << "Feature column: " << vFeature.rows << std::endl;
    
    assert(vFeature.rows == 1);
    
    VocabTreeNode* pNode;
    double min_dist =  DBL_MAX;
    int best_index =0;
    
    std::vector<VocabTreeNode*>::size_type sz = m_pChildNodes.size();
    
    for (int i=0; i<sz; i++)
    {
        pNode = m_pChildNodes[i];
        
        if (pNode != NULL)
        {
			//std::cout << "Feature size: " << vFeature.rows << " x " << vFeature.cols << std::endl;
			//std::cout << "Word size: " << pNode->m_vClusterCenter.rows << " x " << pNode->m_vClusterCenter.cols << std::endl;
            double dist = cv::norm(vFeature, pNode->GetClusterCenter()); 
            //double norm(InputArray src1, InputArray src2, int normType=NORM_L2, InputArray mask=noArray() )
            
            if (dist < min_dist)
            {
                min_dist = dist;
                best_index = i;
            }
        }
        
    }
    
    m_pChildNodes[best_index]->SearchFeature(vFeature, pMatchedNode);
}

//////////////////////////////////////////

void VocabLeafNode::BuildRecursive(cv::Mat& vDescriptorPool, cv::Mat& vFrameIndexOfDescriptor, int nTotalFrames, int nDepth, int nDepthMax, int nBranch, int nLeafSize)
{
    std::cout << "Generate a leaf " << std::endl;
    return;
}

void VocabLeafNode::SearchFeature(cv::Mat& vFeature, VocabTreeNode *pMatchedNode)
{
    // return the matched (leaf) node/word
    pMatchedNode = this;
    //std::cout << "Matched node: " << GetID() << std::endl;
}
