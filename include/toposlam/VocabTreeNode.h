#ifndef VOCABTREENODE_
#define VOCABTREENODE_

#include <opencv2/opencv.hpp>

typedef unsigned long IDType;

enum ScoreScheme
{
	BINARY,
	TF,
	TFIDF,
	IDF
};

class VocabTreeNode
{
private:
    IDType m_nID; // unique ID of the current node, starting from 0
    static IDType m_snCountNodes; //total number of nodes in the tree
    
    // Generate the ID for a node when it is created
    // Since VocabTreeNode is an abstract base class (because it contains pure virtual functions),
    // VocabTreeNode is never instaniated, so GenerateID can only executed by classes derived from VocabTreeNode
    void GenerateID(){ m_nID = m_snCountNodes++; } 
    
protected:    
    double m_fIDF; // inverse document frequency of a word (node), = lg(total number to frames in the tree / number of frames containing this word)
    
    cv::Mat m_vClusterCenter; // a vector storing the center of this cluster
    
    // Calculate the number of frames containing this word (node)
    void CalculateTF(cv::Mat& vFrameIndexOfDescriptor);
    
    // Calculate the each frame's weight within a node (word)
    // m_MapFrameWeight.second = m_MapFrameFrequency.second * m_fWeight
    void CalculateTFIDF();    
    
public:
    VocabTreeNode();
    virtual ~VocabTreeNode(); // virtual deconstructor
    
    // Return the ID of a node
	long GetID() const {return m_nID;}

    // return the total number of nodes in the tree
    long GetNumberOfNodes() const {return VocabTreeNode::m_snCountNodes;}
    
    
    // Cluster a set of feature points in a BATCH manner
    // pure virtual function, must be redefined in derived classes 
    virtual void BuildRecursive(cv::Mat& vDescriptorPool, cv::Mat& vFrameIndexOfDescriptor, int nTotalFrames, int nDepth, int nDepthMax, int nBranch, int nLeafSize) = 0;
    
    // Find the closest word (leaf node) for a  single given feature descriptor
    // pure virtual function, must be redefined in derived classes 
    virtual void SearchFeature(cv::Mat& vFeature, VocabTreeNode *pMatchedNode) = 0; 
    
    // Calculate the weight of each word (node)
    // // weight of current word (node), = ln(number of frames containing this word / total number to frames in the tree)
    void CalculateIDF(cv::Mat& vFrameIndexOfDescriptor, int nTotalFrames);
    
    double GetWordWeight() const { return m_fIDF; }
    
    double GetFrameWeight(IDType nFrameId, ScoreScheme type = IDF) const;
    

    // Set the center of a clustered set
    void SetClusterCenter(cv::Mat vClusterCenter);

    // Get the center of a clustered set
    cv::Mat GetClusterCenter() const { return m_vClusterCenter; }
 
     // a list of pointers pointing to its child nodes
    std::vector< VocabTreeNode* > m_pChildNodes;
    VocabTreeNode* m_pParentNode; // point to its parent node, not used at the moment
  
    std::map<IDType, int> m_mTF; // a map of the index of a frame and the number of the occurrence of the current word (node) in this frame)
    std::map<IDType, double> m_mTFIDF; // m_mTFIDF.second = m_mTF.second * m_fIDF
};

class VocabTreeInteriorNode : public VocabTreeNode
{
public:
    VocabTreeInteriorNode() { };
    ~VocabTreeInteriorNode();
 
    virtual void BuildRecursive(cv::Mat& vDescriptorPool, cv::Mat& vFrameIndexOfDescriptor, int nTotalFrames, int nDepth, int nDepthMax, int nBranch, int nLeafSize);
    virtual void SearchFeature(cv::Mat& vFeature, VocabTreeNode *pMatchedNode);
};

class VocabLeafNode : public VocabTreeNode
{
public:
    VocabLeafNode() { };
    ~VocabLeafNode() { };
    
    virtual void BuildRecursive(cv::Mat& vDescriptorPool, cv::Mat& vFrameIndexOfDescriptor, int nTotalFrames, int nDepth, int nDepthMax, int nBranch, int nLeafSize);   
    virtual void SearchFeature(cv::Mat& vFeature, VocabTreeNode *pMatchedNode);
};

#endif
