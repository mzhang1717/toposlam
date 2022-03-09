#ifndef VOCABTREE_
#define VOCABTREE_

#include <opencv2/opencv.hpp>
#include <toposlam/VocabTreeNode.h>
#include <toposlam/Types.h>

class VocabTree
{
public:    
    VocabTree(int nDepth = 6, int nBranch = 10, int nLeafSize = 100, ScoreScheme eScore = IDF);
    ~VocabTree();
    
    void BuildBatch(cv::Mat& vDescriptorPool, cv::Mat& vFrameIndexOfDescriptor);
    std::map<double, IDType> QueryFrame(cv::Mat& vDescriptors, int nMatched = 5);
    void VisualizeTree();
    
    void CopyDescriptorPool(const cv::Mat& mDescPool, const cv::Mat& mDescIndex);
    cv::Mat GetDescriptors(int nFrameIndex);
    
    void SetVOData(unsigned int nIndex, const cv::Mat& desc, const cv::Mat& points);
    void SaveVOData(std::string filename);
    void LoadVOData(std::string filename);
   
    MatIndexMap m_mVODescriptors;
    MatIndexMap m_mVOPoints;   
    
private:
    int m_nDepth; // number of levels of the tree
    int m_nBranch; // number of sub-branches of each node
    int m_nLeafSize; // size of the bottom node
    VocabTreeInteriorNode* m_pNodeRoot; // Pointer to tree root
        
    ScoreScheme m_enumScore;
    int m_nTotoalFrames;
    cv::Mat m_mDescriptorPool;
    cv::Mat m_mDescriptorIndex;
};

#endif
