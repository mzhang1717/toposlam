#include <iostream>
#include <vector>
#include <ros/ros.h>
#include <toposlam/MultiCameraSubscriber.h>
#include <toposlam/Mapping.h>
#include <toposlam/VocabTree.h>

int main(int argc, char **argv)
{
    std::string filename = "/home/mingfeng/Descriptors.dat";
    
    cv::FileStorage fs;
    
    fs.open(filename, cv::FileStorage::READ);
    
    if (!fs.isOpened())
    {
        std::cerr << "Failed to open " << filename << std::endl;
        return -1;
    }
    
    cv::Mat vDescriptorPool;
    cv::Mat vDescriptorIndex;
    
    fs["Descriptors"] >> vDescriptorPool;
    fs["Index"] >> vDescriptorIndex;
    
    VocabTree  myTree(6, 10,100, IDF); // 6 levels x 10 branches, 10e6 leaf nodes  BINARY, TF, TFIDF, IDF
    myTree.BuildBatch(vDescriptorPool, vDescriptorIndex);
    
    //std::cout << "Total Features: " << vDescriptorIndex.rows << std::endl;
    int nTruthIndex[6] = {0};
      
    for (int queryImage = 0; queryImage <= vDescriptorIndex.at<int>(vDescriptorIndex.rows-1); queryImage++)
    {
        int i=0;
        int nBegin;
        int nEnd;
        
        while (i< vDescriptorIndex.rows)
        {
            if (vDescriptorIndex.at<int>(i) == queryImage)
            {
                nBegin = i;
                break;
            }
            i++;
        }
        
        while (i<= vDescriptorIndex.rows)
        {
            if (vDescriptorIndex.at<int>(i) != queryImage || i == vDescriptorIndex.rows)
            {
                nEnd = i-1;
                break;
            }
            i++;        
        }
        
        std::cout << "Query image ID: " << queryImage << "(" << nBegin << " - " << nEnd << "): " ;
        cv::Mat queryDesc = vDescriptorPool.rowRange(nBegin, nEnd);
        
        std::map<double, IDType> vMatchedFrames = myTree.QueryFrame(queryDesc);
        
        int k = 0;
      
        for (std::map<double, IDType>::iterator it = vMatchedFrames.begin(); it != vMatchedFrames.end(); ++it)
        {
			std:: cout << it->second << "(" << it->first << "), ";
			
			if (queryImage == it->second)
				nTruthIndex[k] += 1;
				
			k++;
		}
		
		std::cout << std::endl;
    }
    
    nTruthIndex[5] = vDescriptorIndex.at<int>(vDescriptorIndex.rows-1) + 1  - nTruthIndex[0] - nTruthIndex[1] - nTruthIndex[2] - nTruthIndex[3] - nTruthIndex[4];
    
    std::cout << "No. candidates containing true match: ";
    for (int j = 0; j < 6; j++)
    {
		std::cout << nTruthIndex[j] << " " ;
	}
    std::cout << std::endl;
    
    return 0;
}


