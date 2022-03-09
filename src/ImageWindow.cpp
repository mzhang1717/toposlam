#include <toposlam/ImageWindow.h>

ImageWindow::ImageWindow(std::string strWindowName, int nKey) : m_strWindowName(strWindowName), m_keyCommand(nKey)
{
    CreateWindow();
    Init();
}

ImageWindow::~ImageWindow()
{
    cv::destroyWindow(m_strWindowName);    

}



void ImageWindow::CreateWindow()
{
    cv::namedWindow(m_strWindowName);
    ROS_INFO_STREAM("Create OpenCV window: " << m_strWindowName);
}

void ImageWindow::Init()
{
	m_bToResize = true;
}

void ImageWindow::RequestResize(bool bResize)
{
	m_bToResize = bResize;
}

//Get total size of all images placed in two columns
//Get the location (ROI) of each image in the merged frame
void ImageWindow::GetImageSizes(ImageBWMap imagesBW)
{
	int width = 0; // number of columns
	int height = 0; // number of rows
	int width_total = 0;
	int height_total = 0;
	int height_total_previous = 0;
	
	int i = 0;
	for (ImageBWMap::iterator it = imagesBW.begin(); it != imagesBW.end(); ++it, i++)
	{
		if (i % 2 == 0)
		{
			width = it->second.cols;
			height = it->second.rows;
			
			height_total_previous = height_total;
			height_total += height;
			
			m_roiMap[it->first] = cv::Rect(0, height_total_previous, width, height);	
		}
		else 	//(i %  2 == 1)
		{
			int w = width;
			int h = it->second.rows;
			
			width += it->second.cols;
			height_total = std::max(height_total, height_total_previous + h);
			
			m_roiMap[it->first] = cv::Rect(w, height_total_previous, it->second.cols,  h);
		}
		
		width_total = std::max(width_total, width);		
	}
	
	m_TotalSize = cv::Size(width_total, height_total);
	m_CurrentFrame.create(m_TotalSize, CV_8UC3);
	
	//std::cout << "Total size: " << m_TotalSize << std::endl;
	
	m_roiImage.clear();
	//std::cout << "ROIs: " << std::endl;
	for (ROIMap::iterator it = m_roiMap.begin(); it != m_roiMap.end(); ++it)
	{
		//std::cout << it->second << std::endl;
		m_roiImage.push_back(m_CurrentFrame(it->second));
	}
}

void ImageWindow::DrawFrame(ImageBWMap imagesBW)
{
    if (imagesBW.empty())
    {
        std::cout << "Empty image!" << std::endl;
        return;
    }
    
    if ( m_bToResize == true)
    {
		GetImageSizes(imagesBW);
		m_bToResize = false;
	}
    
	m_CurrentFrame.setTo(cv::Scalar(0, 0, 0));
    
    int roiIndex = 0;
    for (ImageBWMap::iterator it = imagesBW.begin(); it != imagesBW.end(); ++it, roiIndex++)
    {
        cv::Mat outImage; 
        if (it->second.channels() == 1)
        {
            std::vector<cv::Mat> tempImage;
            tempImage.push_back(it->second);
            tempImage.push_back(it->second);
            tempImage.push_back(it->second);
            cv::merge(tempImage, outImage);
        }
        else
        {
            it->second.copyTo(outImage);
        }
        
        cv::Point vecTextLocation(30, 30);
        cv::putText(outImage, it->first, vecTextLocation, cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 0, 255));
        
        //std::cout << "Out image size = " << outImage.rows << " x " << outImage.cols << std::endl;
        outImage.copyTo(m_roiImage.at(roiIndex));
        //std::cout << "Index = " << roiIndex << std::endl;
    }
    
    //std::cout << "ROI image size = " << roiImage.size() << std::endl;
    //std::cout << "Final image size = " << dst.rows << " x " << dst.cols << std::endl;
    
    cv::imshow(m_strWindowName, m_CurrentFrame);
    m_keyCommand = cv::waitKey(3);        	
}


//void ImageWindow::DrawFrame(ImageBWMap imagesBW)
//{
    //if (imagesBW.empty())
    //{
        //std::cout << "Empty image!" << std::endl;
        //return;
    //}
    
    //unsigned int countImages =  imagesBW.size();
    
    //std::cout << countImages <<" images are received!" << std::endl;
    
    //ImageBWMap::iterator it = imagesBW.begin();
    //cv::Mat image = it->second;
    
    ////std::cout << "Original image size = " << image.rows << " x " << image.cols << std::endl;
    ////std::cout << "Channels of images = " << image.channels() << std::endl;
 
    //cv::Mat dst;
    //std::vector<cv::Mat> roiImage;
    
    //if (countImages ==1 || countImages == 2)
    //{
        //int dstRows = image.rows * 1;
        //int dstCols = image.cols * countImages;
       
        //dst.create(dstRows, dstCols, CV_8UC3); // create an image that can contain up to two (1x2) sub-images
        
        //roiImage.push_back(dst(cv::Rect(0, 0, image.cols, image.rows)));
        //if (countImages==2)
		//{
			//roiImage.push_back(dst(cv::Rect(image.cols, 0, image.cols, image.rows)));
		//}
    //}
    //else if (countImages == 3 || countImages == 4 )
    //{
        //int dstRows = image.rows * 2;
        //int dstCols = image.cols * 2;
       
        //dst.create(dstRows, dstCols, CV_8UC3); // create an image that can contain up to four (2x2) sub-images
        
        //roiImage.push_back(dst(cv::Rect(0, 0, image.cols, image.rows)));
        //roiImage.push_back(dst(cv::Rect(image.cols, 0, image.cols, image.rows)));
        //roiImage.push_back(dst(cv::Rect(0, image.rows, image.cols, image.rows)));
        //if (countImages == 4)
        //{
			//roiImage.push_back(dst(cv::Rect(image.cols, image.rows, image.cols, image.rows)));
		//}
    //}
    //else
    //{
        //std::cout << "Number of received images is greater than 4!" << std::endl;
    //}
    
    //dst.setTo(cv::Scalar(0, 0, 0));
    
    //int roiIndex = 0;

    //for (ImageBWMap::iterator it = imagesBW.begin(); it != imagesBW.end(); it++)
    //{
        //cv::Mat outImage; 
        //if (it->second.channels() == 1)
        //{
            //std::vector<cv::Mat> tempImage;
            //tempImage.push_back(it->second);
            //tempImage.push_back(it->second);
            //tempImage.push_back(it->second);
            //cv::merge(tempImage, outImage);
        //}
        //else
        //{
            //it->second.copyTo(outImage);
        //}
        
        //cv::Point vecTextLocation(30, 30);
        //cv::putText(outImage, it->first, vecTextLocation, cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 0, 255));
        
        ////std::cout << "Out image size = " << outImage.rows << " x " << outImage.cols << std::endl;
        //outImage.copyTo(roiImage[roiIndex]);
        ////std::cout << "Index = " << roiIndex << std::endl;
        //roiIndex++;
    //}
    
    ////std::cout << "ROI image size = " << roiImage.size() << std::endl;
    ////std::cout << "Final image size = " << dst.rows << " x " << dst.cols << std::endl;
    
    //cv::imshow(m_strWindowName, dst);

    //cv::waitKey(3);    
//}

void ImageWindow::DrawFrame(ImageBWMap imagesBW, KeyPointMap keypoint)
{
    if (imagesBW.empty())
    {
        std::cout << "Empty image!" << std::endl;
        return;
    }
    
    ImageBWMap img_kp;
    
    for (ImageBWMap::iterator it = imagesBW.begin(); it != imagesBW.end(); it++)
    {
        cv::Mat outImage; 
        cv::drawKeypoints(it->second, keypoint[it->first], outImage, cv::Scalar::all(-1));
        outImage.copyTo(img_kp[it->first]);
    }
    
    DrawFrame(img_kp);
}

int ImageWindow::GetKeyCommand() 
{
	int temp = m_keyCommand;
	m_keyCommand = 0;
	return temp;
}
