#include <iostream>
#include <iomanip>
#include <map>
#include <algorithm>
#include <vector>

#include <opencv/cv.h>
#include <opencv/highgui.h>

#include <time.h>


#include "cvblob.h"
using namespace cvb;
using namespace std;

bool cmpY(const pair<CvLabel, CvBlob*>  &p1, const pair<CvLabel, CvBlob*> &p2)
{
  return p1.second->centroid.y < p2.second->centroid.y;
}

double dist(CvPoint2D64f u, CvPoint2D64f v)
{
	return sqrt(pow((u.x-v.x),2)+pow((u.y-u.y),2));
}


int main()
{
	cvNamedWindow("blob_analysis", CV_WINDOW_AUTOSIZE);
	//cvNamedWindow("blob_analysis_img", CV_WINDOW_AUTOSIZE);
	//cvNamedWindow("blob_analysis_grayscale", CV_WINDOW_AUTOSIZE);
	//cvNamedWindow("blob_analysis_grayscale_blur", CV_WINDOW_AUTOSIZE);
	cvNamedWindow("blob_analysis_binary", CV_WINDOW_AUTOSIZE);

	CvCapture *capture = cvCaptureFromCAM(-1);
	while (!capture) 
	{
		capture = cvCaptureFromCAM(0);
	}
	cvSetCaptureProperty( capture, CV_CAP_PROP_FRAME_WIDTH, 640 );

	cvSetCaptureProperty( capture, CV_CAP_PROP_FRAME_HEIGHT, 480 );
  
  	cvGrabFrame(capture);
  	IplImage *img = cvRetrieveFrame(capture);

  	CvSize imgSize = cvGetSize(img);

  	IplImage *frame = cvCreateImage(imgSize, img->depth, img->nChannels);
  	//IplConvKernel* morphKernel = cvCreateStructuringElementEx(50, 50, 1, 1, CV_SHAPE_RECT, NULL);
	
	unsigned int frameNumber = 0;
  	unsigned int blobNumber = 0;
	
  	time_t start, end;
  	double sec;
  	time(&start);
  	double fps;
  
 	bool quit = false;
 	while (!quit&&cvGrabFrame(capture))
  	{
    	IplImage *img = cvRetrieveFrame(capture);
    	cvConvertScale(img, frame);
    	//cvShowImage("blob_analysis_img", frame);
    	
    	
		IplImage *grayscale = cvCreateImage(imgSize, IPL_DEPTH_8U, 1);
		cvCvtColor(frame, grayscale, CV_BGR2GRAY);
		//cvShowImage("blob_analysis_grayscale", grayscale);
		
		//cvEqualizeHist(grayscale, grayscale);
		cvSmooth(grayscale, grayscale, CV_BLUR, 5, 5);// try dilate
		//cvShowImage("blob_analysis_grayscale_blur", grayscale);
		
		cvThreshold(grayscale, grayscale, 70, 255, CV_THRESH_BINARY_INV);
		//cvMorphologyEx(grayscale, grayscale, NULL, morphKernel, CV_MOP_OPEN, 1);
		cvShowImage("blob_analysis_binary", grayscale);
		
		IplImage *labelImg=cvCreateImage(cvGetSize(grayscale), IPL_DEPTH_LABEL, 1);
		
		CvBlobs blobs;
		unsigned int result = cvLabel(grayscale, labelImg, blobs);
		cvFilterByArea(blobs, 40, 100000);
		
		vector< pair<CvLabel, CvBlob*> > blobList;
  		copy(blobs.begin(), blobs.end(), back_inserter(blobList));

  		sort(blobList.begin(), blobList.end(), cmpY);
  		vector< vector< pair<CvLabel, CvBlob*> > > groupedBlobs;
		for (int i=0; i<blobList.size(); i++)
		{
			for (int j=i+1; j<=i+4; j++)
			{
				if (j == i || j>=blobList.size()) 
					continue;
				
				if (dist((*blobList[i].second).centroid, (*blobList[j].second).centroid ) <= 3.0)
				{
					vector< pair<CvLabel, CvBlob*> > temp;
					temp.push_back(blobList[i]);
					temp.push_back(blobList[j]);
					groupedBlobs.push_back(temp);
				}
			}
	
		}
		
		vector< vector< pair<CvLabel, CvBlob*> > > targets;
		for (int i=0; i < groupedBlobs.size(); i++)
		{
			if (((*(groupedBlobs[i])[0].second).area/double((*(groupedBlobs[i])[1].second).area) >= 0.25 &&	
				(*(groupedBlobs[i])[0].second).area/double((*(groupedBlobs[i])[1].second).area) <= 0.45 )||
				((*(groupedBlobs[i])[1].second).area/double((*(groupedBlobs[i])[0].second).area) >= 0.25 &&	
				(*(groupedBlobs[i])[1].second).area/double((*(groupedBlobs[i])[0].second).area) <= 0.45 ))
			{
			 	targets.push_back(groupedBlobs[i]);
			}
		}
		
		for (int i=0; i<targets.size(); i++)
		{
			//cout << "Group " << i << ": " << (*(targets[i])[0].second) << (*(targets[i])[1].second) << endl;
		}
		CvBlobs filteredBlobs;
		for (int i=0; i<targets.size(); i++)
		{
			filteredBlobs.insert((targets[i])[0]);
			filteredBlobs.insert((targets[i])[1]);
		}
		
		
		/*for (int i=0; i<blobList.size(); i++)
		{
    		// This will print: [LABEL] -> BLOB_DATA
    		//cout << "[" << blobList[i].first << "] -> " << (*blobList[i].second) << endl;
    		//cvRenderContourChainCode(&((*blobList[i].second).contour), frame, CV_RGB(255., 0., 0.));
    		//cv::RotatedRect box = cv::fitEllipse(*cvConvertChainCodesToPolygon(&((*blobList[i].second).contour)));
    		//cvEllipseBox(frame, CvBox2D(box), CV_RGB(255., 0., 0.));
  		}*/

		
    	cvRenderBlobs(labelImg, filteredBlobs, frame, frame, CV_BLOB_RENDER_CENTROID|CV_BLOB_RENDER_BOUNDING_BOX);
    	
    	cvShowImage("blob_analysis", frame);
    	
    	char k = cvWaitKey(10)&0xff;
    	switch (k)
    	{
     	 	case 27:
      		case 'q':
      		case 'Q':
        		quit = true;
        		break;
      		case 's':
      		case 'S':
       	 		for (CvBlobs::const_iterator it=blobs.begin(); it!=blobs.end(); ++it)
        		{
         			std::stringstream filename;
          			filename << "redobject_blob_" << std::setw(5) << std::setfill('0') << blobNumber << ".png";
          			cvSaveImageBlob(filename.str().c_str(), img, it->second);
         	 		blobNumber++;

          			std::cout << filename.str() << " saved!" << std::endl;
        		}
        		break;
    	}
    	
    	cvReleaseImage(&grayscale);
    	cvReleaseImage(&labelImg);
    	cvReleaseBlobs(blobs);
	
		time(&end);
		++frameNumber;
		sec = difftime (end, start);
		fps = frameNumber / sec;
		//std::cout << fps << "\n";
    }
    
    cvReleaseImage(&frame);
  	cvDestroyWindow("blob_analysis");
  	//cvDestroyWindow("blob_analysis_img");
  	//cvDestroyWindow("blob_analysis_grayscale");
  	//cvDestroyWindow("blob_analysis_grayscale_blur");
  	cvDestroyWindow("blob_analysis_binary");

  	return 0;
}
