#include <iostream>
#include <iomanip>


#include <opencv/cv.h>
#include <opencv/highgui.h>

#include <time.h>


#include "cvblob.h"
using namespace cvb;

int main()
{
  CvTracks tracks;

  cvNamedWindow("red_object_tracking", CV_WINDOW_AUTOSIZE);

  CvCapture *capture = cvCaptureFromCAM(0);
  cvSetCaptureProperty( capture, CV_CAP_PROP_FRAME_WIDTH, 640 );

  cvSetCaptureProperty( capture, CV_CAP_PROP_FRAME_HEIGHT, 480 );
  
  cvGrabFrame(capture);
  IplImage *img = cvRetrieveFrame(capture);

  CvSize imgSize = cvGetSize(img);

  IplImage *frame = cvCreateImage(imgSize, img->depth, img->nChannels);

  //IplConvKernel* morphKernel = cvCreateStructuringElementEx(2, 2, 1, 1, CV_SHAPE_RECT, NULL);

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

    cvConvertScale(img, frame, 1, 0);

    //IplImage *segmentated = cvCreateImage(imgSize, 8, 1);
    
    IplImage*  grayscale    = cvCreateImage(imgSize, IPL_DEPTH_8U, 1);
    cvCvtColor(img,grayscale,CV_BGR2GRAY);
    //cvShowImage("grayscale", grayscale);
    CvScalar gray_min = cvScalar(0, 0, 0, 0);
    CvScalar gray_max = cvScalar(50, 0, 0, 0);
    IplImage*  segmentated    = cvCreateImage(imgSize, IPL_DEPTH_8U, 1);
    cvInRangeS(grayscale, gray_min, gray_max, segmentated);
    //cvShowImage("thresholded", segmentated);
    // Detecting red pixels:
    // (This is very slow, use direct access better...)
    /*for (unsigned int j=0; j<imgSize.height; j++)
      for (unsigned int i=0; i<imgSize.width; i++)
      {
	CvScalar c = cvGet2D(frame, j, i);

	double b = ((double)c.val[0])/255.;
	double g = ((double)c.val[1])/255.;
	double r = ((double)c.val[2])/255.;
	unsigned char f = 255*((r>0.2+g)&&(r>0.2+b));

	cvSet2D(segmentated, j, i, CV_RGB(f, f, f));
      }
      
    */

    //cvMorphologyEx(segmentated, segmentated, NULL, morphKernel, CV_MOP_OPEN, 1);

    //cvShowImage("segmentated", segmentated);

    IplImage *labelImg = cvCreateImage(cvGetSize(frame), IPL_DEPTH_LABEL, 1);

    CvBlobs blobs;
    unsigned int result = cvLabel(segmentated, labelImg, blobs);
    //std::cout << result << "\t";
    cvFilterByArea(blobs, 40, 100000);
   // std::cout << blobs.size() << "\n";
    cvRenderBlobs(labelImg, blobs, frame, frame, CV_BLOB_RENDER_CENTROID|CV_BLOB_RENDER_BOUNDING_BOX);
    //cvShowImage("frame", frame);
    //cvUpdateTracks(blobs, tracks, 20., 5);
    //cvRenderTracks(tracks, frame, frame, CV_TRACK_RENDER_ID|CV_TRACK_RENDER_BOUNDING_BOX);
    //std::cout << tracks.size() << "\n";
    cvShowImage("red_object_tracking", frame);

    /*std::stringstream filename;
    filename << "redobject_" << std::setw(5) << std::setfill('0') << frameNumber << ".png";
    cvSaveImage(filename.str().c_str(), frame);*/

    cvReleaseImage(&labelImg);
    cvReleaseImage(&segmentated);

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

    cvReleaseBlobs(blobs);
	
	time(&end);
	++frameNumber;
	sec = difftime (end, start);
	fps = frameNumber / sec;
    //std::cout << fps << "\n";
  }

  //cvReleaseStructuringElement(&morphKernel);
  cvReleaseImage(&frame);

  cvDestroyWindow("red_object_tracking");

  return 0;
}
