/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2013, The University of Texas at Dallas
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of The University of Texas at Dallas nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */
 
/*
 * Filename: vision_based_localization_node.cpp
 * 
 * Description: This file contains the ROS node that calculates the localization
 * variables from a PS3 eye video stream for the adaptive following node.
 *
 * NOTE: This code contains code copied from other sources so don't publish it publically
 * 
 * Log
 * ----
 * 2014-02-22 File created by Hazen Eckert
 *
 */


// ROS includes
#include "ros/ros.h"
#include "ros/assert.h"

// Library includes
#include <algorithm>
#include <stdio.h>
#include <string.h>
#include <fcntl.h>
#include <inttypes.h>
#include <unistd.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <errno.h>
#include <getopt.h>
#include <math.h>
#include <sched.h>
#include <time.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/select.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <linux/videodev2.h>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>

// Local Package includes
#include "vision_based_localization/VisionBasedLocalizationMsgs.h"

using namespace cv;
using namespace std;
using namespace Eigen;

const int thresh = 45;
const int blur_level = 3;

enum buffer_fill_mode
{
	BUFFER_FILL_NONE = 0,
	BUFFER_FILL_FRAME = 1 << 0,
	BUFFER_FILL_PADDING = 1 << 1,
};

struct buffer
{
	unsigned int padding;
	unsigned int size;
	void *mem;
};

struct device
{
	int fd;

	enum v4l2_buf_type type;
	enum v4l2_memory memtype;
	unsigned int nbufs;
	struct buffer *buffers;

	unsigned int width;
	unsigned int height;
	unsigned int bytesperline;
	unsigned int imagesize;

	void *pattern;
	unsigned int patternsize;
};

int video_free_buffers(struct device *dev)
{
	struct v4l2_requestbuffers rb;
	unsigned int i;
	int ret;

	if (dev->nbufs == 0)
		return 0;

	for (i = 0; i < dev->nbufs; ++i) {
		switch (dev->memtype) {
		case V4L2_MEMORY_MMAP:
			ret = munmap(dev->buffers[i].mem, dev->buffers[i].size);
			if (ret < 0) {
				printf("Unable to unmap buffer %u: %s (%d)\n", i,
					strerror(errno), errno);
				return ret;
			}
			break;

		case V4L2_MEMORY_USERPTR:
			free(dev->buffers[i].mem);
			break;

		default:
			break;
		}

		dev->buffers[i].mem = NULL;
	}

	memset(&rb, 0, sizeof rb);
	rb.count = 0;
	rb.type = dev->type;
	rb.memory = dev->memtype;

	ret = ioctl(dev->fd, VIDIOC_REQBUFS, &rb);
	if (ret < 0) {
		printf("Unable to release buffers: %s (%d).\n",
			strerror(errno), errno);
		return ret;
	}

	printf("%u buffers released.\n", dev->nbufs);

	free(dev->buffers);
	dev->nbufs = 0;
	dev->buffers = NULL;

	return 0;
}

void video_close(struct device *dev)
{
	free(dev->pattern);
	free(dev->buffers);
	close(dev->fd);
}

int video_enable(struct device *dev, int enable)
{
	int type = dev->type;
	int ret;

	ret = ioctl(dev->fd, enable ? VIDIOC_STREAMON : VIDIOC_STREAMOFF, &type);
	if (ret < 0) {
		printf("Unable to %s streaming: %s (%d).\n",
			enable ? "start" : "stop", strerror(errno), errno);
		return ret;
	}

	return 0;
}

void video_verify_buffer(struct device *dev, int index)
{
	struct buffer *buffer = &dev->buffers[index];
	const uint8_t *data = buffer->mem + buffer->size;
	unsigned int errors = 0;
	unsigned int dirty = 0;
	unsigned int i;

	if (buffer->padding == 0)
		return;

	for (i = 0; i < buffer->padding; ++i) {
		if (data[i] != 0x55) {
			errors++;
			dirty = i + 1;
		}
	}

	if (errors) {
		printf("Warning: %u bytes overwritten among %u first padding bytes\n",
		       errors, dirty);

		dirty = (dirty + 15) & ~15;
		dirty = dirty > 32 ? 32 : dirty;

		for (i = 0; i < dirty; ++i) {
			printf("%02x ", data[i]);
			if (i % 16 == 15)
				printf("\n");
		}
	}
}

int video_queue_buffer(struct device *dev, int index, enum buffer_fill_mode fill)
{
	struct v4l2_buffer buf;
	int ret;

	memset(&buf, 0, sizeof buf);
	buf.index = index;
	buf.type = dev->type;
	buf.memory = dev->memtype;
	buf.length = dev->buffers[index].size;
	if (dev->memtype == V4L2_MEMORY_USERPTR)
		buf.m.userptr = (unsigned long)dev->buffers[index].mem;

	if (dev->type == V4L2_BUF_TYPE_VIDEO_OUTPUT) {
		buf.bytesused = dev->patternsize;
		memcpy(dev->buffers[buf.index].mem, dev->pattern, dev->patternsize);
	} else {
		if (fill & BUFFER_FILL_FRAME)
			memset(dev->buffers[buf.index].mem, 0x55, dev->buffers[index].size);
		if (fill & BUFFER_FILL_PADDING)
			memset(dev->buffers[buf.index].mem + dev->buffers[index].size,
			       0x55, dev->buffers[index].padding);
	}

	ret = ioctl(dev->fd, VIDIOC_QBUF, &buf);
	if (ret < 0)
		printf("Unable to queue buffer: %s (%d).\n",
			strerror(errno), errno);

	return ret;
}

int video_alloc_buffers(struct device *dev, int nbufs,
	unsigned int offset, unsigned int padding)
{
	struct v4l2_requestbuffers rb;
	struct v4l2_buffer buf;
	int page_size;
	struct buffer *buffers;
	unsigned int i;
	int ret;

	memset(&rb, 0, sizeof rb);
	rb.count = nbufs;
	rb.type = dev->type;
	rb.memory = dev->memtype;

	ret = ioctl(dev->fd, VIDIOC_REQBUFS, &rb);
	if (ret < 0) {
		printf("Unable to request buffers: %s (%d).\n", strerror(errno),
			errno);
		return ret;
	}

	printf("%u buffers requested.\n", rb.count);

	buffers = malloc(rb.count * sizeof buffers[0]);
	if (buffers == NULL)
		return -ENOMEM;

	page_size = getpagesize();

	/* Map the buffers. */
	for (i = 0; i < rb.count; ++i) {
		const char *ts_type;
		memset(&buf, 0, sizeof buf);
		buf.index = i;
		buf.type = dev->type;
		buf.memory = dev->memtype;
		ret = ioctl(dev->fd, VIDIOC_QUERYBUF, &buf);
		if (ret < 0) {
			printf("Unable to query buffer %u: %s (%d).\n", i,
				strerror(errno), errno);
			return ret;
		}
		switch (buf.flags & V4L2_BUF_FLAG_TIMESTAMP_MASK) {
		case V4L2_BUF_FLAG_TIMESTAMP_UNKNOWN:
			ts_type = "unknown";
			break;
		case V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC:
			ts_type = "monotonic";
			break;
		default:
			ts_type = "invalid";
		}
		printf("length: %u offset: %u timestamp type: %s\n",
		       buf.length, buf.m.offset, ts_type);

		switch (dev->memtype) {
		case V4L2_MEMORY_MMAP:
			buffers[i].mem = mmap(0, buf.length, PROT_READ | PROT_WRITE, MAP_SHARED, dev->fd, buf.m.offset);
			if (buffers[i].mem == MAP_FAILED) {
				printf("Unable to map buffer %u: %s (%d)\n", i,
					strerror(errno), errno);
				return ret;
			}
			buffers[i].size = buf.length;
			buffers[i].padding = 0;
			printf("Buffer %u mapped at address %p.\n", i, buffers[i].mem);
			break;

		case V4L2_MEMORY_USERPTR:
			ret = posix_memalign(&buffers[i].mem, page_size, buf.length + offset + padding);
			if (ret < 0) {
				printf("Unable to allocate buffer %u (%d)\n", i, ret);
				return -ENOMEM;
			}
			buffers[i].mem += offset;
			buffers[i].size = buf.length;
			buffers[i].padding = padding;
			printf("Buffer %u allocated at address %p.\n", i, buffers[i].mem);
			break;

		default:
			break;
		}
	}

	dev->buffers = buffers;
	dev->nbufs = rb.count;
	return 0;
}

int video_prepare_capture(struct device *dev, int nbufs, unsigned int offset,
				  enum buffer_fill_mode fill)
{
	unsigned int padding;
	unsigned int i;
	int ret;

	/* Allocate and map buffers. */
	padding = (fill & BUFFER_FILL_PADDING) ? 4096 : 0;
	if ((ret = video_alloc_buffers(dev, nbufs, offset, padding)) < 0)
		return ret;

	/* Queue the buffers. */
	for (i = 0; i < dev->nbufs; ++i) {
		ret = video_queue_buffer(dev, i, fill);
		if (ret < 0)
			return ret;
	}

	return 0;
}

int video_set_format(struct device *dev, unsigned int w, unsigned int h,
			    unsigned int format, unsigned int stride)
{
	struct v4l2_format fmt;
	int ret;

	memset(&fmt, 0, sizeof fmt);
	fmt.type = dev->type;
	fmt.fmt.pix.width = w;
	fmt.fmt.pix.height = h;
	fmt.fmt.pix.pixelformat = format;
	fmt.fmt.pix.bytesperline = stride;
	fmt.fmt.pix.field = V4L2_FIELD_ANY;

	ret = ioctl(dev->fd, VIDIOC_S_FMT, &fmt);
	if (ret < 0) {
		printf("Unable to set format: %s (%d).\n", strerror(errno),
			errno);
		return ret;
	}

	printf("Video format set: %s (%08x) %ux%u (stride %u) buffer size %u\n",
		v4l2_format_name(fmt.fmt.pix.pixelformat), fmt.fmt.pix.pixelformat,
		fmt.fmt.pix.width, fmt.fmt.pix.height, fmt.fmt.pix.bytesperline,
		fmt.fmt.pix.sizeimage);
	return 0;
}

int video_open(struct device *dev, const char *devname, int no_query)
{
	struct v4l2_capability cap;
	unsigned int capabilities;
	int ret;

	memset(dev, 0, sizeof *dev);
	dev->fd = -1;
	dev->memtype = V4L2_MEMORY_MMAP;
	dev->buffers = NULL;
	dev->type = (enum v4l2_buf_type)-1;

	dev->fd = open(devname, O_RDWR);
	if (dev->fd < 0) {
		printf("Error opening device %s: %s (%d).\n", devname,
		       strerror(errno), errno);
		return dev->fd;
	}

	printf("Device %s opened.\n", devname);

	if (no_query) {
		/* Assume capture device. */
		dev->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		return 0;
	}

	memset(&cap, 0, sizeof cap);
	ret = ioctl(dev->fd, VIDIOC_QUERYCAP, &cap);
	if (ret < 0)
		return 0;

	capabilities = cap.capabilities & V4L2_CAP_DEVICE_CAPS
		     ? cap.device_caps : cap.capabilities;

	if (capabilities & V4L2_CAP_VIDEO_CAPTURE)
		dev->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	else if (capabilities & V4L2_CAP_VIDEO_OUTPUT)
		dev->type = V4L2_BUF_TYPE_VIDEO_OUTPUT;
	else {
		printf("Error opening device %s: neither video capture "
			"nor video output supported.\n", devname);
		close(dev->fd);
		return -EINVAL;
	}

	printf("Device `%s' on `%s' is a video %s device.\n",
		cap.card, cap.bus_info,
		dev->type == V4L2_BUF_TYPE_VIDEO_CAPTURE ? "capture" : "output");
	return 0;
}

int main(int argc, char **argv)
{
	// ROS Initalization
	ros::init(argc, argv, "control");
	
	ros::NodeHandle n;
	ros::NodeHandle private_n("~");
	
	// ROS Parameters
	string camera_name;
	private_n.param<std::string>("camera_name", camera_name, "/dev/video7");

	// ROS Publishers
	ros::Publisher loc_pub = n.advertise<vision_based_localization::VisionBasedLocalizationMsgs>("localization", 10);

	// Video Setup
	struct device dev;

	unsigned int width = 640;
	unsigned int height = 480;
	unsigned int pixelformat = V4L2_PIX_FMT_YUYV;
	unsigned int stride = 0;

	unsigned int nbufs = 8;
	unsigned int userptr_offset = 0;
	enum buffer_fill_mode fill_mode = BUFFER_FILL_NONE;

	if (video_open(&dev, camera_name.c_str(), 0) < 0)
		return 1;

	/* Set the video format. */
	if (video_set_format(&dev, width, height, pixelformat, stride) < 0) {
			video_close(&dev);
			return 1;
	}

	if (video_prepare_capture(&dev, nbufs, userptr_offset, fill_mode)) {
		video_close(&dev);
		return 1;
	}

	if (video_enable(dev, 1)) {
		video_close(&dev);
		return 1;
	}

	struct v4l2_buffer buf;

	time_t ocv_start, ocv_end;
	time(&ocv_start);
	int ocv_counter = 0;

	Mat edges;

	while(ros::ok()) {
		/* Dequeue a buffer. */
		memset(&buf, 0, sizeof buf);
		buf.type = dev->type;
		buf.memory = dev->memtype;
		if (ioctl(dev->fd, VIDIOC_DQBUF, &buf) < 0) {
			if (errno != EIO) {
				printf("Unable to dequeue buffer: %s (%d).\n", strerror(errno), errno);
				break;
			}
			buf.type = dev->type;
			buf.memory = dev->memtype;
			if (dev->memtype == V4L2_MEMORY_USERPTR)
				buf.m.userptr = (unsigned long)dev->buffers[i].mem;
		}

		if (dev->type == V4L2_BUF_TYPE_VIDEO_CAPTURE &&
		    dev->imagesize != 0	&& buf.bytesused != dev->imagesize)
			printf("Warning: bytes used %u != image size %u\n",
			       buf.bytesused, dev->imagesize);

		if (dev->type == V4L2_BUF_TYPE_VIDEO_CAPTURE)
			video_verify_buffer(dev, buf.index);

		// Bring image buffer into openCV
		
		Mat YUYV_mat(480, 640, CV_8UC2, dev->buffers[buf.index].mem);
		
		cvtColor(YUYV_mat, edges, 124); // Extract Grayscale image

		GaussianBlur( edges, edges, Size( blur_level, blur_level ), 0, 0 );
        threshold(edges, edges, thresh, 255, THRESH_BINARY_INV);
        
        // Find contours in image
        vector<vector<Point> > contours;
  		vector<Vec4i> hierarchy;
  		
        findContours(edges.clone(), contours, hierarchy, RETR_TREE, CHAIN_APPROX_NONE, Point(0,0));
        
        // Filter Contours

        // remove insignificant contours
        Point vertices[4];
        bool found = false;
        for (int i = 0; i < contours.size(); i++)
        {
        	// if contour has no parent (is an outer contour) and has a child with a child 
        	// and area > 49.0
        	if (hierarchy[i][3] < 0 && hierarchy[i][2] > -1 && hierarchy[hierarchy[i][2]][2] > -1 
        		&& contourArea(contours[i]) > 49.0)
        	{
			
        		vector<Point> inner = contours[hierarchy[hierarchy[i][2]][2]];

  				vector<double> norm;
		    	vector<double> norm2;
		    	for (int m = 0; m < contours[i].size(); m++)
		    	{
					Point p = contours[i][m];        	
		    		norm.push_back(sqrt(p.x*p.x + p.y*p.y));
		    		p.x -= 640;
		    		norm2.push_back(sqrt(p.x*p.x + p.y*p.y));
		    	}
		    	double min = 999999;
		    	double max = -1;
		    	int min_index = -1;
		    	int max_index = -1;
		    	double min2 = 999999;
		    	double max2 = -1;
		    	int min_index2 = -1;
		    	int max_index2 = -1;
		    	for (int m = 0; m < contours[i].size(); m++)
		    	{
		    		if (norm[m] < min)
		    		{
		    			min = norm[m];
		    			min_index = m;
		    		}
		    		if (norm[m] > max)
		    		{
		    			max = norm[m];
		    			max_index = m;
		    		}
		    		if (norm2[m] < min2)
		    		{
		    			min2 = norm2[m];
		    			min_index2 = m;
		    		}
		    		if (norm2[m] > max2)
		    		{
		    			max2 = norm2[m];
		    			max_index2 = m;
		    		}
		    	} 
				vertices[3] = contours[i][min_index];
				vertices[0] = contours[i][min_index2];
				vertices[1] = contours[i][max_index];
				vertices[2] = contours[i][max_index2];

				bool same = false;

				for (int i = 0; i < 4; i++)
				{
					for (int j = i+1; j < 4; j++) 
					{
						if ( vertices[i].x == vertices[j].x && vertices[i].y == vertices[j].y )
							same = true;
					}
				}

				if (same)
					break;

				//for (int j = 0; j < 4; j++)
    			//	line(img, vertices[j], vertices[(j+1)%4], Scalar(0,0,255), 2);
    				
    			MatrixXd ic(3,4);

		    	for (int i = 0; i < 4; i++)
		    	{
		    		ic(0,i) = (float)vertices[i].x;
		    		ic(1,i) = (float)vertices[i].y;
		    		ic(2,i) = 1.0;
		    	}
		    	
		    	MatrixXd calib(3,3);
		    	calib <<  518.81428608, 	0, 				298.18411412,
		    			0,				517.07586175, 	244.68941109,
		    			0,				0,				1;
		    	
		    	MatrixXd coordinatesSI = calib.inverse()*ic;
		    	
		    	Point2f p = Point2f((coordinatesSI(0,0) + coordinatesSI(0,3))/2.0,(coordinatesSI(1,0) + coordinatesSI(1,3))/2.0);
		    	Point2f q = Point2f((coordinatesSI(0,1) + coordinatesSI(0,2))/2.0,(coordinatesSI(1,1) + coordinatesSI(1,2))/2.0);
		    	

		    	double f = 1.0;
		    	
		    	double dv = 132.0;
		    	
		    	double psi = -atan(p.x/f);
		    	
		    	double z = f*dv/(q.y - p.y);
		    	
		    	double rho = z/cos(psi);
		    	
		    	double zA = f*dv/(coordinatesSI(1,1) - coordinatesSI(1,0));
		    	
		    	double zD = f*dv/(coordinatesSI(1,2) - coordinatesSI(1,3));
		    	
		    	
		    	Vector3d Abar, Bbar, Cbar, Dbar, AD, AB, DC, BC;
		    	
		    	for (int i = 0; i < 3; i++)
		    	{
		    		Abar[i] = zA*coordinatesSI(i,2)/f;
		    		Bbar[i] = zA*coordinatesSI(i,3)/f;
		    		
		    		Cbar[i] = zD*coordinatesSI(i,0)/f;
		    		Dbar[i] = zD*coordinatesSI(i,1)/f;
		    		AD[i] = Dbar[i] - Abar[i];
		    		AB[i] = Bbar[i] - Abar[i];
		    		DC[i] = Cbar[i] - Dbar[i];
		    		BC[i] = Cbar[i] - Bbar[i];
		    		
		    	}
		    	
		    	Abar[2] = zA/f;
		    	Bbar[2] = zA/f;
		    	Dbar[2] = zD/f;
		    	Cbar[2] = zD/f;
		    	
		    	AD[2] = Dbar[2] - Abar[2];
				AB[2] = Bbar[2] - Abar[2];
				DC[2] = Cbar[2] - Dbar[2];
				BC[2] = Cbar[2] - Bbar[2];
			

				if (AD.norm() != 0.0) {
					AD.normalize();
				}

				if (AB.norm() != 0.0) {
					AB.normalize();
				}

				if (DC.norm() != 0.0) {
					DC.normalize();
				}

				if (BC.norm() != 0.0) {
					BC.normalize();
				}
			
				Vector3d n = (DC.cross(AD) + AB.cross(BC))/2.0;

				double gamma;
	
				if (AD[2] < 0.0)
				{
					gamma = atan2(sqrt(1-pow(n[2],2)), n[2]);
				} 
				else 
				{
					gamma = atan2(-sqrt(1-pow(n[2],2)), n[2]);
				}

				if ( gamma !=  gamma ) {
					ROS_INFO("n[2] = %f", n[2]);
				}
				//cout << "----new-----" << endl;
				//cout << "rho: " << rho << endl;
				//cout << "psi: " << psi*180/3.141592 << endl;
				//cout << "gamma: " << gamma << endl;
				
				vision_based_localization::VisionBasedLocalizationMsgs msg; 
				msg.rho = rho;
				msg.psi = psi;
				msg.gamma = gamma;
				loc_pub.publish(msg);
				found = true;
				break;
			} 
        
	        	
	    	if ( !found ) {
				vision_based_localization::VisionBasedLocalizationMsgs msg; 
				msg.rho = 0.0;
				msg.psi = 0.0;
				msg.gamma = 0.0;
				loc_pub.publish(msg);
			}	
	    	

			time(&ocv_end);
        	ocv_counter++;
			//std::cout << "fps: " << ocv_counter/(double)difftime(ocv_end,ocv_start) << std::endl;
		}

		/* Requeue the buffer. */
		if (video_queue_buffer(dev, buf.index, fill) < 0) {
			printf("Unable to requeue buffer: %s (%d).\n", strerror(errno), errno);
			break;
		}
	}
	video_free_buffers(dev);
	video_enable(dev, 0);
	video_close(&dev);
	return 0;
}
