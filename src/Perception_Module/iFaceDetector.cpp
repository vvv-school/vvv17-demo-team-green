//============================================================================
// Name        : FaceDetect.cpp
// Author      : Chen Tai Pang, Lawrence
// Version     :
// Copyright   : Institute for Infocomm Research, Robotics Department
// Description : Hello World in C++, Ansi-style
//============================================================================

/*
Institute for Inforcomm Research
By Chen Tai Pang, Lawrence
FaceDetector.cpp
Function:
 Normalize images
 Detect faces
 Detect Eyes
*/

#include "stdafx.h" // Comment this line for Linux version
#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <vector>

using namespace std;

#include "OpenCVInc.h"
using namespace cv;

#include "iFaceDetector.h"

//float H_Scale=1.0f;

void MessageBox(void *hdl,char *Msg,char* Caption,int Button)
{
    printf("%s: %s\n",Caption,Msg);    
}

iFaceDetector::iFaceDetector()
{
	ClassifiedLoaded = 0;

	MinFaceSize.height = 44;
	MinFaceSize.width = 44;
	MaxFaceSize.height = -1;
	MaxFaceSize.width = -1;
	
	#ifdef USE_GPU_DETECTOR
	gpu::CascadeClassifier_GPU cascade_gpu,lbpCascade1_gpu;
	using namespace cv::gpu;
	#endif

	RegionOfInterest = NULL;
}


iFaceDetector::~iFaceDetector()
{
	if(RegionOfInterest!=NULL)
		delete RegionOfInterest;
}

void iFaceDetector::SetRegionOfInterest(int rx,int ry,int Width,int Height)
{
	if(RegionOfInterest!=NULL)
		delete RegionOfInterest;
	
	RegionOfInterest = new Rect;
	RegionOfInterest->x = rx;
	RegionOfInterest->y = ry;
	RegionOfInterest->width = Width;
	RegionOfInterest->height = Height;
}

void iFaceDetector::SetMinFaceSizeForHaar(int FaceWidth)
{
	if(FaceWidth<22)
		FaceWidth=22; // At least 22 pixels width. 
	MinFaceSize.height = FaceWidth;
	MinFaceSize.width = FaceWidth;
}

void iFaceDetector::SetMaxFaceSize(int FaceWidth)
{
	if(ImgHeight>32 && ImgWidth>32)
	{
		if(FaceWidth>22 && FaceWidth<ImgHeight && FaceWidth<ImgWidth)
		{
			MaxFaceSize.height = FaceWidth;
			MaxFaceSize.width = FaceWidth;
		}
		else
		{
			MaxFaceSize.height = -1;
			MaxFaceSize.width = -1;
		}
	}
}

void iFaceDetector::SetRuntimeImageDimension(int Height,int Width)
{
	ImgHeight = Height;
	ImgWidth = Width;		
    smallImg.create(cvRound ((float)Height/ImgScale),cvRound((float)Width/ImgScale), CV_8UC1 );
}

int iFaceDetector::InitializeFaceDetector_LBP_HAAR_CPU(float scale,int Height,int Width,char *HAARClassifierName,char *LBPClassifierName,char *LeftEyeClassifierName,char *RightEyeClassifierName)
{
	ImgScale = scale;
	SetRuntimeImageDimension(Height,Width);
	
	string cascadeName = HAARClassifierName;//ClassifierName;
	if(ClassifiedLoaded == 0) // Only load the classifier once. 
	{		
		if( !cascade.load( cascadeName ) )
		{			
			char Caption[] = "ERROR: Could not load classifier cascade\n";
			char Text[256];
			sprintf(Text,"Cannot open Classifier:%s\nPlease check the path.",cascadeName.c_str());
			MessageBox(NULL,Text,Caption,MB_OK);
			exit(-1);
			//return -1;
		}	
		string cascadeNameLBP = LBPClassifierName;
		if( !lbpCascade1.load( cascadeNameLBP ) )
		{			
			char Caption[] = "ERROR: Could not load classifier cascade\n";
			char Text[256];
			sprintf(Text,"Cannot open Classifier:%s\nPlease check the path.",cascadeName.c_str());
			MessageBox(NULL,Text,Caption,MB_OK);
			exit(-1);
			//return -1;
		}	
		ClassifiedLoaded = 1;		
	}
/*
	if( leftEyeCascade.empty()){
		if( !leftEyeCascade.load((string)LeftEyeClassifierName) )	{
			printf("ERROR: Could not load left eye cascade" );		
			exit(-1);	}
	}
	if( rightEyeCascade.empty())	{
		if( !rightEyeCascade.load((string)RightEyeClassifierName) )	{
			printf("ERROR: Could not load right eye cascade" );		
			exit(-1);}
	}
 */
	return 0;
}

#ifdef USE_GPU_DETECTOR
int InitializeFaceDetector_LBP_HAAR_GPUCV(float scale,int Height,int Width,char *HAARClassifierName,char *LBPClassifierName)
{
	ImgScale = scale;

	if (getCudaEnabledDeviceCount() == 0)
    {
        MessageBox(NULL,"No GPU found or the library is compiled without GPU support","Init. GPU",MB_OK);
		return -1;
    }

	SetRuntimeImageDimension(Height,Width);
	//smallImg44( 44, 44, CV_8UC1);
	string cascadeName = HAARClassifierName;//ClassifierName;
	if(ClassifiedLoaded == 0) // Only load the classifier once. 
	{			
		if( !cascade_gpu.load( cascadeName ) )
		{			
			char Caption[] = "ERROR: Could not load classifier cascade\n";
			char Text[256];
			sprintf(Text,"Cannot open Classifier:%s\nPlease check the path.",cascadeName);
			MessageBox(NULL,Text,Caption,MB_OK);
			return -1;
		}	
		string cascadeNameLBP = LBPClassifierName;
		if( !lbpCascade1.load( cascadeNameLBP ) )
		{			
			char Caption[] = "ERROR: Could not load classifier cascade\n";
			char Text[256];
			sprintf(Text,"Cannot open Classifier:%s\nPlease check the path.",cascadeName);
			MessageBox(NULL,Text,Caption,MB_OK);
			return -1;
		}
		ClassifiedLoaded = 1;	
	}
	return 0;
}
#endif

int iFaceDetector::InitializeFaceDetector_LBP_HAAR(float scale,int Height,int Width,char *HAARClassifierName,char *LBPClassifierName,char *LeftEyeClassifierName,char *RightEyeClassifierName)
{
#ifdef USE_GPU_DETECTOR
	return InitializeFaceDetector_LBP_HAAR_GPUCV(scale,Height,Width,HAARClassifierName,LBPClassifierName);
#else
	return InitializeFaceDetector_LBP_HAAR_CPU(scale,Height,Width,HAARClassifierName,LBPClassifierName,LeftEyeClassifierName,RightEyeClassifierName);
#endif 
}

// 24/4/2013 Use this version
void iFaceDetector::Preprocessing(cv::Mat &InputImg)
{
	cvtColor( InputImg, gray, CV_BGR2GRAY ); // Convert the color image to Gray image.	
	if(RegionOfInterest!=NULL)
	{
		// ROI in use. 
		gray = gray(*RegionOfInterest);
		if(gray.rows!=ImgHeight || gray.cols!=ImgWidth)
		{
			// In case the camera setting is not the same as initialization. 
			ImgHeight = RegionOfInterest->height;//InputImg.rows;
			ImgWidth = RegionOfInterest->width;//InputImg.cols; 		
			smallImg.create(cvRound ((float)ImgHeight/ImgScale),cvRound((float)ImgWidth/ImgScale),CV_8UC1 );
		}
	}
	else
	{
		// ROI not in use, use the whole image.
		if(gray.rows!=ImgHeight || gray.cols!=ImgWidth)
		{
			// In case the camera setting is not the same as initialization. 
			ImgHeight = InputImg.rows;
			ImgWidth = InputImg.cols;			
			smallImg.create(cvRound ((float)ImgHeight/ImgScale),cvRound((float)ImgWidth/ImgScale),CV_8UC1 );
		}
	}
	// We may need to add a simple low-pass filter to avoid aliasing at here. 
	if(ImgScale==1.0f)
		smallImg = gray.clone();
	else
		resize( gray, smallImg, smallImg.size(), 0, 0, INTER_LINEAR );

/*	if(H_Scale>1.0f)
	{
		Mat StretchedImage;	
		StretchedImage.create(cvRound ((float)ImgHeight*H_Scale),ImgWidth,CV_8UC1 );
		resize(smallImg,StretchedImage,StretchedImage.size(), 0, 0, INTER_LINEAR );
		smallImg = StretchedImage.clone();
	}
*/

	equalizeHist( smallImg, smallImg );
}

Mat* iFaceDetector::GetgrayImage() // Return the gray scale image with the original dimension. 
{
	return &gray;
}

vector<FaceParameters>* iFaceDetector::FaceDetector_HAAR_LBP_CPU() // Mat& in_img)                                     
{
    double timing = 0;   
	faces.clear();
	faces1.clear();

#ifndef OPENCV2_4
	// Older version OpenCV
	cascade.detectMultiScale( smallImg, faces,1.2, 2, 0,	cascade.origWinSize*2 );//haarCascade.origWinSize * 2
#else
	if(MaxFaceSize.width==-1)
		cascade.detectMultiScale( smallImg, faces,1.2, 2, 0,	MinFaceSize); //cascade.getOriginalWindowSize() *2 );//haarCascade.origWinSize * 2
	else
		cascade.detectMultiScale( smallImg, faces,1.2, 2, 0,	MinFaceSize,MaxFaceSize);

#endif

#ifndef OPENCV2_4
	// Older version OpenCV
	lbpCascade1.detectMultiScale( smallImg, faces1, 1.1, 2, 0, lbpCascade1.origWinSize *2);//lbpCascade.origWinSize * 2
#else
	if(MaxFaceSize.width==-1)
		lbpCascade1.detectMultiScale( smallImg, faces1, 1.1, 2, 0, lbpCascade1.getOriginalWindowSize() *2);//lbpCascade.origWinSize * 2
	else
		lbpCascade1.detectMultiScale( smallImg, faces1, 1.1, 2, 0, lbpCascade1.getOriginalWindowSize() *2,MaxFaceSize);
#endif

	faces.insert(faces.end(), faces1.begin(), faces1.end() ); 
	faces1.clear();

	rweights.clear();
	groupRectangles(faces, rweights,1);

	DetectedFaces.clear();
	Rect *r1;	
	int NumDetectedFace = 0;
	FaceParameters faceParam;
	for(vector<Rect>::const_iterator r = faces.begin(); r != faces.end(); r++ )
	{
		       // Convert back to original scale.
			   r1 = &faceParam.FaceRegion;
			   r1->x = cvRound(r->x*ImgScale);
			   //if(H_Scale==1.0f)
			   r1->y = cvRound(r->y*ImgScale);
			   //else
			   //	r1->y = cvRound((r->y*ImgScale)/H_Scale);   
			   r1->width = cvRound(r->width*ImgScale);
			   r1->height = cvRound(r->height*ImgScale); 
			   if(RegionOfInterest!=NULL)
			   {
				   // For ROI, check the whether the object is out of boundary!
				   if((r1->x+r1->width>RegionOfInterest->width))
					   continue;
				   if((r1->y+r1->height>RegionOfInterest->height))
					   continue;
			   }
			   smallImgROI = gray(*r1); // Extracted the face image.
			   if(RegionOfInterest!=NULL)
			   {
				   r1->x += RegionOfInterest->x;
				   r1->y += RegionOfInterest->y;
			   }	
                           /*
			   if(locateLeftEye())
			   {
					faceParam.LeftEyeRegion = LeftEye;
					if(H_Scale>1.0f)
					{
						faceParam.LeftEyeRegion.y /= H_Scale;
						faceParam.LeftEyeRegion.height /= H_Scale;
                                        }
			   }
			   else
			   {
					faceParam.LeftEyeRegion.x = -1;
					faceParam.LeftEyeRegion.y = -1;
					faceParam.LeftEyeRegion.width = -1;
					faceParam.LeftEyeRegion.height = -1;
			   }
                           */
                           /*
			   if(locateRightEye())
			   {
					faceParam.RightEyeRegion = RightEye;
					//*if(H_Scale>1.0f)
					//{
					//	faceParam.RightEyeRegion.y /= H_Scale;
					//	faceParam.RightEyeRegion.height /= H_Scale;
					//}
			   }
			   else
			   {
					faceParam.RightEyeRegion.x = -1;
					faceParam.RightEyeRegion.y = -1;
					faceParam.RightEyeRegion.width = -1;
					faceParam.RightEyeRegion.height = -1;
			   }
                           */
			   DetectedFaces.push_back(faceParam);
			   NumDetectedFace++;
	}
	
	return &DetectedFaces; //faces;
}

/*
bool iFaceDetector::locateLeftEye()
{
	vector<Rect> objects, tmpEyes;

	Mat data ;

	if (leftEyeCascade.empty() )
	{
		printf("Null Classifier error! exit!\n");
		exit(-1);
	}

	data = smallImgROI; //origFaceData ;

	objectDetection( data, leftEyeCascade, objects);

	int halfFaceX = cvRound( data.cols * 0.5 ) ;
	int halfFaceY = cvRound( data.rows * 0.5 / 1.2 ) ;

	for ( vector<Rect>::const_iterator w = objects.begin(); w != objects.end(); w++ )
	{
		if (   cvRound( w->y + w->height*0.5) < halfFaceY  && cvRound( w->x + w->width*0.5) > halfFaceX  )
				tmpEyes.push_back( cv::Rect( w->x , w->y , w->width, w->height )  );

	}

	std::vector<int> rweights;
	groupRectangles(tmpEyes, rweights,0,1);

	if (tmpEyes.empty())
		return false;

	LeftEye=tmpEyes.at(0);

	return true ;

}

bool iFaceDetector::locateRightEye()
{
	vector<Rect> objects, tmpEyes;
	Mat data ;

	if (rightEyeCascade.empty() )
		return false;
	data = smallImgROI; //origFaceData ;

	objectDetection( data, rightEyeCascade, objects);

	int halfFaceX = cvRound( data.cols * 0.5 ) ;
	int halfFaceY = cvRound( data.rows * 0.5 / 1.2 ) ;

	for ( vector<Rect>::const_iterator w = objects.begin(); w != objects.end(); w++ )	{
		if (   cvRound( w->y + w->height*0.5) < halfFaceY && cvRound( w->x + w->width*0.5) < halfFaceX  )
				tmpEyes.push_back( cv::Rect( w->x , w->y , w->width, w->height )  );
	}

	std::vector<int> rweights;
	groupRectangles(tmpEyes, rweights,0,1);

	if (tmpEyes.empty())
		return false;
	RightEye=tmpEyes.at(0) ;
	//isRightEyeDetected = true ;

	return true ;

}
 */

vector<FaceParameters>* iFaceDetector::DetectFaceParameters(Mat &ImageFrame)
{
	Preprocessing(ImageFrame);
	return FaceDetector_HAAR_LBP();
}

void iFaceDetector::objectDetection(Mat& img, CascadeClassifier& cascadeDetector, vector<Rect>& objects)
{
	if (cascadeDetector.empty() )
		return ;

	Mat tmpGray;

	if (img.channels()>1)
		cvtColor( img, tmpGray, CV_BGR2GRAY );
	else
		img.copyTo( tmpGray );

#ifdef OPENCV2_4
	cascadeDetector.detectMultiScale( tmpGray, objects,
		1.1, 1, 0
		//|CV_HAAR_FIND_BIGGEST_OBJECT
		//|CV_HAAR_DO_ROUGH_SEARCH
		//|CV_HAAR_SCALE_IMAGE
		,
		//Size(52, 52) );
		cascadeDetector.getOriginalWindowSize()); // origWinSize);  //getOriginalWindowSize() );
#else
	cascadeDetector.detectMultiScale( tmpGray, objects,
		1.1, 1, 0
		//|CV_HAAR_FIND_BIGGEST_OBJECT
		//|CV_HAAR_DO_ROUGH_SEARCH
		//|CV_HAAR_SCALE_IMAGE
		,
		//Size(52, 52) );
		cascadeDetector.origWinSize);  //getOriginalWindowSize() );

#endif
	return ;
}


#ifdef USE_GPU_DETECTOR
cv::vector<cv::Rect>* FaceDetector_HAAR_LBP_GPU() // Mat& in_img)                                     
{
    int i,j;
    double timing = 0;   
	faces.clear();

	Mat faces_cpu;
	gpu::GpuMat FrameGPU,facesBuf_gpu;

#ifndef OPENCV2_4
	cascade.detectMultiScale( smallImg, faces,1.2, 2, 0,	cascade.origWinSize*2 );//haarCascade.origWinSize * 2
#else
	FrameGPU.upload(smallImg);
	int detections_num = cascade_gpu.detectMultiScale(FrameGPU, facesBuf_gpu, 1.2,2);
    facesBuf_gpu.colRange(0, detections_num).download(faces_cpu);
	Rect* faces_local = faces_cpu.ptr<Rect>();
	for(int i = 0; i < detections_num; ++i)
		faces[i] = faces_local[i];
	//cascade.detectMultiScale( smallImg, faces,1.2, 2, 0,	cascade.getOriginalWindowSize() *2 );//haarCascade.origWinSize * 2
#endif

#ifndef OPENCV2_4
	lbpCascade1.detectMultiScale( smallImg, faces1, 1.1, 2, 0, lbpCascade1.origWinSize *2);//lbpCascade.origWinSize * 2
#else
	/*
	detections_num = lbpCascade1_gpu.detectMultiScale(FrameGPU, facesBuf_gpu, 1.1,4);
	facesBuf_gpu.colRange(0, detections_num).download(faces_cpu);
	faces_local = faces_cpu.ptr<Rect>();
	for(int i = 0; i < detections_num; ++i)
		faces1[i] = faces_local[i];
	*/
	lbpCascade1.detectMultiScale( smallImg, faces1, 1.1, 2, 0, lbpCascade1.getOriginalWindowSize() *2);//lbpCascade.origWinSize * 2
#endif

	faces.insert(faces.end(), faces1.begin(), faces1.end() ); 
	faces1.clear();

	//rweights.clear();
	groupRectangles(faces, rweights,1);
    //timing = (double)cvGetTickCount();
    //timing = (double)cvGetTickCount() - timing ;	
	// Sort rectangles in descending order (big -> small)	
	return &faces;
}
#endif

std::vector<FaceParameters>* iFaceDetector::FaceDetector_HAAR_LBP()
{
#ifdef USE_GPU_DETECTOR
		return FaceDetector_HAAR_LBP_GPU();
#else
	    return FaceDetector_HAAR_LBP_CPU();
#endif
}

float iFaceDetector::GetImgScale()
{
	return ImgScale;
}

