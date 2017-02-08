/*
 * Copyright (C) 2016 iCub Facility - Istituto Italiano di Tecnologia
 * Author: Vadim Tikhanoff
 * email:  vadim.tikhanoff@iit.it
 * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU General Public License, version 2 or any
 * later version published by the Free Software Foundation.
 *
 * A copy of the license can be found at
 * http://www.robotcub.org/icub/license/gpl.txt
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details
 */

#include <yarp/os/BufferedPort.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/Network.h>
#include <yarp/os/Log.h>
#include <yarp/os/Time.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/Semaphore.h>
#include <yarp/sig/Image.h>
#include <yarp/os/RpcClient.h>

// For the call back function. 
#include <yarp/os/Thread.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/Semaphore.h>

//#define DEBUG_LOCAL

using namespace yarp::os;

#include <vector>

using namespace std;

#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>

using namespace cv;

#include "OpenCVInc.h"
#include "iFaceDetector.h"

#include "closestBlob_IDL.h"

char Haar_ClassifierFile[] = "./../Data/haarcascade_frontalface_alt.xml"; 
//char Haar_ClassifierFile[] = "/home/lawrence/vvv17/assignment_closest-blob-LawrenceChen-i2r/Data/haarcascade_frontalface_alt.xml";

char LBP_ClassifierFile[] = "./../Data/lbpcascade_frontalface.xml";  
//char LBP_ClassifierFile[] = "/home/lawrence/vvv17/assignment_closest-blob-LawrenceChen-i2r/Data/lbpcascade_frontalface.xml";

#define FaceEventID 1
#define ObjectEventID 2

//yarp write ... /detection/detector_command:i
//yarp read ... /detection/detected_face_obj:o

class Callback:public BufferedPort<Bottle>
{
private:
	Semaphore mutex;
	Bottle Datum;
        Port inPort;
        
        bool ReceivedGetFaceCommand;
        bool ReceivedGetObjectCommand;
        bool ReceivedGetCroppedImageCommand;
        IplImage crop_object2sm; 
                
public:
           
	Callback()
	{
            ReceivedGetFaceCommand = false;
            ReceivedGetObjectCommand = false;
            ReceivedGetCroppedImageCommand = false;
            Datum.clear();
	}

  	void onRead(Bottle &v)
	{
            mutex.wait(); 
            Datum=v;   
            
            if(Datum.toString()=="GetFace")
                ReceivedGetFaceCommand = true;  
            if(Datum.toString()=="GetObject")
                ReceivedGetObjectCommand = true;      
            if(Datum.toString()=="GetCropImg")
                ReceivedGetCroppedImageCommand = true;                        
            
            Datum.clear();
            //Time::delay(5);
            mutex.post();
            //printf("Data received: num. data = %i ",Datum.size());
            //for(int i=0;i<Datum.size();i++)
            //{
            //    printf("%f, ",Datum.get(i).asDouble());
            //}
            //fprintf(stderr, "Callback got: %s\n",Datum.toString().c_str());
        }

	void lock()
	{
		mutex.wait();
	}

    void unlock()
    {
        mutex.post();
    }

    Bottle get()
    {
        return Datum;
    }
    bool IsGetFaceCommand()
    {
        bool result = ReceivedGetFaceCommand;
        ReceivedGetFaceCommand = false; // reset the flag. 
        return result;
    }
    
    bool IsGetObjectCommand()
    {
        bool result = ReceivedGetObjectCommand;
        ReceivedGetObjectCommand = false; // reset the flag. 
        return result;
    }
    
    //ReceivedGetCroppedImageCommand
    bool IsGetCroppedImageCommand()
    {
        bool result = ReceivedGetCroppedImageCommand;
        ReceivedGetCroppedImageCommand = false; // reset the flag. 
        return result;
    }    
    
};

Callback MyCallBack;



/********************************************************/
class Processing : public yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelMono> >
{
    std::string moduleName;

    yarp::os::RpcServer handlerPort;

    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb> >   inPort;
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb> >   outPort;
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb> >   cropOutPort;
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb> >   cropOutPort2sm;
    yarp::os::BufferedPort<yarp::os::Bottle>  targetPort;
    yarp::os::BufferedPort<yarp::os::Bottle>  detectedFacePort;
    yarp::os::BufferedPort<yarp::os::Bottle>  detector2gazePort;
    

    yarp::os::RpcClient rpc;
    
    iFaceDetector iFD;
    int FaceDropCounter;
    cv::Rect DetectedFace;
    bool FaceDetectorInitialized; 
    cv::Point FaceCenter;
    cv::Point ObjectCenter;
    
    cv::Mat SavedCropImage;
    bool savedImageValid = false;

    
    void InitFaceDetector(int Height,int Width)
    {
        iFD.InitializeFaceDetector_LBP_HAAR(1.0f,Height,Width,Haar_ClassifierFile,LBP_ClassifierFile,"DummyLeft","DummyRight");
	iFD.SetMinFaceSizeForHaar(22);
        iFD.SetMaxFaceSize(200);   
        DetectedFace.x = -1; // No Face Detected.    
        FaceDropCounter = -1; 
        FaceDetectorInitialized = true;
    }

public:
    /********************************************************/
        

    Processing( const std::string &moduleName )
    {
        FaceDetectorInitialized = false;
        FaceCenter.x = -1;
        FaceCenter.y = -1;
        ObjectCenter.x = -1;
        ObjectCenter.y = -1;
        savedImageValid = false;
        this->moduleName = moduleName;
    }

    /********************************************************/
    ~Processing()
    {

    };

    /********************************************************/
    bool open(){

        this->useCallback();

        BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelMono> >::open( "/" + moduleName + "/disparity:i" );
        inPort.open("/"+ moduleName + "/image:i");
        outPort.open("/"+ moduleName + "/image:o");
        cropOutPort.open("/" + moduleName + "/crop:o");
        cropOutPort2sm.open("/"+ moduleName +"/crop_obj:o");
        targetPort.open("/"+ moduleName + "/target:o");
        detectedFacePort.open("/"+ moduleName + "/detected_face_obj:o");             
        detector2gazePort.open("/"+ moduleName + "/detector2gaze:o");     

        return true;
    }

    /********************************************************/
    void close()
    {
        inPort.close();
        outPort.close();
        targetPort.close();
        cropOutPort.close();
        BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelMono> >::close();
    }

    /********************************************************/
    void interrupt()
    {
        BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelMono> >::interrupt();
    }
            
    int PerformFaceDetection(cv::Mat &in_image)
    {
        if(!FaceDetectorInitialized)
        {
            //yError()<<"Error: OpenCV Face detected is not initialized!";
            return -1;
        }
        
	std::vector<FaceParameters>* DetectedFaces = iFD.DetectFaceParameters(in_image);
	int ObjCounter=0;
        int LargestSize = 0,LargestSideIndex = -1;
        bool iFaceDetected = false;
	for(std::vector<FaceParameters>::const_iterator r = DetectedFaces->begin(); r != DetectedFaces->end(); r++ )
	{
            // Get the face with maximum size. 
            if(r->FaceRegion.width>LargestSize)
            {
                LargestSize = r->FaceRegion.width;
                LargestSideIndex = ObjCounter;
                DetectedFace = r->FaceRegion;
                FaceDropCounter = 0;
                iFaceDetected  = true;
            }               
			// Circle the face and eyes (if detected) on screen.
			// Note: the position of eyes are relative to the position of the face.
		//DrawOnObject(frame1,(Rect*)&r->FaceRegion,(Rect*)&r->LeftEyeRegion,(Rect*)&r->RightEyeRegion);
		//printf("%d:Face at (%d,%d), Sq. Width = %d\n",ObjCounter,r->FaceRegion.x,r->FaceRegion.y,r->FaceRegion.width);
            ObjCounter++;
	}  
        
        if(!iFaceDetected)
        {
            if(FaceDropCounter>=0)
            {
                FaceDropCounter++;
                if(FaceDropCounter>5)
                {
                    // Face not detected in the previous 5 frames, 
                    // person may be gone, remove the detected face object. 
                    FaceDropCounter = -1;
                    DetectedFace.x = -1; 
                }
            }
        }
        
        /*
        yarp::os::Bottle &outFace = detectedFacePort.prepare();                                   
        MyCallBack.lock();
        bool FaceCommandReceived = MyCallBack.IsGetFaceCommand();
        MyCallBack.unlock();
        if(FaceCommandReceived)
        {
            // Send out the bottle of detected face.
            yarp::os::Bottle &face_b = outFace.addList();
            face_b.clear();            
            if(LargestSideIndex!=-1)
            {
                int FaceCenterX = (DetectedFace.x+DetectedFace.width/2);
                int FaceCenterY = (DetectedFace.y+DetectedFace.height/2);                
                face_b.addInt(FaceCenterX);
                face_b.addInt(FaceCenterY);
                face_b.addInt(DetectedFace.width);
                //detectedFacePort
                if(face_b.size()>0)
                   detectedFacePort.write();          
            }
            else
            {                
                face_b.addInt(-1);
                face_b.addInt(-1);
                face_b.addInt(-1);
                //detectedFacePort
                if(face_b.size()>0)
                   detectedFacePort.write();                                      
            }

        }          
        */
                      
        return LargestSideIndex;
    }

    /********************************************************/
    void onRead( yarp::sig::ImageOf<yarp::sig::PixelMono> &dispImage )
    {
                       
        yarp::sig::ImageOf<yarp::sig::PixelRgb> &outImage  = outPort.prepare();
        yarp::sig::ImageOf<yarp::sig::PixelRgb> &cropOutImage  = cropOutPort.prepare();        
        yarp::sig::ImageOf<yarp::sig::PixelRgb> &cropOutImage2sm  = cropOutPort2sm.prepare();
        yarp::os::Bottle &outTargets = targetPort.prepare();                                                      
        yarp::sig::ImageOf<yarp::sig::PixelRgb> *inImage = inPort.read();

        outImage.resize(dispImage.width(), dispImage.height());
        cropOutImage.resize(dispImage.width(), dispImage.height());
        
        outImage.zero();
        cropOutImage.zero();
        
        cv::Mat inColour_cv = cv::cvarrToMat((IplImage *)inImage->getIplImage());  // prepare the image ports and targets
        cv::Mat inDisp_cv = cv::cvarrToMat((IplImage *)dispImage.getIplImage());
        
        
        cv::Mat disp = inDisp_cv.clone();
        
        int DetectedFaceObj = -1;
        if(!FaceDetectorInitialized)
        {
            this->InitFaceDetector(inColour_cv.rows,inColour_cv.cols);
        }
        else
            DetectedFaceObj = PerformFaceDetection(inColour_cv);
       

        yarp::os::Bottle &outFace = detectedFacePort.prepare(); 
        outFace.clear();
        MyCallBack.lock();
        bool FaceCommandReceived = MyCallBack.IsGetFaceCommand();
        MyCallBack.unlock();
        
        
        if(DetectedFaceObj!=-1)
        {
            // Face detected. 
            FaceCenter.x = (DetectedFace.x+DetectedFace.width/2);
            FaceCenter.y = (DetectedFace.y+DetectedFace.height/2);
        }
        else
        {
            // Face not detected. 
            FaceCenter.x = -1;
            FaceCenter.y = -1;
        }
                
                
        if(FaceCommandReceived)
        {
            // Send out the bottle of detected face. 
            yarp::os::Bottle &face_b = outFace.addList();            
            if(DetectedFaceObj!=-1)
            {
                face_b.addInt(FaceEventID);
                face_b.addInt(FaceCenter.x);
                face_b.addInt(FaceCenter.y);
                face_b.addInt(DetectedFace.width);
                //this->
                //if(face_b.size()>0)
                //   detectedFacePort.write();          
            }
            else
            {
                face_b.addInt(FaceEventID);                
                face_b.addInt(-1);
                face_b.addInt(-1);
                face_b.addInt(-1);                
                //if(face_b.size()>0)
                  // detectedFacePort.write();                                      
            }
            if(face_b.size()>0)
               detectedFacePort.write();                                      
            
        }    
        
        
        //FILL IN THE CODE
        
        // Apply image processing techniques on the disparity image to smooth things out 
        cv:: Mat pDispMap = inDisp_cv.clone();
        
        int gaussian_size = 5;
        cv::GaussianBlur(pDispMap,pDispMap,cv::Size(gaussian_size,gaussian_size),
                2,2);
    
        // Apply some threshold on the image to remove background:
        // have a look at cv::threshold function
        
        // Find the max value and its position
        cv::Point MinLoc,MaxLoc;
        double minValue,maxValue;
        minMaxLoc( pDispMap, &minValue, &maxValue, &MinLoc, &MaxLoc, cv::Mat() );
        
        int Threshold = cvRound(maxValue)-20; 
        cv::threshold(pDispMap, pDispMap, Threshold , maxValue, 0);
        
        //....

        //Find the contour of the closest objects with moments and mass center
        //
        cv::Mat canny;
        std::vector<std::vector<cv::Point> > contours;
        std::vector<cv::Vec4i> hierarchy;
        
        cv::Canny( pDispMap, canny, Threshold, Threshold*2, 3 );          
        cv::findContours( canny, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0) );                       
        
        // Use Polygon test to check the area and center of mask. 
        int Area[100]={0},MaxValContour[100]={0};
        int MaxObjValue = 0,SelectedObjDistIndex = -1;
        int SelectedObjArea;
        
        cv::Rect ObjRect;
        
        for(int k=0;k<contours.size();k++)
        {
            int MinX=99999,MinY=9999999,MaxX=0,MaxY=0;
            for( int j = 0; j < pDispMap.rows; j++ )
            { for( int i = 0; i < pDispMap.cols; i++ )
              { 
                if(pointPolygonTest( contours[k], cv::Point2f(i,j), false )>0)
                {
                    Area[k] = Area[k] + 1;
                    // Get the max. value within the contour. 
                    if(pDispMap.at<unsigned char>(j,i)>MaxValContour[k])
                        MaxValContour[k] = static_cast<int>(pDispMap.at<unsigned char>(j,i));                        
                    if(i<MinX)
                        MinX = i;
                    if(j<MinY)
                        MinY = j;
                    if(i>MaxX)
                        MaxX = i;
                    if(j>MaxY)
                        MaxY = j;                    
                }
              }
            } 
            
            if(MaxObjValue<MaxValContour[k])
            {
                if(MinX>50 && MinY>50 && MaxX<pDispMap.cols && MaxY<pDispMap.rows)
                {                
                    MaxObjValue = MaxValContour[k];
                    SelectedObjDistIndex = k;
                    ObjRect.x = MinX;
                    ObjRect.y = MinY;
                    ObjRect.width = MaxX-MinX;
                    ObjRect.height = MaxY-MinY;
                    SelectedObjArea = Area[k];
                    yInfo()<<"SelectedObjArea = "<<SelectedObjArea;
                }                
            }
        }
        
        /// Get the moments
        std::vector<cv::Point2f> mc(1);
        if(SelectedObjDistIndex>-1)
        {
          /*if(SelectedObjArea<6500 && ObjRect.width<400 && ObjRect.height<450
               && ObjRect.x>50 && ObjRect.y>50 && 
                  ObjRect.x+ObjRect.width<inDisp_cv.cols-50 && ObjRect.y+ObjRect.height<inDisp_cv.rows-50 &&
                  ObjRect.width>30 && ObjRect.height>30)
           * */
          if( ObjRect.width<220 && ObjRect.height<300 //SelectedObjArea<7500
               && ObjRect.x>50 && ObjRect.y>50  
               //&& ObjRect.x+ObjRect.width<InDisparityImage.cols-50 && ObjRect.y+ObjRect.height<InDisparityImage.rows-50 
               //&& ObjRect.width>30 && ObjRect.height>30
            )            
          {          
          std::vector<cv::Moments> mu(1);
          //for( int i = 0; i < contours.size(); i++ )
           //  { 
              mu[0] = moments( contours[SelectedObjDistIndex], false ); 
           //}

          ///  Get the mass centers:
          
          //for( int i = 0; i < contours.size(); i++ )
          //{ 
              mc[0] = cv::Point2f( mu[0].m10/mu[0].m00 , mu[0].m01/mu[0].m00 );           
          //} 
          }
          else 
              SelectedObjDistIndex = -1;              
        }
        
        // Filer out the large object
       
        //....

        // optional hint: you could use pointPolygonTest and the previous maxvalue location to compare with all contours found and get the actual brightest one

        //....

        // Use the result of pointPolygonTest or your own technique as the closest contour to:
        // 1 - draw it on the disparity image
        // 2 - create a cropped image containing the rgb roi
        // 3 - fill in a yarp bottle with the bounding box

        //be aware that the expected Bottle should be a list containing:
        // (tl.x tl.y br.x br.y)
        //where tl is top left and br - bottom right

        //disp = pDispMap.clone();
        
        cvtColor(disp, disp, CV_GRAY2RGB);
        
        //cv::Mat drawing = Mat::zeros( canny_output.size(), CV_8UC3 );
        cv::RNG rng(12345);
                
        cv::Mat outColour_cv = inColour_cv.clone();
        //outColour_cv = cv::Scalar::all(0);
        
        /*
        int MaxWidth = inColour_cv.cols;
        int MaxHight = inColour_cv.rows;
        if(SelectedObjDistIndex>-1)
        {
            if(ObjRect.x+ObjRect.width> (MaxWidth*3/4)
               || ObjRect.y+ObjRect.height> (MaxHight*3/4) )
            {
               SelectedObjDistIndex = -1;                           
            }
        }
         */
        
        MyCallBack.lock();
        // Clear the command from the queue.         
        if(MyCallBack.IsGetCroppedImageCommand())
        {
            if(savedImageValid)
            {
                cropOutImage2sm.resize(SavedCropImage.cols,SavedCropImage.rows);
                IplImage crop_img_ipl = SavedCropImage;
                cvCopy( &crop_img_ipl, (IplImage *) cropOutImage2sm.getIplImage());
                cropOutPort2sm.write();    
                savedImageValid = false; // Saved, set the flag
            }                        
        }        
        MyCallBack.unlock();
        
        
        outTargets.clear();
        if(SelectedObjDistIndex>-1)
        {
            //for( int i = 0; i< contours.size(); i++ )
            //{
               cv::Scalar color = cv::Scalar( rng.uniform(0, 0), rng.uniform(0,255), rng.uniform(0,0) );
               drawContours( disp, contours, SelectedObjDistIndex, color, 2, 8, hierarchy, 0, cv::Point() );
               cv::circle( disp, mc[0], 4, color, -1, 8, 0 );                   
               cv::rectangle(disp,ObjRect,cv::Scalar(0,0,255),4);  // Debugging 
               //cv::imshow("Debug",disp);
               //cv::waitKey(1);
            //}
            /*               
            ObjRect.width = ObjRect.width-1;
            ObjRect.height = ObjRect.height-1;
            cv::Mat input_roi = inColour_cv(ObjRect);            
            
            input_roi.copyTo(outColour_cv(ObjRect));
            */
            
            
            cv::rectangle(outColour_cv,ObjRect,cv::Scalar(0,255,0),4);  
            
                        
            yarp::os::Bottle &t = outTargets.addList();
            t.addDouble(ObjRect.x);
            t.addDouble(ObjRect.y);
            t.addDouble(ObjRect.x+ObjRect.width);
            t.addDouble(ObjRect.y+ObjRect.height);   
            
            ObjectCenter.x = ObjRect.x + ObjRect.width/2;
            ObjectCenter.y = ObjRect.y + ObjRect.height/2;
            
            MyCallBack.lock();
            if(MyCallBack.IsGetObjectCommand())
            {
                // Sending out detected object rect. 
                yarp::os::Bottle &object_b = outFace.addList();                                          
                int FaceCenterX = (DetectedFace.x+DetectedFace.width/2);
                int FaceCenterY = (DetectedFace.y+DetectedFace.height/2); 
                object_b.addInt(ObjectEventID);
                object_b.addInt(ObjRect.x);                    
                object_b.addInt(ObjRect.y);
                object_b.addInt(ObjRect.width);
                object_b.addInt(ObjRect.height);
                
                if(object_b.size()>0)
                    detectedFacePort.write(); 
                
                // Sending out cropped image
                cv::Mat cropped_img_roi = inColour_cv(ObjRect); 
                SavedCropImage = cropped_img_roi.clone();
                savedImageValid = true;
                /*
                cropOutImage2sm.resize(SavedCropImage.cols,SavedCropImage.rows);
                IplImage crop_img_ipl = SavedCropImage;
                cvCopy( &crop_img_ipl, (IplImage *) cropOutImage2sm.getIplImage());
                cropOutPort2sm.write();    
                */
                /*
                cropOutImage2sm.resize(cropped_img_roi.cols,cropped_img_roi.rows);
                IplImage crop_img_ipl = cropped_img_roi;
                cvCopy( &crop_img_ipl, (IplImage *) cropOutImage2sm.getIplImage());
                cropOutPort2sm.write();    
                */                                            
                //cropOutPort.write();                                    
            }
            MyCallBack.unlock();
            //if (outTargets.size() > 0)
            //    targetPort.write();                        
        }
        else
        {  
            ObjectCenter.x = -1;
            ObjectCenter.y = -1;

            MyCallBack.lock();
            if(MyCallBack.IsGetObjectCommand())
            {
                yarp::os::Bottle &object_b = outFace.addList();            
                object_b.addInt(ObjectEventID);
                object_b.addInt(-1);                    
                object_b.addInt(-1);
                object_b.addInt(-1);
                object_b.addInt(-1);  
                if(object_b.size()>0)
                    detectedFacePort.write();                                                      
            }   
            MyCallBack.unlock();
        }
        
        
        //cropOutImage2sm
        
        // face detected
        if(DetectedFaceObj!=-1)
        {
            cv::rectangle(outColour_cv,DetectedFace,cv::Scalar(64,255,0),4);                        
        }

  
        if(contours.size()>100)
        {
            pDispMap = cv::Scalar::all(0);;
            disp = pDispMap.clone();        
            cvtColor(disp, disp, CV_GRAY2RGB);                        
        }
                        
        
        if (outTargets.size()>0 )
            targetPort.write();  

        yarp::os::Bottle &objects2gaze =  detector2gazePort.prepare();
        objects2gaze.clear();
        yarp::os::Bottle &objects2gaze_b = objects2gaze.addList();
        objects2gaze_b.addString("face");
        objects2gaze_b.addInt(this->FaceCenter.x);
        objects2gaze_b.addInt(this->FaceCenter.y);
        objects2gaze_b.addString("object");
        objects2gaze_b.addInt(this->ObjectCenter.x);
        objects2gaze_b.addInt(this->ObjectCenter.y);
        detector2gazePort.write();                                

        // Display the disparity map. 
        IplImage out = disp;
        outImage.resize(out.width, out.height);
        cvCopy( &out, (IplImage *) outImage.getIplImage());
        outPort.write();

        // Display the color image.
        IplImage crop = outColour_cv; //inColour_cv;
        cropOutImage.resize(crop.width, crop.height);
        cvCopy( &crop, (IplImage *) cropOutImage.getIplImage());
        cropOutPort.write();
    }
};

/********************************************************/
class Module : public yarp::os::RFModule, public closestBlob_IDL
{
    yarp::os::ResourceFinder    *rf;
    yarp::os::RpcServer         rpcPort;

    Processing                  *processing;
    friend class                processing;

    bool                        closing;

    /********************************************************/
    bool attach(yarp::os::RpcServer &source)
    {
        return this->yarp().attachAsServer(source);
    }

public:

    /********************************************************/
    bool configure(yarp::os::ResourceFinder &rf)
    {
        this->rf=&rf;
        std::string moduleName;
#ifdef DEBUG_LOCAL
        moduleName = "detection";
#else
        moduleName = rf.check("name", yarp::os::Value("GC_detection"), "module name (string)").asString();
#endif
        // override the module name to "detection". 
        //moduleName = "detection";
        //rf.check()
        setName(moduleName.c_str());

#ifdef DEBUG_LOCAL        
        rpcPort.open(("/"+getName("/rpc")).c_str());
#else
        rpcPort.open("/" + moduleName + "/rpc");
#endif

        closing = false;

        processing = new Processing( moduleName );

        /* now start the thread to do the work */
        processing->open();

        attach(rpcPort);
                
        MyCallBack.open("/"+ moduleName + "/detector_command:i"); //"/computed-torque/qDes:i");
        MyCallBack.useCallback();  
        

        return true;
    }

    /**********************************************************/
    bool close()
    {
        processing->interrupt();
        processing->close();
        delete processing;
        return true;
    }

    /**********************************************************/
    bool quit(){
        closing = true;
        return true;
    }

    /********************************************************/
    double getPeriod()
    {
        return 0.1;
    }

    /********************************************************/
    bool updateModule()
    {
        return !closing;
    }
};

/********************************************************/
int main(int argc, char *argv[])
{
    yarp::os::Network::init();

    yarp::os::Network yarp;
    if (!yarp.checkNetwork())
    {
        yError("YARP server not available!");
        return 1;
    }

    Module module;
    yarp::os::ResourceFinder rf;

    rf.setVerbose();
    rf.configure(argc,argv);

    return module.runModule(rf);
}
