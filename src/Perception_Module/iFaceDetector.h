/*
Institute for Inforcomm Research
By Chen Tai Pang, Lawrence
FaceDetector.cpp
Function:
 Normalize images
 Detect faces
 Detect Eyes
*/

#ifndef OPENCV2_4
#define OPENCV2_4
#endif 

#define FACE_SIZE 44

typedef struct tagFaceParameters
{
	Rect FaceRegion; // Detected face region (using the coordinate from the captured image.
	Rect LeftEyeRegion;  // Left eye position relative to the detected face (inside the face)
	Rect RightEyeRegion; // Righe eye position relative to the detected face (inside the face) 
} FaceParameters;

class iFaceDetector
{
	int ClassifiedLoaded, ClassifiedLoadedSingleImage; // Initialized is not loaded at starting of the DLL
	int ImgHeight,ImgWidth;
	float ImgScale;
	cv::Mat gray; // The full size image in gray color
	cv::Mat smallImg; // The decimated image from gray image. 
	cv::Mat smallImgROI; // Detected Faceimage extracted from the gray image.
	cv::Size MinFaceSize;
	cv::Size MaxFaceSize;
	CascadeClassifier cascade,cascadeSingleImage,lbpCascade1;
	//CascadeClassifier leftEyeCascade, rightEyeCascade;
	#ifdef USE_GPU_DETECTOR
	//gpu::CascadeClassifier_GPU cascade_gpu,lbpCascade1_gpu;
	//using namespace cv::gpu;
	#endif
	std::vector<cv::Rect> faces,faces1;// Added on 14 June 2012 
	std::vector<int> rweights;// Added on 14 June 2012 	 
	std::vector<FaceParameters> DetectedFaces; 
	cv::Rect *RegionOfInterest;

	void SetRuntimeImageDimension(int Height,int Width);
	void Preprocessing(cv::Mat &InputImg);// For multiple IPcam	
	std::vector<FaceParameters>* FaceDetector_HAAR_LBP_CPU();
	std::vector<FaceParameters>* FaceDetector_HAAR_LBP();	
	int InitializeFaceDetector_LBP_HAAR_CPU(float scale,int Height,int Width,char *HAARClassifierName,char *LBPClassifierName,char *LeftEyeClassifierName,char *RightEyeClassifierName);

	// For eye detection.
	Rect LeftEye,RightEye;
	void objectDetection(cv::Mat& img, CascadeClassifier& cascadeDetector, std::vector<cv::Rect>& objects);
	//bool locateLeftEye();
	//bool locateRightEye();
	//bool locateEyes();
	
public:
	iFaceDetector();
	~iFaceDetector();
		
	std::vector<FaceParameters>* DetectFaceParameters(cv::Mat &Image);	
	int InitializeFaceDetector_LBP_HAAR(float scale,int Height,int Width,char *HAARClassifierName,char *LBPClassifierName,char *LeftEyeClassifierName,char *RightEyeClassifierName);		
	void SetRegionOfInterest(int rx,int ry,int Width,int Height);
	void SetMinFaceSizeForHaar(int FaceWidth); // always square.
	void SetMaxFaceSize(int FaceWidth);
	float GetImgScale(); 
        cv::Mat* GetgrayImage();

	// Using Nvida GPU
	//int InitializeFaceDetector_LBP_HAAR_GPUCV(float scale,int Height,int Width,char *HAARClassifierName,char *LBPClassifierName);
	//std::vector<cv::Rect>* FaceDetector_HAAR_LBP_GPU();
};
