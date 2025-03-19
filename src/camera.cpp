#include "ti_es_cam_package/camera.hpp"
#include <cstdint>
#include <cstdlib>
#include <ASICamera2.h>
#include <iostream>


/**
 * @brief Constructor
 * 
 * @param width
 * @param height
 */
Camera::Camera(uint16_t width, uint16_t height)
{
	this->_camWidth 	= width;
	this->_camHeight 	= height;
	this->_imgType 		= ASI_IMG_RAW8;

	this->_cameraInfo 	= (ASI_CAMERA_INFO*)malloc(sizeof(ASI_CAMERA_INFO));

	int err = this->initCamera();
	if (err != 0) {
		std::cout << "Can't init the camera!\n";
	}
}


/**
 * @brief Constructor
 * 
 * @param width
 * @param height
 * @param imgType
 */
Camera::Camera(uint16_t width, uint16_t height, ASI_IMG_TYPE imgType)
{
	this->_camWidth 	= width;
	this->_camHeight 	= height;
	this->_imgType 		= imgType;

	this->_cameraInfo 	= (ASI_CAMERA_INFO*)malloc(sizeof(ASI_CAMERA_INFO));

	int err = this->initCamera();
	if (err != 0) {
		std::cout << "Can't init the camera!\n";
	}
}


/**
 * @brief Destructor
 * 
 */
Camera::~Camera()
{
	// Shutdown the camera
	ASICloseCamera(this->_cameraInfo->CameraID);

	// Free all the pointers
	if (this->_cameraInfo != nullptr) {
		free(this->_cameraInfo);
	}
}


/**
 * @brief Method to initialize the camera with default settings
 * 
 * @return int
 */
int Camera::initCamera()
{
	int err = 0;

	// Check if there are any cameras connected
	if (this->getTotalCameras() <= 0) {
		err = -1;
	}

	// Try to get the information of the current connected camera
	ASIGetCameraProperty(this->_cameraInfo, 0);

	// Try to open the camera in the software
	if (ASIOpenCamera(this->_cameraInfo->CameraID)) {
		std::cout << "Failed to open camera!\n";
		err = -2;
	}

	// Try to init the camera
	ASIInitCamera(this->_cameraInfo->CameraID);

	// Set values to configure the camera
	this->setExposureValue(1000);
	this->setGainValue(20);
	this->setBandWidthOverloadValue(40);

	// Try to configure the camera
	int r = this->configureCamera();
	if (r != 0) {
		std::cout << "Can´t configure the camera!\n";
		err = -3;
	}

	return err;
}


/**
 * @brief Method that configures the cam settings by every update cycle so the cam can be live updated
 * 
 * @return int
 */
int Camera::configureCamera()
{
	int err = 0;

	ASISetControlValue(this->_cameraInfo->CameraID, ASI_EXPOSURE, this->_exposureValue, ASI_FALSE);
  	ASISetControlValue(this->_cameraInfo->CameraID, ASI_GAIN, this->_gainValue, ASI_FALSE);
  	ASISetControlValue(this->_cameraInfo->CameraID, ASI_BANDWIDTHOVERLOAD, this->_bandWidthOverloadValue, ASI_FALSE);

  	ASISetROIFormat(this->_cameraInfo->CameraID, this->_camWidth, this->_camHeight, 1, this->_imgType);

	return err;
}


/**
 * @brief Method to start the video capture sequence by the camera
 * 
 * @return int
 */
int Camera::startCamera()
{
	int err = 0;

	if (ASIStartVideoCapture(this->_cameraInfo->CameraID) != ASI_SUCCESS) {
		std::cout << "Could not start the video capture sequence!\n";
		err = -1;
	}

	return err;
}


/**
 * @brief Method to stop the camera sequence
 * 
 * @return int
 */
int Camera::stopCamera()
{
	int err = 0;

	if (ASIStopVideoCapture(this->_cameraInfo->CameraID) != ASI_SUCCESS) {
		std::cout << "Could not stop the video capture sequence!\n";
		err = -1;
	}

	return err;
}


/**
 * @brief 
 * 
 * @return struct VideoFrame *
 */
struct VideoFrame* Camera::captureVideoFrame()
{
	// Create a new frame object
	struct VideoFrame *frame = (struct VideoFrame *)malloc(sizeof(struct VideoFrame));
	if (frame == nullptr) {
		std::cout << "Can´t allocate a frame struct!\n";
		return nullptr;
	}

	// Put meta data into the frame object 
	frame->width 	= this->_camWidth;
	frame->height 	= this->_camHeight;
	frame->format  	= this->_imgType;
	frame->buffer 	= (unsigned char *)malloc(frame->width * frame->height * sizeof(unsigned char));
	if (frame->buffer == nullptr) {
		std::cout << "Could not initialize framebuffer in mem!\n";
		free(frame);
		return nullptr;
	}

	// Try to get video data
	if (ASIGetVideoData(this->_cameraInfo->CameraID, frame->buffer, frame->width * frame->height, 2000) != ASI_SUCCESS) {
		std::cout << "Can´t retrieve data from camera!\n";
		free(frame->buffer);
		free(frame);
		return nullptr;
	}

	return frame;
}


/**
 * @brief Wrapper method to get the total connected ZWO ASI cameras
 * 
 * @return int
 */
int Camera::getTotalCameras()
{
	return ASIGetNumOfConnectedCameras();
}


/**
 * @brief Getter for retrieving the information about the camera
 * 
 * @return ASI_CAMERA_INFO
 */
ASI_CAMERA_INFO* Camera::getCameraInfo()
{
	return this->_cameraInfo;
}


/**
 * @brief Getter for retrieving the camera exposure value
 * 
 * @return uint32_t
 */
uint32_t Camera::getExposureValue()
{
	return this->_exposureValue;
}


/**
 * @brief Getter for retrieving the gain value of the camera
 * 
 * @return uint8_t
 */
uint8_t Camera::getGainValue()
{
	return this->_gainValue;
}


/**
 * brief Getter for retrieving the bandwidthoverload value of the camera
 * 
 * @return uint8_t
 */
uint8_t	Camera::getBandWidthOverloadValue()
{
	return this->_bandWidthOverloadValue;
}


/**
 * @brief Setter for setting the value for the exporure time of the camera
 * 
 * @param value
 */
void Camera::setExposureValue(uint32_t value)
{
	this->_exposureValue = value;
}


/**
 * @brief Setter for setting the gain of the camera
 * 
 * @param value
 */
void Camera::setGainValue(uint8_t value)
{
	this->_gainValue = value;
}


/**
 * @brief Setter for setting the bandwith overload for the camera
 * 
 * @param value
 */
void Camera::setBandWidthOverloadValue(uint8_t value)
{
	this->_bandWidthOverloadValue = value;
}