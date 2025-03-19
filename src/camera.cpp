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
	if (this->_cameraInfo != nullptr) {
		free(this->_cameraInfo);
	}
}


/**
 * 
 * 
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
		printf("Failed to open camera!\n")
		err = -2;
	}

	// Try to init the camera
	ASIInitCamera(this->_cameraInfo->CameraID);

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

	SetControlValue(this->_cameraInfo->CameraID, ASI_EXPOSURE, this->_exposureValue, ASI_FALSE);
  	ASISetControlValue(this->_cameraInfo->CameraID, ASI_GAIN, this->_gainValue, ASI_FALSE);
  	ASISetControlValue(this->_cameraInfo->CameraID, ASI_BANDWIDTHOVERLOAD, this->_bandWidthOverloadValue, ASI_FALSE);

  	ASISetROIFormat(this->_cameraInfo->CameraID, this->_camWidth, this->_camHeight, 1, this->_imgType);
  	ASIStartVideoCapture(this->_cameraInfo->CameraID);

	return err;
}


/**
 * 
 * 
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
	this->_gainValue;
}


/**
 * brief Getter for retrieving the bandwidthoverload value of the camera
 * 
 * @return uint8_t
 */
uint8_t	Camera::getBandWidthOverloadValue()
{
	this->_bandWidthOverloadValue;
}


void Camera::setExposureValue(uint32_t value)
{
	this->_exposureValue = value;
}


void Camera::setGainValue(uint8_t value)
{
	this->_gainValue = value;
}


void Camera::setBandWidthOverloadValue(uint8_t value)
{
	this->_bandWidthOverloadValue = value;
}