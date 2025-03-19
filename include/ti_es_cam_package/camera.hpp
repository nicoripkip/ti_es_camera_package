#ifndef TI_ES_CAM_PACKAGE_CAMERA_HPP
#define TI_ES_CAM_PACKAGE_CAMERA_HPP


#include <cstdint>
#include <ASICamera2.h>


struct VideoFrame
{
	uint16_t 		width;
	uint16_t		height;
	ASI_IMG_TYPE 	format;
	unsigned char*	buffer;
};


class Camera
{
public:
	Camera(uint16_t width, uint16_t height);
	Camera(uint16_t width, uint16_t height, ASI_IMG_TYPE imgType);
	~Camera();

	int 				initCamera();
	int 				configureCamera();

	struct VideoFrame*	captureVideoFrame();
			

	// Getters
	ASI_CAMERA_INFO*	getCameraInfo();
	int  				getTotalCameras();	

	uint32_t 			getExposureValue();
	uint8_t 			getGainValue();
	uint8_t				getBandWidthOverloadValue();

	// Setters
	void 				setExposureValue(uint32_t value);
	void				setGainValue(uint8_t value);
	void				setBandWidthOverloadValue(uint8_t value);

private:
	ASI_CAMERA_INFO*	_cameraInfo;
	uint32_t 			_exposureValue;
	uint8_t				_gainValue;
	uint8_t				_bandWidthOverloadValue;
	uint16_t			_camWidth;
	uint16_t			_camHeight;
	ASI_IMG_TYPE		_imgType;

};



#endif