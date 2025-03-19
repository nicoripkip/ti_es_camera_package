#include <cstdio>
#include <cstdlib>
#include <opencv2/opencv.hpp>
#include <ASICamera2.h>
#include "ti_es_cam_package/camera.hpp"


#define CAM_WIDTH   1280
#define CAM_HEIGHT  960


int main(int argc, char ** argv)
{
  (void) argc;
  (void) argv;


  int cameras = ASIGetNumOfConnectedCameras();

  if (cameras <= 0) {
    printf("No ASI cameras found!\n");
    return -1;
  }

  ASI_CAMERA_INFO cameraInfo;
  ASIGetCameraProperty(&cameraInfo, 0);
  printf("Detected camera: %s\n", cameraInfo.Name);

  if (ASIOpenCamera(cameraInfo.CameraID)) {
    printf("Failed to open camera!\n");
    return -1;
  }

  ASIInitCamera(cameraInfo.CameraID);

  ASISetControlValue(cameraInfo.CameraID, ASI_EXPOSURE, 1000, ASI_FALSE);
  ASISetControlValue(cameraInfo.CameraID, ASI_GAIN, 20, ASI_FALSE);
  ASISetControlValue(cameraInfo.CameraID, ASI_BANDWIDTHOVERLOAD, 40, ASI_FALSE);

  ASISetROIFormat(cameraInfo.CameraID, CAM_WIDTH, CAM_HEIGHT, 1, ASI_IMG_RAW8);
  ASIStartVideoCapture(cameraInfo.CameraID);

  unsigned char *buffer = (unsigned char *)malloc(CAM_WIDTH * CAM_HEIGHT * sizeof(char));

  // Do opencv stuff
  cv::namedWindow("Astrocam", cv::WINDOW_AUTOSIZE);

  while (true) {
    if (ASIGetVideoData(cameraInfo.CameraID, buffer, CAM_WIDTH * CAM_HEIGHT, 2000) != ASI_SUCCESS) {
      printf("Cant get data from camera!\n");
    }

    cv::Mat frame(CAM_HEIGHT, CAM_WIDTH, CV_8UC1, buffer);

    cv::imshow("Camera", frame);

    if (cv::waitKey(30) >= 0) break;
  }

  free(buffer);

  ASIStopVideoCapture(cameraInfo.CameraID);

  // Always clean up 
  ASICloseCamera(cameraInfo.CameraID);

  printf("Hej, Verden!\n");
  return 0;
}
