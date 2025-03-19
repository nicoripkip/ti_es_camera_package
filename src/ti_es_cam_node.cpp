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

  Camera astrocam(CAM_WIDTH, CAM_HEIGHT);

  astrocam.startCamera();

  // Do opencv stuff
  cv::namedWindow("Astrocam", cv::WINDOW_AUTOSIZE);

  while (true) {
    struct VideoFrame* f = astrocam.captureVideoFrame();

    cv::Mat frame(f->height, f->width, CV_8UC1, f->buffer);

    cv::imshow("Camera", frame);

    if (cv::waitKey(30) >= 0) break;
  }


  astrocam.stopCamera();


  printf("Hej, Verden!\n");
  return 0;
}
