#include <cstdio>
#include <cstdlib>
#include <opencv2/opencv.hpp>
#include <ASICamera2.h>
#include <memory>
#include <iostream>
#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "ti_es_cam_package/camera.hpp"


using namespace std::chrono_literals;


const uint16_t CAM_WIDTH  = 1280;
const uint16_t CAM_HEIGHT = 960;


class CameraNode : public rclcpp::Node
{
public:
  /**
   * @brief Constructor
   * 
   */
  CameraNode() 
    : Node("camera_node")
  {
    // Setup publishers
    this->_loggerPublisher          = this->create_publisher<std_msgs::msg::String>("ti/es/logger_data", 10);
    this->_videoDataPublisher       = this->create_publisher<std_msgs::msg::String>("ti/es/video_data", 10);

    // Setup subscribers
    this->_videoConfigSubscription  = this->create_subscription<std_msgs::msg::String>(
        "ti/es/video_config", 
        10, 
        std::bind(&CameraNode::camConfigCallback, this, std::placeholders::_1)
    ); 

    // Set the timers
    this->controlLoop               = this->create_wall_timer(
      1ms, 
      std::bind(&CameraNode::controlLoopCallback, this)
    );
  }


  /**
   * @brief Destructor
   * 
   */
  ~CameraNode()
  {
    // Stop the recording
    this->_astrocam->stopCamera();

    // Clean up the astrocam object
    if (this->_astrocam != nullptr) {
      delete this->_astrocam;
    }
  }


  /**
   * 
   * 
   */
  void configAstroCam()
  {

  }


  /**
   * @brief Method to enable the camera for recording
   * 
   */
  void enableAstroCam()
  {
    // Check if the ptr is set
    if (this->_astrocam != nullptr) {

      // Start the camera
      this->_astrocam->startCamera();

      // Enable the loop
      this->_astrocamEnabled = true;
    }
  }


  /**
   * @brief Method to disable the camera from recording
   * 
   */
  void disableAstroCam()
  {
    // Check if the ptr is set
    if (this->_astrocam != nullptr) {
      // Disable the camera
      this->_astrocam->stopCamera();

      // Disable the loop
      this->_astrocamEnabled = false;
    }
  }


  /**
   * @brief Setter to add an existing astrocam to the node
   * 
   * @param cam
   */
  void setAstroCam(Camera *cam)
  {
    this->_astrocam = cam;
  }


private:
  rclcpp::TimerBase::SharedPtr                            controlLoop;
  Camera*                                                 _astrocam;
  bool                                                    _astrocamEnabled;

  // Publishers
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr     _loggerPublisher;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr     _videoDataPublisher;

  // Subscribers
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr  _videoConfigSubscription;


  /**
   * @brief Callback for controlling 
   * 
   */
  void controlLoopCallback()
  {
    if (this->_astrocamEnabled) {
      std::cout << "Trying to capture video frame data!\n";

      struct VideoFrame* f = this->_astrocam->captureVideoFrame();

      cv::Mat frame(f->height, f->width, CV_8UC1, f->buffer);

      cv::imshow("Camera", frame);

      // When a key is pressed, close the window
      if (cv::waitKey(30) >= 0) {
        this->_astrocamEnabled = false;
        cv::destroyWindow("Astrocam");
      }
    }
  }


  /**
   * @brief This callback is used when configurations are published to the cam
   * 
   * @param msg
   */
  void camConfigCallback(std_msgs::msg::String::SharedPtr msg)
  {
    uint32_t  exposure  = 0;
    uint8_t   gain      = 0;
    uint8_t   bandwidth = 0;

    std::cout << "Data: " << msg << "\n";

    bool condition = false;

    this->_astrocam->setExposureValue(exposure);
    this->_astrocam->setGainValue(gain);
    this->_astrocam->setBandWidthOverloadValue(bandwidth);

    this->_astrocam->configureCamera();

    if (condition) {
      this->enableAstroCam();
    } else {
      this->disableAstroCam();
    }
  }
};


/**
 * @brief This is the main function
 * 
 */
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  Camera astrocam(CAM_WIDTH, CAM_HEIGHT);
  std::shared_ptr<CameraNode> cnode = std::make_shared<CameraNode>();

  cnode->setAstroCam(&astrocam);
  cnode->enableAstroCam();

  // Do opencv stuff -> Dit is puur om te testen, als het uiteindelijke product er staat wordt het verwijderd.
  cv::namedWindow("Astrocam", cv::WINDOW_AUTOSIZE);

  rclcpp::spin(cnode);

  return 0;
}
