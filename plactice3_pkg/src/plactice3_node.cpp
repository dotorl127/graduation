// plactice3_node.cpp
// done
// 서브스크라이빙 한 ROI 이미지 처리
// 이미지로 디버깅 하고 가제보 카메라로 확인 할 수 있게 수정
// 허프라인 변환 및 출력 확인(hsv 파라미터 값에 매우 크게 좌우됨)
// 가제보 카메라 이미지로 허프라인 변환 확인
// 영상 잘라내기 및 트랙바 파라미터 추가(argv[3]이 비어있으면 이미지 서브스크라이빙)

// should
// 양쪽 동시에 허프라인 출력 방법 모색(왼쪽, 오른쪽 차선 노드를 각각 따로 실행)
// 차선 데이터로 맵 만드는 방식 공부(point2cloud)
// 소스 이미지로 처리 했을때 이미지 퍼블리싱 부분 추가했는데 불안정

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <string>
#include <iostream>
#include <stdlib.h>
#include <stdio.h>

using namespace std;

static const std::string OPENCV_WINDOW = "view_image_processed";
static int srcFlag = 0; // 0 : 가제보 카메라 서브스크라이빙
                        // 1 : 이미지 서브스크라이빙
static int trackbar = 1; // 0 : 디폴트 적용
                         // 1 : 트랙바 만들기
static int y_hmin, y_hmax, y_smin, y_smax, y_vmin, y_vmax;
static int w_hmin, w_hmax, w_smin, w_smax, w_vmin, w_vmax;

class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;

public:
  ImageConverter() : it_(nh_)
  {
    image_sub_ = it_.subscribe("/camera/rgb/image_raw", 1, &ImageConverter::imageCb, this);
    image_pub_ = it_.advertise("/plactice_imageROI2", 1);

    cv::namedWindow(OPENCV_WINDOW);
  }

  ~ImageConverter()
  {
    cv::destroyWindow(OPENCV_WINDOW);
  }

  void detectWhiteLane(cv::Mat& src, cv::Mat& dst, int hmin, int hmax, int smin, int smax, int vmin, int vmax, int amin, int amax)
  {
      cv::cvtColor(src, dst, CV_BGR2HSV);
      cv::inRange(dst, cv::Scalar(hmin, smin, vmin, amin), cv::Scalar(hmax, smax, vmax, amax), dst);
      //inrange set 255 at value involving input range and set 0 otherwise
      //1 channels ->𝚍𝚜𝚝(I)=𝚕𝚘𝚠𝚎𝚛𝚋(I)0≤𝚜𝚛𝚌(I)0≤𝚞𝚙𝚙𝚎𝚛𝚋(I)0
  }

  void detectYHSVcolor(const cv::Mat& src, cv::Mat& dst, double minHue, double maxHue, double minSat, double maxSat, double minVal, double maxVal) 
  {
    //detectHSVcolor(bev, yellow, 7, 21, 52, 151, 0, 180);//tackbar is false
    cv::Mat hsv;
    cv::cvtColor(src, hsv, cv::COLOR_BGR2HSV);

    inRange(hsv, cv::Scalar(minHue, minSat, minVal), cv::Scalar(maxHue, maxSat, maxVal), dst);
  }

  static void getThreshold( int, void*)  
  {  
    
  }  

  void initMyHSVTrackbar(const std::string &trackbar_name, int *hmin, int *hmax, int *smin, int *smax, int *vmin, int *vmax)
  {
    cv::namedWindow(trackbar_name, cv::WINDOW_AUTOSIZE);

    cv::createTrackbar("h min", trackbar_name, hmin, 179, ImageConverter::getThreshold);
    cv::setTrackbarPos("h min", trackbar_name, *(hmin));

    cv::createTrackbar("h max", trackbar_name, hmax, 179, ImageConverter::getThreshold);
    cv::setTrackbarPos("h max", trackbar_name, *(hmax));

    cv::createTrackbar("s min", trackbar_name, smin, 255, ImageConverter::getThreshold);
    cv::setTrackbarPos("s min", trackbar_name, *(smin));

    cv::createTrackbar("s max", trackbar_name, smax, 255, ImageConverter::getThreshold);
    cv::setTrackbarPos("s max", trackbar_name, *(smax));

    cv::createTrackbar("v min", trackbar_name, vmin, 255, ImageConverter::getThreshold);
    cv::setTrackbarPos("v min", trackbar_name, *(vmin));

    cv::createTrackbar("v max", trackbar_name, vmax, 255, ImageConverter::getThreshold);
    cv::setTrackbarPos("v max", trackbar_name, *(vmax));

    *hmin = cv::getTrackbarPos("h min", trackbar_name);
    *hmax = cv::getTrackbarPos("h max", trackbar_name);
    *smin = cv::getTrackbarPos("s min", trackbar_name);
    *smax = cv::getTrackbarPos("s max", trackbar_name);
    *vmin = cv::getTrackbarPos("v min", trackbar_name);
    *vmax = cv::getTrackbarPos("v max", trackbar_name);
  }

  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
    cv::Mat img;
     // 이미지 변수에다 복사
    cv::resize(cv_ptr->image, img, cv::Size(((cv_ptr->image.cols) * 0.5) , ((cv_ptr->image.rows) * 0.5)), 0, 0, CV_INTER_AREA);

    // imshow("gazebo_camera", cv_ptr->image);

    int minThrForCanny = 80; // 프로세싱 변수 선언
    int maxThrForCanny = 15;
    int kernelSize = 3;
    cv::Mat grayImg, blurImg, cannyImg, bev, yellow_hsv, white_hsv;

    // cv::cvtColor(roi1, grayImg, CV_BGR2GRAY); // 이미지 프로세싱 시작 gray, blur, canny 순
    // cv::GaussianBlur(grayImg, blurImg, cv::Size(5, 5), 0, 5);
    // cv::Canny(blurImg, cannyImg, minThrForCanny, maxThrForCanny, kernelSize);

    // cv::imshow(OPENCV_WINDOW, roi1); // 원본이미지
    // cv::imshow("view_image_processed_gray", grayImg); // 그레이이미지
    // cv::imshow("view_image_processed_blur", blurImg); // 블러이미지
    // cv::imshow("view_image_processed_canny", cannyImg); // 캐니이미지

    bev = img.clone();

    if (trackbar) {
      ImageConverter::initMyHSVTrackbar("YELLOW_TRACKBAR", &y_hmin, &y_hmax, &y_smin, &y_smax, &y_vmin, &y_vmax);
      ImageConverter::detectYHSVcolor(bev, yellow_hsv, y_hmin, y_hmax, y_smin, y_smax, y_vmin, y_vmax);
      // ImageConverter::initMyHSVTrackbar("WHITE_TRACKBAR", &w_hmin, &w_hmax, &w_smin, &w_smax, &w_vmin, &w_vmax);
      // ImageConverter::detectWhiteLane(bev, white_hsv, w_hmin, w_hmax, w_smin, w_smax, w_vmin, w_vmax, 0, 0);
      cv::imshow("YELLOW_TRACKBAR", yellow_hsv);
      // cv::imshow("WHITE_TRACKBAR", white_hsv);
    }
    else { // 색상검출 디폴트
      ImageConverter::detectYHSVcolor(bev, yellow_hsv, 7, 21, 52, 151, 0, 180);
      ImageConverter::detectWhiteLane(bev, white_hsv, 0, 180, 0, 29, 179, 255, 0, 0);
      cv::imshow("YELLOW_TRACKBAR", yellow_hsv); 
      cv::imshow("WHITE_TRACKBAR", white_hsv);
    }

    /*std::vector<cv::Vec4i> lines_white;
    std::vector<cv::Vec4i>::iterator it_white;
    cv::Mat hough_zero_right = cv::Mat::zeros(bev.rows, bev.cols, CV_8UC1);
    cv::Mat hough_white = white_hsv.clone();
    
    cv::HoughLinesP(hough_white, lines_white, 1, (CV_PI/180.0), 80, 80, 5);

    float ladian_w;
    int degree_w;
    
    int w_x0, w_x1, w_y0, w_y1;
    if(!lines_white.empty())
    {
      it_white = lines_white.end() - 1;
      ladian_w = atan2f((*it_white)[3] - (*it_white)[1], (*it_white)[2] - (*it_white)[0]);
      degree_w = ladian_w * 180 / CV_PI;
      if(degree_w >= 10 && degree_w <= 90)
      {
        //  w_lines_buffer = lines_white;
        //  it_w_buffer = w_lines_buffer.end() - 1;
        cv::line(hough_zero_right, cv::Point((*it_white)[0], (*it_white)[1]), cv::Point((*it_white)[2], (*it_white)[3]), cv::Scalar(255, 0, 0), 2);

        // (CvArr* img, CvPoint pt1, CvPoint pt2, CvScalar color, int thickness CV_DEFAULT(1), int line_type CV_DEFAULT(8), int shift CV_DEFAULT(0))
        
        w_x0 = (*it_white)[0];
        w_y0 = (*it_white)[1];
        w_x1 = (*it_white)[2];
        w_y1 = (*it_white)[3];
      }
      else if(degree_w <= 10 && degree_w >= 90)
      {
        //  w_lines_buffer = lines_white;
        //  it_w_buffer = w_lines_buffer.end() - 1;
        cv::line(hough_zero_right, cv::Point((*it_white)[0], (*it_white)[1]), cv::Point((*it_white)[2], (*it_white)[3]), cv::Scalar(0, 0, 255), 2);
        w_x0 = (*it_white)[0];
        w_y0 = (*it_white)[1];
        w_x1 = (*it_white)[2];
        w_y1 = (*it_white)[3];
      }
    }

    cv::imshow("hough_test", hough_zero_right);

    sensor_msgs::ImagePtr msgPub = cv_bridge::CvImage(std_msgs::Header(), "bgr8", roi1).toImageMsg();
    image_pub_.publish(msgPub);*/
    imshow("detect red", yellow_hsv);
    cv::waitKey(1);
  }
};

void detectWhiteLane(cv::Mat& src, cv::Mat& dst, int hmin, int hmax, int smin, int smax, int vmin, int vmax, int amin, int amax)
{
    cv::cvtColor(src, dst, CV_BGR2HSV);
    cv::inRange(dst, cv::Scalar(hmin, smin, vmin, amin), cv::Scalar(hmax, smax, vmax, amax), dst);
    //inrange set 255 at value involving input range and set 0 otherwise
    //1 channels ->𝚍𝚜𝚝(I)=𝚕𝚘𝚠𝚎𝚛𝚋(I)0≤𝚜𝚛𝚌(I)0≤𝚞𝚙𝚙𝚎𝚛𝚋(I)0
}

void detectYHSVcolor(const cv::Mat& src, cv::Mat& dst, double minHue, double maxHue, double minSat, double maxSat, double minVal = 0, double maxVal = 255) 
{
  //detectHSVcolor(bev, yellow, 7, 21, 52, 151, 0, 180);//tackbar is false
  cv::Mat hsv;
  cv::cvtColor(src, hsv, cv::COLOR_BGR2HSV);

  std::vector<cv::Mat> channels;
  cv::split(hsv, channels);

  cv::Mat mask1;
  cv::Mat mask2;

  cv::threshold(channels[0], mask1, maxHue, 255, cv::THRESH_BINARY_INV);
  cv::threshold(channels[0], mask2, minHue, 255, cv::THRESH_BINARY);
  cv::Mat hueMask;

  if (minHue < maxHue)
  {
    hueMask = mask1 & mask2;//all values are zero
  }
  else 
  {//minHue > maxHue
    hueMask = mask1 | mask2;//all values are 255
  }

  cv::threshold(channels[1], mask1, maxSat, 255, cv::THRESH_BINARY_INV);

  cv::threshold(channels[1], mask2, minSat, 255, cv::THRESH_BINARY);

  cv::Mat satMask;
  satMask = mask1 & mask2;

  cv::threshold(channels[0], mask1, maxVal, 255, cv::THRESH_BINARY_INV);
  cv::threshold(channels[0], mask2, minVal, 255, cv::THRESH_BINARY);

  cv::Mat valMask;
  valMask = mask1 & mask2;

  dst = hueMask & satMask & valMask;
}

static void getThreshold( int, void*) {}  

void initMyHSVTrackbar(const std::string &trackbar_name, int *hmin, int *hmax, int *smin, int *smax, int *vmin, int *vmax)
{
  cv::namedWindow(trackbar_name, cv::WINDOW_AUTOSIZE);

  cv::createTrackbar("h min", trackbar_name, hmin, 179, getThreshold);
  cv::setTrackbarPos("h min", trackbar_name, *(hmin));

  cv::createTrackbar("h max", trackbar_name, hmax, 179, getThreshold);
  cv::setTrackbarPos("h max", trackbar_name, *(hmax));

  cv::createTrackbar("s min", trackbar_name, smin, 255, getThreshold);
  cv::setTrackbarPos("s min", trackbar_name, *(smin));

  cv::createTrackbar("s max", trackbar_name, smax, 255, getThreshold);
  cv::setTrackbarPos("s max", trackbar_name, *(smax));

  cv::createTrackbar("v min", trackbar_name, vmin, 255, getThreshold);
  cv::setTrackbarPos("v min", trackbar_name, *(vmin));

  cv::createTrackbar("v max", trackbar_name, vmax, 255, getThreshold);
  cv::setTrackbarPos("v max", trackbar_name, *(vmax));

  *hmin = cv::getTrackbarPos("h min", trackbar_name);
  *hmax = cv::getTrackbarPos("h max", trackbar_name);
  *smin = cv::getTrackbarPos("s min", trackbar_name);
  *smax = cv::getTrackbarPos("s max", trackbar_name);
  *vmin = cv::getTrackbarPos("v min", trackbar_name);
  *vmax = cv::getTrackbarPos("v max", trackbar_name);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_processor");
  /*if(srcFlag) // 이미지 서브스크라이빙
  {
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_(nh_);
    image_transport::Publisher image_pub_ = it_.advertise("/plactice_imageROI2", 1);

    std::string srcImg = "/home/moon/catkin_ws/src/picture/road/road30.jpg";
    if(argv[3] != NULL) { trackbar = 1; }
    else { trackbar = 0; }
    while(nh_.ok())
    {
      cv::Mat img = cv::imread(srcImg, CV_LOAD_IMAGE_COLOR);
      // cv::resize(img, img, cv::Size(((img.cols) * 0.5) , ((img.rows) * 0.5)), 0, 0, CV_INTER_AREA);
      // img = img(cv::Rect(0, 100, img.cols, ((img.rows) - 100)));

      int minThrForCanny = 80; // 프로세싱 변수 선언
      int maxThrForCanny = 15;
      int kernelSize = 3;
      cv::Mat grayImg, blurImg, cannyImg, yellow_hsv, white_hsv;

      cv::cvtColor(img, grayImg, CV_BGR2GRAY); // 이미지 프로세싱 시작 gray, blur, canny 순

      if (trackbar) {
        // initMyHSVTrackbar("YELLOW_TRACKBAR", &y_hmin, &y_hmax, &y_smin, &y_smax, &y_vmin, &y_vmax);
        // detectYHSVcolor(img, yellow_hsv, y_hmin, y_hmax, y_smin, y_smax, y_vmin, y_vmax);
        initMyHSVTrackbar("WHITE_TRACKBAR", &w_hmin, &w_hmax, &w_smin, &w_smax, &w_vmin, &w_vmax);
        detectWhiteLane(img, white_hsv, w_hmin, w_hmax, w_smin, w_smax, w_vmin, w_vmax, 0, 0);
        // cv::imshow("YELLOW_TRACKBAR", yellow_hsv);
        cv::imshow("WHITE_TRACKBAR", white_hsv);
      }
      else { // 색상검출 디폴트
        detectYHSVcolor(img, yellow_hsv, 7, 21, 52, 151, 0, 180);
        detectWhiteLane(img, white_hsv, 0, 180, 0, 29, 179, 255, 0, 0);
        cv::imshow("YELLOW_TRACKBAR", yellow_hsv); 
        cv::imshow("WHITE_TRACKBAR", white_hsv);
      }

      cv::GaussianBlur(white_hsv, blurImg, cv::Size(5, 5), 0, 5);
      cv::Canny(blurImg, cannyImg, minThrForCanny, maxThrForCanny, kernelSize);

      cv::imshow(OPENCV_WINDOW, img); // 원본이미지
      cv::imshow("view_image_processed_canny", cannyImg); // 캐니이미지

      std::vector<cv::Vec4i> lines_white;
      std::vector<cv::Vec4i>::iterator it_white;
      cv::Mat hough_zero_right = cv::Mat::zeros(img.rows, img.cols, CV_8UC1);

      cv::Mat hough_white = cannyImg.clone();
      
      cv::HoughLinesP(hough_white, lines_white, 1, (CV_PI/180.0), 80, 80, 5);

      float ladian_w;
      int degree_w;
      
      int w_x0, w_x1, w_y0, w_y1;
      if(!lines_white.empty())
      {
        it_white = lines_white.end() - 1;
        ladian_w = atan2f((*it_white)[3] - (*it_white)[1], (*it_white)[2] - (*it_white)[0]);
        degree_w = ladian_w * 180 / CV_PI;
        if(degree_w >= 10 && degree_w <= 90)
        {
          cv::line(img, cv::Point((*it_white)[0], (*it_white)[1]), cv::Point((*it_white)[2], (*it_white)[3]), cv::Scalar(255, 0, 0), 2, CV_AA);
          w_x0 = (*it_white)[0];
          w_y0 = (*it_white)[1];
          w_x1 = (*it_white)[2];
          w_y1 = (*it_white)[3];
        }
        if(degree_w >= -90 && degree_w <= -10)
        {
          cv::line(img, cv::Point((*it_white)[0], (*it_white)[1]), cv::Point((*it_white)[2], (*it_white)[3]), cv::Scalar(0, 0, 255), 2, CV_AA);
          w_x0 = (*it_white)[0];
          w_y0 = (*it_white)[1];
          w_x1 = (*it_white)[2];
          w_y1 = (*it_white)[3];
        }
      }
      sensor_msgs::ImagePtr msgPub = cv_bridge::CvImage(std_msgs::Header(), "bgr8", img).toImageMsg();
      image_pub_.publish(msgPub);

      cv::imshow("hough_test", img);

      if(cv::waitKey(1) == 27) break;
      ros::spin();
    }
  }
  else // 가제보 카메라 서브스크라이빙*/
    ImageConverter ic;

  ros::spin();
  return 0;
}