// plactice2_node.cpp
// done
// 단순 이미지 서브스크라이빙 테스트
// 가제보 카메라 서브스크라이빙
// 카메라 이미지 관심영역(ROI)지정 및 각 관심영역 이미지 출력
// 사실상 테스트 완료 쓸모 없음

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

cv::Mat imageROI_1, imageROI_2, imageROI_3;

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  cv_bridge::CvImagePtr cv_ptr;

  try
  {
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8); // 센서 메시지에서 이미지 추출
  }
  catch(cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception : %s",e.what());
    return;
  }

  cv::Mat frame = cv_ptr->image; // 이미지 변수에다 복사

  cv::resize(frame, frame, cv::Size(((frame.cols) * 0.5) , ((frame.rows) * 0.5)), 0, 0, CV_INTER_AREA); // 원본 이미지 리사이징

  imageROI_1 = frame(cv::Rect(0, 0, 50, 100)); // roi 정의
  imageROI_2 = frame(cv::Rect(50, 0, 910, 540));
  imageROI_3 = frame(cv::Rect(0, 100, 50, 440));

  try
  {
    cv::imshow("view_ROI_1", imageROI_1); // 각 roi마다 창 띄우기
    cv::imshow("view_ROI_2", imageROI_2);
    cv::imshow("view_ROI_3", imageROI_3);

    if(cv::waitKey(1) == 27) return;
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "image_listener");
  ros::NodeHandle nh;
  cv::namedWindow("view_ROI_2");
  cv::startWindowThread();
  image_transport::ImageTransport it(nh);
  // image_transport::Subscriber sub = it.subscribe("/plactice_image_raw", 1, imageCallback); // /plactice_image_raw : 이미지
  image_transport::Subscriber sub = it.subscribe("/camera/rgb/image_raw", 1, imageCallback); // /camera/rgb/image_raw : 가제보 카메라
  
  // image_transport::Publisher pub = it.advertise("/plactice_imageROI2", 1);
  // sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", imageROI_2).toImageMsg();
  // pub.publish(msg);

  ros::spin();
  cv::destroyWindow("view_ROI_2");
}