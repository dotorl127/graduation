// plactice1_node.cpp
// done
// 포인트클라우드 예제 테스트 완료
// 포인트클라우드를 임의로 뿌리고 메시지 퍼블리싱
// 포인트클라우드 메시지를 이미지 메시지로 변환하여 퍼블리싱도 하지만 이미지 토픽은 rviz에서 확인이 안됨
// -> 확인 결과 yz평면에서 바라보는 이미지가 출력 됨
// 메커니즘 확인 완료 랜덤으로 좌우 포인트 클라우드 만들고 색도 다르게 지정함
// 허프변환하여 검출한 선 이미지를 가지고 픽셀접근하여 x, y 좌표를 뽑아 포인트클라우드로 퍼블리싱 성공
// warpPerspective로 선 인식 및 곡선 그리기 성공, 점선은 조금 더 고민해봐야함
// warpPerspective revers 성공 특정 부분을 잘라내더라도 warp 사이즈는 원본과 동일해야함
// 1차 코드 merge 작업 완료 merge 전과 결과는 동일
// cloud point 부분은 아직도 수정 필요
// 서브스크라이브 부분 수정
// 표지판 인식부분 문제있음 로직문제 해결 cvtcolor 에러뜸 해결
// 클라우드 뿌리는 것 수정 완료 -> 렉이 너무 심함 해결해야 할 듯 -> 해결 포인트 클라우드 배열 크기 문제인듯 함
// imshow 오류는 hough line을 찾지 못해 나온 것 검정화면으로 초기화 하여 해결
// pixel 접근으로 가이드 라인을 그려 놓고 허프라인으로 직선 검출하는 방법으로 수정
// 허프라인으로 검출 된 직선 중 적절한 것을 선택하기 위하여 필터링 하는 방법 연구

// should
// 영상 테스트 및 최적화

#include <ros/ros.h>
#include <std_msgs/String.h>

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>

#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>  
#include <opencv2/highgui.hpp>  
#include <opencv2/xfeatures2d.hpp>
#include <opencv2/features2d.hpp>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#include <iostream>
#include <string>
#include <cmath>

using namespace cv;
using namespace std;

class img_process
{
  private:
  const int MAX_FEATURES = 100;
  int width, height, check = 0,
  minHue = 0, maxHue = 10, 
  minSat = 230, maxSat = 255, 
  minVal = 35, maxVal = 45;
  float x, y, imgWidth, imgHeight;
  bool matchSign = false, sign_label = false;
  string text;
  std_msgs::String msg;

  cv_bridge::CvImagePtr cv_ptr;

  string sign10 = "/home/moon/catkin_ws/src/picture/sign/10.png";
  string sign20 = "/home/moon/catkin_ws/src/picture/sign/20.png";
  string stopsign = "/home/moon/catkin_ws/src/picture/sign/stop.png";

  Mat img, src_hsv_sign, src_blur, src_gray, src_threshold, roi_right, roi_left, canvas, warpImg, trans, result, img_copy, src_sign, src_hsv_trafficLight;
  Mat imMatches;
  Mat color_dst;
  Size_<int> warpSize;

  vector<Point> points, points2;
  vector<Point> points_box; 
  
  public:
  ros::NodeHandle nh_;
  ros::Publisher pcl_pub;
  ros::Publisher msg_pub; 
  image_transport::ImageTransport it_;
  image_transport::Subscriber img_sub;
  pcl::PointCloud<pcl::PointXYZRGB> cloud;
  sensor_msgs::PointCloud2 output;

  img_process();
  ~img_process();
  void imageCallback(const sensor_msgs::ImageConstPtr& msg);
  void img_preProcess();
  void img_detectTrafficLight();
  void img_labeling();
  void img_warp();
  void img_postProcess();
  void img_detectLine();
  void img_matchingSign();
  void img_showReselt();
  void img_publishLine();
};

img_process::img_process() : it_(nh_)
{
  pcl_pub = nh_.advertise<sensor_msgs::PointCloud2> ("/pcl_output", 1);
  msg_pub = nh_.advertise<std_msgs::String>("/state", 1);
  img_sub = it_.subscribe("/camera/rgb/image_raw", 1, boost::bind(&img_process::imageCallback, this, _1));

  msg.data = "";
  this->src_sign = imread(sign20);
  imgWidth = img.cols;
  imgHeight = img.rows;
  Size warpSize(img.cols, img.rows);
}

img_process::~img_process()
{
  destroyAllWindows();
}

///////////////////////////////////////////////////////////////////////////////////////// 추가부분

void img_process::imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  try
  {
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8); // 센서 메시지에서 이미지 추출
  }
  catch(cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception : %s", e.what());
    return;
  }

  img = cv_ptr->image.clone();
  resize(img, img, cv::Size(((img.cols) * 0.5) , ((img.rows) * 0.5)), 0, 0, CV_INTER_AREA);
  
  img_preProcess();
  img_detectTrafficLight();
  // img_labeling();
  img_showReselt();
}

/////////////////////////////////////////////////////////////////////////////////////////////////////

void img_process::img_preProcess()
{
  imMatches = Mat::zeros(30, 30, CV_8U);

  cvtColor(img, src_hsv_sign, COLOR_BGR2HSV);
  cvtColor(img, src_gray, COLOR_BGR2GRAY);
  inRange(src_hsv_sign, Scalar(0, 210, 160), Scalar(0, 255, 255), src_hsv_trafficLight);
  inRange(src_hsv_sign, Scalar(minHue, minSat, minVal), Scalar(maxHue, maxSat, maxVal), src_hsv_sign);
  blur(src_hsv_sign, src_blur, Size(3, 3));
  // segmentation error의 경우 inRange 함수 안에서 src, dst를 바꿔 코딩하여 dst가 0값이라 발생했음 해결
}

void img_process::img_detectTrafficLight()
{
  Mat img_test = img.clone();
  Mat blur_img;

  blur(src_gray, blur_img, Size(3, 3));

  vector<Vec3f> circles;
  HoughCircles(blur_img, circles, HOUGH_GRADIENT, 1, src_gray.rows/8, 200, 30, 0, 0);

  for(size_t i = 0; i < circles.size(); i++)
  {
    int B = img_test.at<Vec3b>(circles[i][1], circles[i][0])[0];
    int G = img_test.at<Vec3b>(circles[i][1], circles[i][0])[1];
    int R = img_test.at<Vec3b>(circles[i][1], circles[i][0])[2];

    Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
    int radius = cvRound(circles[i][2]);

    if(B < 50 && G < 50 && R > 100)
    {
      circle(img_test, center, radius, Scalar(255, 0, 0), 2, 8, 0);
      msg.data = "trafficLightRed";
    }
    else if(B < 50 && G > 100 && R < 50)
    {
      circle(img_test, center, radius, Scalar(0, 0, 255), 2, 8, 0);
      msg.data = "trafficLightGreen";
    }
    else ;
  }

  imshow("test", img_test);
  waitKey(1);
}

void img_process::img_labeling()
{
  Mat img_labels, stats, centroids;
  int numOfLables = connectedComponentsWithStats(src_blur, img_labels, stats, centroids, 8, CV_32S);
  Mat src_labels;
  src_labels = img.clone();

  msg.data = "";
  
  for (int j = 1; j < numOfLables; j++)
  {
    int area = stats.at<int>(j, CC_STAT_AREA);
    int left = stats.at<int>(j, CC_STAT_LEFT);
    int top  = stats.at<int>(j, CC_STAT_TOP);
    float width = stats.at<int>(j, CC_STAT_WIDTH);
    float height  = stats.at<int>(j, CC_STAT_HEIGHT);
    if(width >= 80 && height >= 80 && (width / height) >= 0.90 && (width / height) <= 1.10) // 빨강 직사각형 라벨은 width, height 변수가 int 일 때 나타남
    {
      src_gray = src_gray(Rect(left, top, width, height)); // rect error occur -> 한 frame에 두 개 이상 빨강 라벨이 생길 시 생김
      img_matchingSign();
      rectangle(src_labels, Point(left, top), Point(left + width, top + height), Scalar(0, 0, 255), 1);
      putText(src_labels, text, Point(left, top - 20), FONT_HERSHEY_SIMPLEX, 1, Scalar(0, 0, 255), 2);
    }
    else
    {
      rectangle(src_labels, Point(left, top), Point(left + width, top + height), Scalar(255, 0, 0), 1);
    }
  }
  // imshow("matching", src_labels);
  // imshow("imMatches", imMatches);
  // waitKey(1);
}

void img_process::img_warp()
{
  vector<Point2f> corners(4);
  corners[0]=Point2f(335, 542);
  corners[1]=Point2f(424, 542);
  corners[2]=Point2f(0, 784);
  corners[3]=Point2f(581, 784);

  points_box.push_back(Point(335, 542));
  points_box.push_back(Point(424, 542));
  points_box.push_back(Point(581, 784));
  points_box.push_back(Point(0, 784));

  Mat img_copy = img.clone();
  polylines(img_copy, points_box, true, Scalar(255, 0, 0), 2);
  imshow("source", img_copy);
  points_box.clear();

  warpImg = img.clone();

  vector<Point2f> warpCorners(4);
  warpCorners[0]=Point2f(0, 0);
  warpCorners[1]=Point2f(warpImg.cols, 0);
  warpCorners[2]=Point2f(0, warpImg.rows);
  warpCorners[3]=Point2f(warpImg.cols, warpImg.rows);

  trans = getPerspectiveTransform(corners, warpCorners);	 

  warpPerspective(img, warpImg, trans, warpSize);
}

void img_process::img_postProcess()
{
  cvtColor(warpImg, src_gray, CV_BGR2GRAY);
  threshold(src_gray, src_threshold, 122, 255, THRESH_BINARY);

  canvas = Mat::zeros(warpImg.rows, warpImg.cols, img.type());

  roi_left = src_threshold(Rect(0, 0, src_threshold.cols / 3, src_threshold.rows));
  roi_right = src_threshold(Rect(src_threshold.cols / 3 * 2, 0, src_threshold.cols / 3, src_threshold.rows));
}

void img_process::img_detectLine()
{

  // pixel 접근 방식
  
  for(int i = 0; i < roi_left.rows - 1; i += 20)
  {
    int value = 0;
    for(int j = roi_left.cols - 1; j > 0; j--)
    {
      value = roi_left.at<uchar>(i, j); // pixel 접근 x,y순이 아니라 y,x순임 개소름
      if(value != 0)
      {
        circle(roi_left, Point(j, i), 5, Scalar(255, 0, 0));
        points.push_back(Point(j, i));
        break;
      }
    }
  }

  polylines(canvas, points, false, Scalar(0, 0, 255), 20);
  // points.clear();

  for(int i = 0; i < roi_right.rows - 1; i += 20)
  {
    int value = 0;
    for(int j = 0; j < roi_right.cols - 1; j++)
    {
      value = roi_right.at<uchar>(i, j); // pixel 접근 x,y순이 아니라 y,x순임 개소름
      if(value != 0)
      {
        circle(roi_right, Point(j, i), 5, Scalar(255, 0, 0));
        points2.push_back(Point(j + (src_threshold.cols / 3 * 2), i));
        break;
      }
    }
  }

  polylines(canvas, points2, false, Scalar(0, 255, 0), 20);
  // points2.clear();
  
  Mat hough;
  Canny(canvas, hough, 50, 200, 3); // 추가부분
  imshow("canny output", hough);

  vector<Vec4i> lines;
  HoughLinesP(hough, lines, 1, CV_PI/180, 120, 200, 200);
  color_dst = Mat::zeros(warpImg.rows, warpImg.cols, img.type());;
  for( size_t i = 0; i < lines.size(); i++ )
  {
    line(color_dst, Point(lines[i][0], lines[i][1]), Point(lines[i][2], lines[i][3]), Scalar(0,0,255), 3, 8 );
  }
  imshow("hough result", color_dst);
}

void img_process::img_matchingSign()
{
  vector<KeyPoint> keypoints1, keypoints2;
  Mat descriptors1, descriptors2;
  Mat im2Gray;
  cvtColor(src_sign, im2Gray, COLOR_BGR2GRAY);

  resize(im2Gray, im2Gray, cv::Size(src_gray.cols, src_gray.rows), 0, 0, CV_INTER_AREA);

  Ptr<Feature2D> orb = ORB::create(MAX_FEATURES);
  orb->detectAndCompute(src_gray, noArray(), keypoints1, descriptors1);
  orb->detectAndCompute(im2Gray, noArray(), keypoints2, descriptors2);

  vector<vector<DMatch>> matches;
  BFMatcher matcher(NORM_HAMMING);
  int k = 2;
  matcher.knnMatch(descriptors1, descriptors2, matches, k);

  vector<DMatch> goodMatches;
  float nndrRatio = 0.7f;
  for(int i = 0; i < matches.size(); i++)
  {
    if(matches.at(i).size() == 2 && matches.at(i).at(0).distance <= nndrRatio * matches.at(i).at(1).distance)
      goodMatches.push_back(matches[i][0]);
  }

  drawMatches(src_gray, keypoints1, im2Gray, keypoints2, goodMatches, imMatches, Scalar(0, 255, 0), Scalar(0, 0, 255), vector<char>(), 2);

  float a = goodMatches.size();
  float b = matches.size();
  int r = a / b * 100;

  if(r > 65)
  {
    text.clear();
    switch(check)
    {
      case 0:
      {
        text = to_string(r) + "% sign 10";
        msg.data = "speed10";
      }
      break;
      case 1:
      {
        text = to_string(r) + "% sign 20";
        msg.data = "speed20";
      }
      break;
      case 2:
      {
        text = to_string(r) + "% sign stop";
        msg.data = "stop";
      }
      break;
      default:
      break;
    }
  }
  else
  {
    text = to_string(r) + "%";
    ++check;
    switch(check)
    {
      case 1:
      src_sign = imread(sign20);
      break;
      case 2:
      src_sign = imread(stopsign);
      break;
      default:
      {
        src_sign = imread(sign10);
        check = 0;
      }
      break;
    }
  }
}

void img_process::img_publishLine()
{
  int a = 0;
  cloud.width = 100;
  cloud.height = 1;
  cloud.points.resize(cloud.width * cloud.height);

  ros::Time start = ros::Time::now();
  output.header.stamp = start;
  ++output.header.seq;
  
  for(int i = 0; i < points.size(); i++)
  {
    x = points[i].x / imgWidth;
    y = points[i].y / imgHeight;
    cloud.points[i].x = x;
    cloud.points[i].y = y;
    cloud.points[i].r = 255;
    cloud.points[i].g = 0;
    cloud.points[i].b = 0;
  }

  for(int i = points.size(); i < points2.size(); i++)
  {
    x = points2[i].x / imgWidth;
    y = points2[i].y / imgHeight;
    cloud.points[i].x = x;
    cloud.points[i].y = y;
    cloud.points[i].r = 0;
    cloud.points[i].g = 0;
    cloud.points[i].b = 255;
  }

  points.clear();
  points2.clear();
  
  pcl::toROSMsg(cloud, output);
  output.header.frame_id = "odom";
  pcl_pub.publish(output);
}

void img_process::img_showReselt()
{
  /*imshow("binary warpImg left", roi_left); 검출 라인 출력 라인
  imshow("binary warpImg right", roi_right);

  imshow("line result", color_dst);

  result = img_copy.clone();
  img_copy = img.clone();
  warpPerspective(color_dst, result, trans, warpSize, WARP_INVERSE_MAP); // WARP_INVERSE_MAP

  add(result, img_copy, img_copy);

  imshow("final result", img_copy);*/

  // imshow("gazebo_camera", img);
    
  // waitKey(1);
  msg_pub.publish(msg);
}

int main (int argc, char **argv)
{
  ros::init(argc, argv, "img_processing_node");
  
  img_process one;

  ros::spin();

  return 0;
}