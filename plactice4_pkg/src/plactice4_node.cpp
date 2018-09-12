// done
// 어떻게 작동하는지 알아보고 contours 함수 조작해보기
// 라벨링 테스트
// to_string 함수 사용법 찾아야 할 듯(cmakelists에서 컴파일 버전 해결)
// morp 예제 작성 및 라벨링 범위 제한(정사각형 모양, 하한 픽셀 범위)
// 라벨링 한 범위 roi 지정 및 잘라서 findcontours
// hsv, dilate 트랙바
// 트랙바 안 먹는 것은 namedWindow, waitkey 함수 확인해 볼 것
// hsv2gray 사용해서 템플릿 매칭까지 시도
// 외곽선 추출 성공적이나 사용할지 의문
// 트랙바 제거 및 빨강색 판별 후 표지판 ROI로 특이점 매칭까지 실행 완료
// 비디오 파일 읽어 처리 예제 작성
// 임의의 메시지 파일 생성 후 퍼블리싱 확인 -> 스테이트를 숫자로 넘겨주는 방법?

// should <180715 까지>
// 적당한 비디오 파일 혹은 웹캠으로 매칭 시도
// 표지판 검출하여 주행 메시지 보내는 것
// FSM 만들기

// should <180722 까지>
// pcl 공부해서 rviz에 차선을 장애물로 인식하게 하기
// -> pointcloud2 메시지에 값을 입력해주는 걸로 가능할 듯 하다 파폭 북마크 참조
// -> 단순한 호기심에서 pointcloud 메시지를 이미지로 출력해보면 어떻게 되는지 한번 확인해보자

// should <180729 까지>
// 교수님께 중간점검 받고 8월 지도 받기

#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <opencv2/xfeatures2d.hpp>
#include <opencv2/features2d.hpp>
// #include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <string>
#include <math.h>

#include "plactice4_pkg/state.h"

using namespace cv;
using namespace std;
using namespace cv::xfeatures2d;

const int MAX_FEATURES = 100;
// const int TRACK_BAR_FLAG = 0;
int check = 0;

string road20 = "/home/moon/catkin_ws/src/picture/road/road20.jpg";
string road30 = "/home/moon/catkin_ws/src/picture/road/road30.jpg";
string roadstop = "/home/moon/catkin_ws/src/picture/road/roadstop.jpg";
string vedio = "/home/moon/catkin_ws/src/picture/vedio.mp4";

string sign20 = "/home/moon/catkin_ws/src/picture/sign/20.jpg";
string sign30 = "/home/moon/catkin_ws/src/picture/sign/30.jpg";
string stopsign = "/home/moon/catkin_ws/src/picture/sign/stop.jpg";

static void void_callback(int, void* ){}

void matching(Mat &im1Gray, Mat &im2)
{
  vector<KeyPoint> keypoints1, keypoints2;
  Mat descriptors1, descriptors2;
  Mat im2Gray;
  cvtColor(im2, im2Gray, COLOR_BGR2GRAY);

  resize(im2Gray, im2Gray, cv::Size(im1Gray.cols, im1Gray.rows), 0, 0, CV_INTER_AREA);

  Ptr<Feature2D> orb = ORB::create(MAX_FEATURES);
  orb->detectAndCompute(im1Gray, noArray(), keypoints1, descriptors1);
  orb->detectAndCompute(im2Gray, noArray(), keypoints2, descriptors2);

  vector<vector<DMatch>> matches;
  BFMatcher matcher(NORM_HAMMING);
  // Ptr<DescriptorMatcher> matcher = DescriptorMatcher::create("BruteForce-Hamming");
  // matcher->match(descriptors1, descriptors2, matches);
  int k = 2;
  matcher.knnMatch(descriptors1, descriptors2, matches, k);

  /*
  // 코어 덤프 해결
  double minDist, maxDist;
  minDist = maxDist = matches[0].distance;
  for(int i = 0; i < matches.size(); i++)
  {
    double dist = matches[i].distance;
    if(dist < minDist) minDist = dist;
    if(dist > maxDist) maxDist = dist;
  }
  */

  vector<DMatch> goodMatches;
  // double fTh = 4 * minDist;
  float nndrRatio = 0.7f;
  for(int i = 0; i < matches.size(); i++)
  {
    if(matches.at(i).size() == 2 && matches.at(i).at(0).distance <= nndrRatio * matches.at(i).at(1).distance)
      goodMatches.push_back(matches[i][0]);
  }
  // if(matches[i].distance <= max(fTh, 0.02)) goodMatches.push_back(matches[i]);

  Mat imMatches;
  drawMatches(im1Gray, keypoints1, im2Gray, keypoints2, goodMatches, imMatches, Scalar(0, 255, 0), Scalar(0, 0, 255), vector<char>(), 2);
  imshow("result", imMatches);

  /*
  int Dist2 = 0, good = 0;
  int x1, x2, y1, y2;
  Point point1, point2;

  for(int i = 0; i < goodMatches.size(); i++)
  {
    point1 = keypoints1[goodMatches[i].queryIdx].pt;
    point2 = keypoints2[goodMatches[i].trainIdx].pt;
    x1 = point1.x;
    x2 = point2.x;
    y1 = point1.y;
    y2 = point2.y;

    int Dist1 = sqrt(pow(x1 - x2, 2) + pow(y1 - y2, 2));

    if(Dist2 == 0) Dist2 = Dist1;
    else if(Dist1 >= (Dist2 * 0.95) && Dist1 <= (Dist2 * 1.05)) good++;
  }
  */
 
  check++;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "plactice4_test");
  ros::NodeHandle n;
  ros::Publisher msg_pub = n.advertise<plactice4_pkg::state>("/state_pub_test", 1000);
  plactice4_pkg::state msg;

  int minHue = 0, maxHue = 30, 
  minSat = 230, maxSat = 255, 
  minVal = 220, maxVal = 255;

  Mat src, src_gray, im2;

  src = imread(road20);
  im2 = imread(stopsign);
  
  // resize(src, src, cv::Size(((src.cols) * 0.5), ((src.rows) * 0.5)), 0, 0, CV_INTER_AREA);

  /*
  if (src.empty())
  {
    cerr << "No image supplied ..." << endl;
    return -1;
  }
  */

  /*VideoCapture cap(vedio);

  if(!cap.isOpened())
  {
    cout << "Error opening video stream or file" << endl;
    return -1;
  }
  double fps = cap.get(CV_CAP_PROP_FPS);*/

  while(1)
  {
    // cap >> src;

    /*
    if(TRACK_BAR_FLAG)
    {
      namedWindow("filtered_image");
      createTrackbar("Hmin value", "filtered_image", &minHue, 180, void_callback);
      setTrackbarPos("Hmin value", "filtered_image", 0);
      createTrackbar("Hmax value", "filtered_image", &maxHue, 180, void_callback);
      setTrackbarPos("Hmax value", "filtered_image", 30);

      createTrackbar("Smin value", "filtered_image", &minSat, 255, void_callback);
      setTrackbarPos("Smin value", "filtered_image", 230);
      createTrackbar("Smax value", "filtered_image", &maxSat, 255, void_callback);
      setTrackbarPos("Smax value", "filtered_image", 255);

      createTrackbar("Vmin value", "filtered_image", &minVal, 255, void_callback);
      setTrackbarPos("Vmin value", "filtered_image", 220);
      createTrackbar("Vmax value", "filtered_image", &maxVal, 255, void_callback);
      setTrackbarPos("Vmax value", "filtered_image", 255);

      minHue = getTrackbarPos("Hmin value", "filtered_image");
      maxHue = getTrackbarPos("Hmax value", "filtered_image");
      minSat = getTrackbarPos("Smin value", "filtered_image");
      maxSat = getTrackbarPos("Smax value", "filtered_image");
      minVal = getTrackbarPos("Vmin value", "filtered_image");
      maxVal = getTrackbarPos("Vmax value", "filtered_image");
    }
    */

    // 색 분리 작업
    Mat src_hsv, src_blur, src_gray;
  
    cvtColor(src, src_hsv, COLOR_BGR2HSV);
    inRange(src_hsv, Scalar(minHue, minSat, minVal), Scalar(maxHue, maxSat, maxVal), src_hsv);
    blur(src_hsv, src_blur, Size(3, 3));

    // inRange로 마스크 씌우는게 훨씬 효과적

    cvtColor(src, src_gray, COLOR_BGR2GRAY);
    
    /*
    // Mat element5(5, 5, CV_8U, cv::Scalar(1));
    // morphologyEx(src_gray, src_gray, MORPH_CLOSE, element5);
    // morphologyEx(src_threshold, src_threshold, MORPH_OPEN, element5);
    */

    imshow("Source_Img", src);

    /*
    // vector<vector<Point>> contours; // 외곽선 추적이 필요한가? 생각해보자
    // vector<Vec4i> hierarchy;
    // vector<vector<Point>> contours_poly(contours.size());
    // vector<Rect> boundRect(contours.size());

    // findContours(src_threshold, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE, Point(0, 0));
    // Mat drawing = Mat::zeros(src_threshold.size(), CV_8UC3);
    // for(int i = 0; i < contours.size(); i++)
    //   drawContours(drawing, contours, i, Scalar(255, 255, 255), 2, 8, hierarchy, 0, Point()); // 최외곽선 그리는 것 까지 완료, 좌표 저장후 roi따기
    */

    Mat img_labels, stats, centroids;
    int numOfLables = connectedComponentsWithStats(src_hsv, img_labels, stats, centroids, 8, CV_32S);

    Mat src_labels;
    src_labels = src.clone();

    //라벨링 된 이미지에 각각 직사각형으로 둘러싸기
    for (int j = 1; j < numOfLables; j++)
    {
      int area = stats.at<int>(j, CC_STAT_AREA);
      int left = stats.at<int>(j, CC_STAT_LEFT);
      int top  = stats.at<int>(j, CC_STAT_TOP);
      int width = stats.at<int>(j, CC_STAT_WIDTH);
      int height  = stats.at<int>(j, CC_STAT_HEIGHT);
      if(width >= 50 && height >= 50 && (width / height) >= 0.999 && (width / height) <= 1.001)
      {
        src_gray = src_gray(Rect(left, top, width, height));
        rectangle(src_labels, Point(left,top), Point(left+width,top+height), Scalar(0, 0, 255), 1 );
        putText(src_labels, to_string(j), Point(left+20,top+20), FONT_HERSHEY_SIMPLEX, 1, Scalar(0, 0, 255), 2);
      }
      else
      {
        rectangle(src_labels, Point(left,top), Point(left+width,top+height), Scalar(255, 0, 0), 1 );
        putText(src_labels, to_string(j), Point(left+20,top+20), FONT_HERSHEY_SIMPLEX, 1, Scalar(255, 0, 0), 2);
      }
    }
    imshow("src_roi_labels", src_labels);

    // roi이미지가 아닌 비교할 이미지를 roi이미지 크기로 변환해서 매칭하면 되지 않을까?
    // 에러코드 (-215)는 imread에서 발생하는 것 같음 imread 함수나 이미지 경로 확인 할 것

    matching(src_gray, im2);
    
    if(check == 0) ;
    else if(check == 1) 
    {
      im2 = imread(sign30);
      msg.data = "30 limit sign";
    }
    else if(check == 2)
    {
      im2 = imread(stopsign);
      msg.data = "stop sign";
    } 
    else
    {
      im2 = imread(sign20);
      msg.data = "20 limit sign";
      check = 0;
    }

    msg_pub.publish(msg);
 
    if(waitKey(0) == 27) break;
  }
  // cout << fps << endl;
  // cap.release();
  destroyAllWindows();
  ros::spin();
  return 0;
}