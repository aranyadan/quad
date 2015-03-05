#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>

static const std::string OPENCV_WINDOW = "Image window";
using namespace cv;
using namespace std;
int l=121,u=75;
class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;
  
public:
  ImageConverter()
    : it_(nh_)
  {
    // Subscrive to input video feed and publish output video feed
    image_sub_ = it_.subscribe("/ardrone/image_raw", 1, 
      &ImageConverter::imageCb, this);
    image_pub_ = it_.advertise("/image_converter/output_video", 1);

    cv::namedWindow(OPENCV_WINDOW);
  }

  ~ImageConverter()
  {
    cv::destroyWindow(OPENCV_WINDOW);
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

    // Draw an example circle on the video stream
    //if (cv_ptr->image.rows > 60 && cv_ptr->image.cols > 60)
    //  cv::circle(cv_ptr->image, cv::Point(50, 50), 10, CV_RGB(255,0,0));
    int thresh = 128;
    //int l = 0, u = 0;
    Mat img3(cv_ptr->image.rows, cv_ptr->image.cols, CV_8UC1);
    Mat img2 = cv_ptr->image;
    cvtColor(img2,img2, CV_BGR2GRAY);
    //namedWindow("normalwindow", CV_WINDOW_FULLSCREEN);
    namedWindow("Canny", CV_WINDOW_FULLSCREEN);
    //createTrackbar("thresh", "normalwindow", &thresh, 255);
    createTrackbar("upper","Canny",&u,255);
    createTrackbar("lower", "Canny", &l, 255);
    // Update GUI Window
    Canny(img2, img3, l, u);
    imshow("Canny", img3);
    Mat img4=img3;
    Mat img5(cv_ptr->image.rows, cv_ptr->image.cols, CV_8UC1);
    cvtColor( img4, img5, CV_GRAY2BGR );
    vector<Vec2f> lines;
    HoughLines( img4, lines, 1, CV_PI/180, 100 );

    for( size_t i = 0; i < lines.size(); i++ )
    {
        float rho = lines[i][0];
        float theta = lines[i][1];
        double a = cos(theta), b = sin(theta);
        double x0 = a*rho, y0 = b*rho;
        Point pt1(cvRound(x0 + 1000*(-b)),
                  cvRound(y0 + 1000*(a)));
        Point pt2(cvRound(x0 - 1000*(-b)),
                  cvRound(y0 - 1000*(a)));
        line( img5, pt1, pt2, Scalar(0,0,255), 3, 8 );
    }
    //namedWindow( "Source", 1 );
    //imshow( "Source", img4 );

    namedWindow( "Detected Lines", 1 );
    imshow( "Detected Lines", img5 );
    //cv::imshow(OPENCV_WINDOW, cv_ptr->image);
    cv::waitKey(3);
    
    // Output modified video stream
    image_pub_.publish(cv_ptr->toImageMsg());
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_converter");
  ImageConverter ic;
  ros::spin();
  return 0;
}
