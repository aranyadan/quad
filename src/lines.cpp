#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <queue>
#include <iostream>

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
  bool intersection(Point2f o1, Point2f p1, Point2f o2, Point2f p2,Point2f &r)
{
    Point2f x = o2 - o1;
    Point2f d1 = p1 - o1;
    Point2f d2 = p2 - o2;

    float cross = d1.x*d2.y - d1.y*d2.x;
    if (abs(cross) < 1e-8)
        return false;

    double t1 = (x.x * d2.y - x.y * d2.x)/cross;
    r = o1 + d1 * t1;
    return true;
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
    queue<Point> q1;
    queue<Point> q2;
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
        q1.push(pt1);
        q2.push(pt2);
        //cout<<pt1.x<<","<<pt1.y<<"\n";
    }
    //namedWindow( "Source", 1 );
    //imshow( "Source", img4 );
    
    for(int i=-1;i<=1;i++)
    {
      for(int j=-1;j<=1;j++)
      {
        img5.at<Vec3b>(img5.rows/2+i,img5.cols/2+j)[0]=255;
      }
    }
    


    /*double rho1=lines[0][0];
    double rho2=lines[1][0];
    double a1=cos(lines[0][1]);
    double b1=sin(lines[0][1]);
    double a2=cos(lines[1][1]);
    double b2=sin(lines[1][1]);
    Point pt1(cvRound(a1*rho1 + 1000*(-b1)),
                  cvRound(b1*rho1 + 1000*(a1)));
        Point pt2(cvRound(a1*rho1 - 1000*(-b1)),
                  cvRound(b1*rho1 - 1000*(a1)));
        Point pt3(cvRound(a2*rho2 + 1000*(-b2)),
                  cvRound(b2*rho2 + 1000*(a2)));
        Point pt4(cvRound(a2*rho2 - 1000*(-b2)),
                  cvRound(b2*rho2 - 1000*(a2)));
    Point pt5((pt1.x+pt3.x)/2,(pt1.y+pt3.y)/2);
    Point pt6((pt2.x+pt4.x)/2,(pt2.y+pt4.y)/2);
    line(img5,pt5,pt6,Scalar(0,255,0),1,8);
*/
    while(q1.empty()==false)
    {
      Point p1=q1.front();
      Point p2=q2.front();
      q1.pop();
      q2.pop();
      if(q1.empty())
        break;
      Point p3=q1.front();
      Point p4=q2.front();
      q1.pop();
      q2.pop();
      Point p5((p1.x+p3.x)/2,(p1.y+p3.y)/2);
      Point p6((p2.x+p4.x)/2,(p2.y+p4.y)/2);
      line(img5,p5,p6,Scalar(0,255,0),1,8);
    }
    /*for( size_t i = 0; i < 4; i++ )
    {
        queue<Point> q1;
    queue<Point> q2;
        float theta = lines[i][1];
        for(size_t j = 0; j < 4; j++)
        {
          if(lines[j][1]==theta|| lines[j][1]<theta+10/360*3.14||lines[j][1]>theta-10/360*3.14)
          {
            float rho = lines[i][0];
            
            double a = cos(theta), b = sin(theta);
            double x0 = a*rho, y0 = b*rho;
            Point pt1(cvRound(x0 + 1000*(-b)),
                  cvRound(y0 + 1000*(a)));
            Point pt2(cvRound(x0 - 1000*(-b)),
                  cvRound(y0 - 1000*(a)));
            q1.push(pt1);
            q2.push(pt2);
          }
        }
        Point p1(0,0);
        Point p2(0,0);
        int c=0;
          while(q1.empty()==false)
          {
            p1.x+=q1.front().x;
            p1.y+=q1.front().y;
            p2.x+=q2.front().x;
            p2.y+=q2.front().y;
            c++;
            q1.pop();
            q2.pop();
          }
        p1.x/=c;
        p1.y/=c;
        p2.x/=c;
        p2.y/=c;
        line( img5, p1, p2, Scalar(0,255,0), 1, 8 );
    }*/
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