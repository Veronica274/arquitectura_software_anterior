
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/Float32.h>

class FindYellowGoal
{
public:
  FindYellowGoal(): it_(nh_)
  {
    image_sub_ = it_.subscribe("/hsv/image_filtered", 1, &FindYellowGoal::imageCb, this);
  }

  void imageCb(const sensor_msgs::Image::ConstPtr& msg)
  {
    cv_bridge::CvImagePtr cv_ptr, cv_imageout;

    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);

    int height = cv_ptr->image.rows;
    int width = cv_ptr->image.cols;
    int step = cv_ptr->image.step;
    int channels = 3;  // RGB

    int x = 0;
    int y = 0;
    int counter = 0;
    for (int i=0; i < height; i++ ){
      for (int j=0; j < width; j++ )
      {
        int posdata = i * step + j * channels;

        if (cv_ptr->image.data[posdata] != 0 && cv_ptr->image.data[posdata + 1] != 0 && cv_ptr->image.data[posdata + 2] != 0)
        {
            x += i;
            y += j;
            counter++;
        }
      }
    }
    if (counter > 0){
        ROS_INFO("Goal at %d %d\n", x/counter, y/counter);
    } else {
        ROS_INFO("No goal found");
    }

  }

private:
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "find_yellow_goal");
  FindYellowGoal fyg;
  ros::spin();
  return 0;
}
