#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

namespace nodelet_hello_world
{

class Hello : public nodelet::Nodelet
{
private:
    virtual void onInit()
    {
        ros::NodeHandle& private_nh = getPrivateNodeHandle();
        NODELET_DEBUG("Initialized the Nodelet");

        // Initialize publisher and subscriber
        pub = private_nh.advertise<sensor_msgs::Image>("image_out", 5);
        sub = private_nh.subscribe("image_in", 5, &Hello::callback, this);
    }

    void callback(const sensor_msgs::ImageConstPtr& input)
    {
        try
        {
            // Convert ROS image message to OpenCV image
            cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(input, sensor_msgs::image_encodings::BGR8);

            // Resize the image to 320x240
            cv::Mat resized_image;
            cv::resize(cv_ptr->image, resized_image, cv::Size(320, 240));

            // Convert the image to grayscale
            cv::Mat gray_image;
            cv::cvtColor(resized_image, gray_image, cv::COLOR_BGR2GRAY);

            // Convert OpenCV image back to ROS image message
            cv_bridge::CvImage out_msg;
            out_msg.header = input->header; // Same timestamp and tf frame as input image
            out_msg.encoding = sensor_msgs::image_encodings::MONO8;
            out_msg.image = gray_image;

            // Publish the image
            pub.publish(out_msg.toImageMsg());
        }
        catch (cv_bridge::Exception& e)
        {
            NODELET_ERROR("cv_bridge exception: %s", e.what());
        }
    }

    ros::Publisher pub;
    ros::Subscriber sub;
};

}

PLUGINLIB_EXPORT_CLASS(nodelet_hello_world::Hello, nodelet::Nodelet);
