#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#include <opencv2/highgui/highgui.hpp>

#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/image_encodings.h>

#include "std_msgs/String.h"

class imageTransportNode{
    public:
    imageTransportNode(ros::NodeHandle nh, std::string topic){
        this->nh = nh;
        
        image_transport::ImageTransport it(nh);

        this->sub = it.subscribe(topic, 1, &imageTransportNode::callback, this);
        this->pub = it.advertise("Compressed", 1);
    }

    void ImagePublisher(cv::Mat image){
        //TODO: Compress the image message into a PNG        
        sensor_msgs::ImagePtr image_msg = cv_bridge::CvImage(std_msgs::Header(), sensor_msgs::image_encodings::MONO16, 
        image).toImageMsg();

        ros::Rate loop_rate(5);
        while(nh.ok()){
            pub.publish(image_msg);
            ros::spinOnce();
            loop_rate.sleep();
        }
        
    }
    void callback(const sensor_msgs::ImageConstPtr& msg){
        cv_bridge::CvImagePtr cvImagePtr;
        try {
            // cvImagePtr = cv_bridge::toCvCopy(msg);
            cvImagePtr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO16);
        } 
        catch (cv_bridge::Exception &e) {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }
        cv::Mat img = cvImagePtr->image;
        ImagePublisher(img);
    }

    protected:
        ros::NodeHandle nh;
        image_transport::Publisher pub;
        image_transport::Subscriber sub;

};

int main(int argc, char** argv){
    ros::init(argc, argv, "transport_node");
    ros::NodeHandle nh("~");

    std::string name; 
    nh.param<std::string>("robotName", name, "$robot"); //default samoyed

    //topic to subscribe to
    std::string range_topic = "/" + name + "/img_node16/range_image";
    imageTransportNode imgT(nh, range_topic);

    ros::spin();

    return EXIT_SUCCESS;
}