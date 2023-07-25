// #include "ouster_ros/os_ros.h"
// clang-format on

#include <pcl/conversions.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>

#include <sensor_msgs/image_encodings.h>

#include "std_msgs/String.h"

#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include "ouster/client.h"
#include "ouster/image_processing.h"
#include "ouster/types.h"
#include "ouster_ros/OSConfigSrv.h"
#include "ouster_ros/ros.h"

namespace sensor = ouster::sensor;
namespace viz = ouster::viz;

using pixel_type = uint16_t;
constexpr size_t bit_depth = 8 * sizeof(pixel_type); //sizeof(pixel_type) = 2 for 16 bit
const size_t pixel_value_max = std::numeric_limits<pixel_type>::max();

sensor_msgs::Image range_image;
sensor_msgs::Image ambient_image;
sensor_msgs::Image intensity_image;

int main(int argc, char** argv){
    ros::init(argc, argv, "img_node");
    ros::NodeHandle nh("~");

    // double denom;
    // nh.param("range", denom, 200.0);
    // double range_multiplier = 1.0 / denom;  

    //Default values:
        // std::vector<int> offset_default = {9, 6, 3, 0, 9, 6,
        //                                 3, 0, 9, 6, 3, 0,
        //                                 9, 6, 3, 0, 9, 6,
        //                                 3, 0, 9, 6, 3, 1,
        //                                 9, 6, 3, 1, 9, 6,
        //                                 4, 1, 9, 6, 3, 1,
        //                                 9, 7, 4, 1, 10, 7,
        //                                 4, 1, 10, 7, 4, 1, 
        //                                 10, 7, 4, 1, 10, 7, 
        //                                 4, 1, 10, 7, 4, 1,
        //                                 10, 7, 4, 1

        // };
        // size_t defaultW = 64; 
        // size_t defaultH = 512;

    size_t FinalH = 0;
    size_t FinalW = 0;
    std::vector<int> Final_offset_default = {0,0,0,0,0,0,0};

    ouster_ros::OSConfigSrv cfg{};
    auto client = nh.serviceClient<ouster_ros::OSConfigSrv>("os_config");
    bool LoadFromService = false; 

    nh.param<bool>("LoadFromService", LoadFromService, false);

    //UNCOMMENT IF RUNNING FROM A ROBOT, NOT A BAG
    if(LoadFromService){
        ROS_WARN("Getting Height, width, and pixel offset from os_config service");
        client.waitForExistence();
        if (!client.call(cfg)) {

            ROS_ERROR("Calling os config service failed");

            //Use rosParam intead if this happens

            return EXIT_FAILURE;
        }
        else{
            ROS_WARN("Getting info from a client");
            auto info = sensor::parse_metadata(cfg.response.metadata);
            
            FinalH = info.format.pixels_per_column;
            FinalW = info.format.columns_per_frame;
            Final_offset_default = info.format.pixel_shift_by_row;

        }
    }
    else{
        if(ros::param::has("~px_offset") && ros::param::has("~pixels_per_column") && ros::param::has("~pixels_per_row")){
            int tempH, tempW;
            std::vector<int> offset_temp;

            nh.param<int>("pixels_per_column", tempH, 64);
            nh.param<int>("pixels_per_row", tempW, 512);


            ros::param::get("~px_offset", offset_temp);

            Final_offset_default = offset_temp;
            FinalW = (size_t) tempW;
            FinalH = (size_t) tempH;

        }
        else{
            ROS_ERROR("COULDN'T GET ROSPARAMS: \npx_offset\npixels_per_column\npixels_per_row");
            return EXIT_FAILURE;
        }
    }


    const std::vector<int> px_offset = Final_offset_default;
    size_t W =  FinalW;
    size_t H = FinalH;

    ros::Publisher range_image_pub =
        nh.advertise<sensor_msgs::Image>("range_image", 100);
    ros::Publisher ambient_image_pub =
        nh.advertise<sensor_msgs::Image>("ambient_image", 100);
    ros::Publisher intensity_image_pub =
        nh.advertise<sensor_msgs::Image>("intensity_image", 100);

    ouster_ros::Cloud cloud{};

    viz::AutoExposure ambient_ae, intensity_ae;
    viz::BeamUniformityCorrector ambient_buc;

    std::stringstream encoding_ss;
    encoding_ss << "mono" << bit_depth;
    std::string encoding = encoding_ss.str();
    //OG step = (W*H*Bit_depth)/8
     range_image.width = W;
        range_image.height = H;
        range_image.step = (W * bit_depth)/8;
        // ROS_INFO("range_image.step: %d", range_image.step);
        // ROS_INFO("bit depth: %ld", bit_depth);
        //range_image.step = W;
        range_image.encoding = encoding;
        range_image.data.resize(W * H * bit_depth /
                                (8 * sizeof(*range_image.data.data())));
        
        // ROS_INFO("range_image.data.data().size(): %ld", range_image.data.size());


        ambient_image.width = W;
        ambient_image.height = H;
        ambient_image.step = (W * bit_depth)/8;
        //ambient_image.step = W;
        ambient_image.encoding = encoding;
        ambient_image.data.resize(W * H * bit_depth /
                                  (8 * sizeof(*ambient_image.data.data())));


        intensity_image.width = W;
        intensity_image.height = H;
        intensity_image.step = (W * bit_depth)/8;
        //intensity_image.step = W;
        intensity_image.encoding = encoding;
        intensity_image.data.resize(W * H * bit_depth /
                                    (8 * sizeof(*intensity_image.data.data())));
    auto cloud_handler = [&](const sensor_msgs::PointCloud2::ConstPtr& m) {
        // only publish if somebody is listening
        if (range_image_pub.getNumSubscribers() == 0
            && ambient_image_pub.getNumSubscribers() == 0
            && intensity_image_pub.getNumSubscribers() == 0) {
            return;
        }

        pcl::fromROSMsg(*m, cloud);

        // sensor_msgs::Image range_image;
        // sensor_msgs::Image ambient_image;
        // sensor_msgs::Image intensity_image;

        range_image.header.stamp = m->header.stamp;

        ambient_image.header.stamp = m->header.stamp;

        intensity_image.header.stamp = m->header.stamp;
        auto range_image_map = Eigen::Map<ouster::img_t<pixel_type>>(
            (pixel_type*)range_image.data.data(), H, W);
        ouster::img_t<double> ambient_image_eigen(H, W);
        ouster::img_t<double> intensity_image_eigen(H, W);

        for (size_t u = 0; u < H; u++) {
            for (size_t v = 0; v < W; v++) {
                const size_t vv = (v + W - px_offset[u]) % W;
                const auto& pt = cloud[u * W + vv];

                // 16 bit img: use 4mm resolution and throw out returns >
                // 260m
                auto r = (pt.range + 0b10) >> 2;
                range_image_map(u, v) = r > pixel_value_max ? 0 : r;
                ambient_image_eigen(u, v) = pt.ambient;
                intensity_image_eigen(u, v) = pt.intensity;
            }
        }

        ambient_buc(ambient_image_eigen);
        ambient_ae(ambient_image_eigen);
        intensity_ae(intensity_image_eigen);
        ambient_image_eigen = ambient_image_eigen.sqrt();
        intensity_image_eigen = intensity_image_eigen.sqrt();
        for (size_t u = 0; u < H; u++) {
            for (size_t v = 0; v < W; v++) {
                reinterpret_cast<pixel_type*>(
                    ambient_image.data.data())[u * W + v] =
                    ambient_image_eigen(u, v) * pixel_value_max;
                reinterpret_cast<pixel_type*>(
                    intensity_image.data.data())[u * W + v] =
                    intensity_image_eigen(u, v) * pixel_value_max;
            }
        }


        range_image_pub.publish(range_image);
        ambient_image_pub.publish(ambient_image);
        intensity_image_pub.publish(intensity_image);
    };

    auto pc_sub =
        nh.subscribe<sensor_msgs::PointCloud2>("points", 500, cloud_handler);

    ros::spin();
    return EXIT_SUCCESS;
}
