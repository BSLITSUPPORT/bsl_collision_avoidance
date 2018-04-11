#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>

class SubscribeAndPublish {
    public: 
    SubscribeAndPublish(){
        tf2_ros::TransformListener tflistener(tfBuffer);
        pub = n.advertise<nav_msgs::OccupancyGrid>("OccGrid", 10);
        sub = n.subscribe("cloud_drop", 10, &SubscribeAndPublish::callback, this);
        
    }
    
    void callback(const sensor_msgs::PointCloud2& cloud){
        
        //Find Transform from laser into real world (might not work because i am looking up transform just after creating it        geometry_msgs::TransformStamped transformStamped;
        geometry_msgs::TransformStamped transformStamped;
        try{
          transformStamped = tfBuffer.lookupTransform("laser", "map", ros::Time(0));
        }
        catch (tf2::TransformException &ex) {
          ROS_WARN("%s",ex.what());
          return;
        }
        
        //Generate an empty OccuppancyGrid
        nav_msgs::OccupancyGrid myOccupancyGrid;
        myOccupancyGrid.header.stamp = cloud.header.stamp;
        myOccupancyGrid.header.frame_id = "map";
        myOccupancyGrid.info.map_load_time = cloud.header.stamp;
        myOccupancyGrid.info.resolution = 0.1;
        myOccupancyGrid.info.width = 22/myOccupancyGrid.info.resolution;
        myOccupancyGrid.info.height = 10/myOccupancyGrid.info.resolution;
        myOccupancyGrid.info.origin.position.x = -2;
        myOccupancyGrid.info.origin.position.y = -5;
        myOccupancyGrid.info.origin.position.z = 0;
        myOccupancyGrid.info.origin.orientation.x = 0;
        myOccupancyGrid.info.origin.orientation.y = 0;
        myOccupancyGrid.info.origin.orientation.z = 0;
        myOccupancyGrid.info.origin.orientation.w = 0;
        
        double start = ros::Time::now().toSec();
        
        sensor_msgs::PointCloud2 newCloud;
        tf2::doTransform(cloud, newCloud, transformStamped);
        
        //sensor_msgs::PointCloud2 newCloud;
        //pcl_ros::transformPointCloud("/map", cloud, newCloud, listener);
        
        myOccupancyGrid.data.assign(22000, 0);

        double duration = (ros::Time::now().toSec() - start);
        
        ROS_INFO_STREAM("yay2 " << duration);
        
        pub.publish(myOccupancyGrid);
    }
    
    public:
    ros::NodeHandle n;
    ros::Publisher pub;
    ros::Subscriber sub;
    tf2_ros::Buffer tfBuffer;
    
};

int main(int argc, char **argv){
    ros::init(argc, argv, "occupancy_grid_creation");
    
    SubscribeAndPublish SAPobject;
    
    ros::spin();

    return 0;
}

