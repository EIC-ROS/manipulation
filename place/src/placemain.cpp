#include <ros/ros.h>
#include <cmath>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <tf/transform_listener.h>
#include <pickplace_include/pickplace.h>
#include <pickplace_include/placecond.h>
#include <pickplace_include/caleddim.h>

class place{
    private:
        tf2_ros::StaticTransformBroadcaster bcplace;
        tf2_ros::StaticTransformBroadcaster bcplaceapproach;
        tf2_ros::StaticTransformBroadcaster bcplacesaferetreat;

        ros::ServiceServer service;
        tf::StampedTransform tfStamped;

        double pi = M_PI;
        double approach = 0.05;
        double roll, pitch, yaw;
        double safe_retreat = 0.1;
        double retreat = 0.05;
        double up = 0.02;
        double dimensionx, dimensiony, dimensionz;

        bool table;

        ros::ServiceClient client;
        ros::ServiceClient client2;
        moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
        pickplace_include::placecond srv;
        pickplace_include::caleddim srv2;
    
    public:
        place(ros::NodeHandle *nodehan){
            service = nodehan->advertiseService("placeit", &place::callback, this);
            client = nodehan->serviceClient<pickplace_include::placecond>("place");
            client2 = nodehan->serviceClient<pickplace_include::caleddim>("getdim");
        }

        bool callback(pickplace_include::pickplace::Request& req, pickplace_include::pickplace::Response& res){
            if(client2.call(srv2)){
                dimensionx = srv2.response.dimension.x;
                dimensiony = srv2.response.dimension.y;
                dimensionz = srv2.response.dimension.z;
            }
            else{
                ROS_ERROR("Service Failed");
            }
            createCollisionObjectFrame(bcplace, req.posercam.header.frame_id, "place_place", req.posercam.pose.position.x, req.posercam.pose.position.y,
                                       req.posercam.pose.position.z + up + dimensionz/2, req.posercam.pose.orientation.x, req.posercam.pose.orientation.y,
                                       req.posercam.pose.orientation.z, req.posercam.pose.orientation.w);
            tf::TransformListener listener2;
            table = req.check;
            ROS_WARN_STREAM(table);
            try{
                ros::Time now = ros::Time::now();
                listener2.waitForTransform("cr3_base_link", "place_place", now, ros::Duration(0.3));

                if(table){
                    createCollisionObjectFrameRPY(bcplaceapproach, "place_place", "place_approach", -retreat, 0, 0, 0, 0, 0);
                    createCollisionObjectFrameRPY(bcplacesaferetreat, "place_place", "place_safe_retreat", -safe_retreat, 0, 0, 0, 0, 0);
                    srv.request.states = 1;
                }
                else{
                    createCollisionObjectFrameRPY(bcplaceapproach, "place_place", "place_approach", -approach, 0, 0, 0, 0, 0);
                    createCollisionObjectFrameRPY(bcplacesaferetreat, "place_place", "place_safe_retreat", -safe_retreat, 0, 0, 0, 0, 0);
                    srv.request.states = 2;
                }
                tf::TransformListener listener3;
                try{
                ros::Time now = ros::Time::now();
                listener3.waitForTransform("cr3_base_link", "place_approach", now, ros::Duration(0.3));
                if(client.call(srv)){
                    if(srv.response.success == true){
                        res.graspsuccess = true;
                    }
                    else{
                        res.graspsuccess = false;
                    }
                }
                else{
                    ROS_ERROR("Service Failed");
                }
                }
                catch (tf::TransformException ex){
                    ROS_ERROR("%s",ex.what());
                }
            }
            catch (tf::TransformException ex){
                ROS_ERROR("%s",ex.what());
            }
            return true;
        }

        void createCollisionObjectFrame(tf2_ros::StaticTransformBroadcaster& broadcaster,
                                const std::string& parent_frame,
                                const std::string& child_frame,
                                const double x, const double y, const double z,
                                const double qx, const double qy, const double qz, const double qw)
        {
        ROS_INFO("Main Object Frame Created");
        geometry_msgs::TransformStamped transform;
        transform.header.stamp = ros::Time::now();
        transform.header.frame_id = parent_frame;
        transform.child_frame_id = child_frame;
        transform.transform.translation.x = x;
        transform.transform.translation.y = y;
        transform.transform.translation.z = z;

        transform.transform.rotation.x = qx;
        transform.transform.rotation.y = qy;
        transform.transform.rotation.z = qz;
        transform.transform.rotation.w = qw;

        broadcaster.sendTransform(transform);
        return;
        }

        void createCollisionObjectFrameRPY(tf2_ros::StaticTransformBroadcaster& broadcaster,
                                const std::string& parent_frame,
                                const std::string& child_frame,
                                const double x, const double y, const double z,
                                const double roll, const double pitch, const double yaw)
        {
        geometry_msgs::TransformStamped transform;
        transform.header.stamp = ros::Time::now();
        transform.header.frame_id = parent_frame;
        transform.child_frame_id = child_frame;
        transform.transform.translation.x = x;
        transform.transform.translation.y = y;
        transform.transform.translation.z = z;
        tf2::Quaternion quat;
        quat.setRPY(roll, pitch, yaw);
        transform.transform.rotation = tf2::toMsg(quat);
        broadcaster.sendTransform(transform);
        return;
        }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "preplaccenode");
    ros::NodeHandle n;
    place k = place(&n);
    ros::spin();
}
