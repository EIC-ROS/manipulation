#include <ros/ros.h>
#include <cmath>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/PoseStamped.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/Pose.h>
#include <tf/transform_listener.h>
#include <pickplace_include/dimension.h>
#include <pickplace_include/tfcount.h>
#include <pickplace_include/caleddim.h>

class place{
    private:
        ros::ServiceServer service;
        ros::ServiceServer service2;
        ros::ServiceServer service3;
        tf::StampedTransform tfStamped;

        moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

        double pi = M_PI;
        double approach = 0.05;
        double roll, pitch, yaw;
        double up = 0.02;
        double dimensionx, dimensiony, dimensionz;
        double graspdimensionx, graspdimensiony, graspdimensionz;

        bool getdim, gettf;

        ros::ServiceClient client;
    
    public:
        place(ros::NodeHandle *nodehan){
            service = nodehan->advertiseService("getdim", &place::callback, this);
            service2 = nodehan->advertiseService("setdim", &place::callback2, this);
            service3 = nodehan->advertiseService("setplace", &place::callback3, this);
        }

        bool callback(pickplace_include::caleddim::Request& req, pickplace_include::caleddim::Response& res){
            res.dimension.x = graspdimensionx;
            res.dimension.y = graspdimensiony;
            res.dimension.z = graspdimensionz;
            return true;
        }

        bool callback3(pickplace_include::tfcount::Request& req, pickplace_include::tfcount::Response& res){
            gettf = true;
            tf::TransformListener listener;
            try{
                ros::Time now = ros::Time::now();
                listener.waitForTransform("obj", req.frame, now, ros::Duration(0.3));
                listener.lookupTransform("obj", req.frame, ros::Time(0), tfStamped);
            }
            catch (tf::TransformException ex){
                ROS_ERROR("%s",ex.what());
            }

            tf::Quaternion objor = tfStamped.getRotation();
            tf::Quaternion q(objor[0], objor[1], objor[2], objor[3]);
            tf::Matrix3x3 m(q);
            m.getRPY(roll, pitch, yaw);
            
            setdimension(roll , pitch, yaw, dimensionx, dimensiony, dimensionz);
            return true;
        }

        bool callback2(pickplace_include::dimension::Request& req, pickplace_include::dimension::Response& res){
            getdim = true;
            dimensionx = req.dimension.x;
            dimensiony = req.dimension.y;
            dimensionz = req.dimension.z;
            res.done = true;
            return true;
        }


        void setdimension(const double r, const double p, const double y,
                          const double dx, const double dy, const double dz)
        {
            if(roundd(fabs(p)) == 0){
                if(roundd(fabs(y)) == roundd(pi/2)){
                    graspdimensionx = dy;
                    graspdimensiony = dx;
                    graspdimensionz = dz;
                }
                else{
                    graspdimensionx = dx;
                    graspdimensiony = dy;
                    graspdimensionz = dz;
                }
            }
            else if(roundd(fabs(p)) == roundd(pi/2)){
                if(roundd(fabs(r)) == roundd(pi/2)){
                    graspdimensionx = dz;
                    graspdimensiony = dx;
                    graspdimensionz = dy;
                }
                else{
                    graspdimensionx = dz;
                    graspdimensiony = dy;
                    graspdimensionz = dx;
                }
            }
            else{
                graspdimensionx = dx;
                graspdimensiony = dy;
                graspdimensionz = dz;
            }
            ROS_WARN_STREAM(graspdimensionx);
            ROS_WARN_STREAM(graspdimensiony);
            ROS_WARN_STREAM(graspdimensionz);
            return;
        }

        double roundd(double f){
            double v = (int)(f * 100 + .5);
            return (double)v / 100;
        }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "dimensioncalculator");
    ros::NodeHandle n;
    place k = place(&n);
    ros::spin();
}
