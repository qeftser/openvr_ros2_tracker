
#include <ctime>
#include <cmath>
#include <functional>
#include <thread>

#include <openvr.h>

#include <rclcpp/rclcpp.hpp>

#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "builtin_interfaces/msg/time.hpp"
#include "tf2/LinearMath/Quaternion.h"

/*
 * ROS2 node for interfacing with steam vr and registering position and
 * velocity data from the vive tracker. This node functions as a simple
 * wrapper around a simple openvr instance. The associated launch file
 * will launch steam vr given steam is installed, and given steam vr
 * and the linux runtime sniper are installed on steam.
 */

/* Note that this node and the associated launch file assume ubuntu 22.04
 * is being used. If another operating system or non-default install paths
 * for steam are used, the paths in the launch file may have to be edited
 */

using std::placeholders::_1;

class OpenVRTrackerNode : public rclcpp::Node {
public:

   OpenVRTrackerNode() : Node("openvr_tracker") {

      /* the topic to publish odometry on */
      this->declare_parameter("topic","openvr_tracker/odom");
      /* the topic to publish visualization output on. This
       * topic will only be active if visualization is set
       * to true */
      this->declare_parameter("visual_topic","openvr_tracker/visual");
      /* controls whether or not to publish visualization messages */
      this->declare_parameter("visualization",false);
      /* controls how long we wait after publishing data before going
       * back to try and publish it again. You can poll as fast as
       * you want practically with these systems, so a limiter is
       * nice. A value of zero will result in no delay. Note the value
       * is in milliseconds                                            */
      this->declare_parameter("poll_delay",25);

      /* odometry publisher */
      odometry_out = this->create_publisher<nav_msgs::msg::Odometry>(
            this->get_parameter("topic").as_string(), 10);

      /* welcome the user. We are all nice here */
      RCLCPP_INFO(this->get_logger(),"Hello and welcome...");

      /* collect poll delay */
      poll_delay = this->get_parameter("poll_delay").as_int();

      /* decide whether or not to publish visual data given the 
       * visualization parameter                               */
      if (this->get_parameter("visualization").as_bool()) {
         /* visualization publisher */
         markers_out = this->create_publisher<visualization_msgs::msg::Marker>(
               this->get_parameter("visual_topic").as_string(), 10);

         /* helper variable for run_process_visual */
         id_num = 0;

         run_process_visual();
      }
      else
         run_process();

   }

private:

   /* publishers */
   rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry_out;
   rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr markers_out;

   /* for markers */
   uint id_num;

   /* capture the poll delay on startup and do not deviate from it */
   int poll_delay;

   /* this is an infinite loop which continously polls for input from the vive
    * node and publishes any input discovered to the associated odometry topic */
   void run_process(void) {

      /* how many times will we try to connect to open vr */
      int remaining_attempts = 100;

retry:
      /* handle to all open vr functionalities */
      vr::IVRSystem * vr_pointer = NULL;

      vr::EVRInitError eError = vr::VRInitError_None;

      /* get the open vr pointer */
      vr_pointer = vr::VR_Init(&eError,vr::VRApplication_Background);

      /* if we cannont get the pointer, none of the remaining code in
       * this process will work. As a result of this, we will try again
       * for about 5 minutes before giving up                           */
      if (eError != vr::VRInitError_None) {
         --remaining_attempts;
         if (remaining_attempts < 0) {
            RCLCPP_INFO(this->get_logger(),"100 attempts to sync have failed, exiting");
            return;
         }
         RCLCPP_INFO(this->get_logger(),"Failed to find steamvr instance, retrying in 3 seconds");
         std::this_thread::sleep_for(std::chrono::seconds(3));
         goto retry;
      }

      RCLCPP_INFO(this->get_logger(),"OpenVR instance obtained");

      /* our processing loop */
      while (true) {
         /* there are 64 possible devices that can connect to an
          * openvr instance. We do not know which ones we want,
          * so we will loop through all of them                 */
         for (int i = 0; i < 64; ++i) {

            /* useful variables for when we get data */
            vr::TrackedDevicePose_t trackedDevicePose;
            vr::VRControllerState_t controllerState;

            /* get the ith device */
            vr::ETrackedDeviceClass c = vr_pointer->GetTrackedDeviceClass(i);

            /* check if there is currently a device active on this connection */
            if (!vr_pointer->IsTrackedDeviceConnected(i))
               continue;

            /* additionally check if the device will provide us with the tracking
             * data we want                                                      */
            if (c == vr::TrackedDeviceClass_GenericTracker) {

               /* collect the tracking data */
               vr_pointer->GetControllerStateWithPose(
                     vr::TrackingUniverseStanding,i,&controllerState,
                     sizeof(controllerState),&trackedDevicePose);

               /* seperate the tracking data */
               vr::HmdMatrix34_t vive_pose = trackedDevicePose.mDeviceToAbsoluteTracking;
               vr::HmdVector3_t  linear_vel = trackedDevicePose.vVelocity;
               vr::HmdVector3_t  angular_vel = trackedDevicePose.vAngularVelocity;

               /* ensure that the tracking data is valid before publishing it */
               if (trackedDevicePose.bPoseIsValid) {

                  /* begin construction of our message */
                  nav_msgs::msg::Odometry msg;
                  msg.header.stamp = this->get_clock()->now();
                  msg.header.frame_id = "base_link";

                  /* the layout of this data is the upper three rows
                   * of a homogeneous transformation matrix. Because
                   * of this, our pose can be recovered from the fourth
                   * column, and our rotation can be recovered from the
                   * rotation matrix in the upper left 3x3 area        */

                  /* we will not fill in the covariance matrix */
                  geometry_msgs::msg::PoseWithCovariance pose;
                  pose.pose.position.x = vive_pose.m[0][3];
                  pose.pose.position.y = vive_pose.m[1][3];
                  pose.pose.position.z = vive_pose.m[2][3];

                  /* convert from rotation matrix to quaternion */
                  double trace = vive_pose.m[0][0] + vive_pose.m[1][1] + vive_pose.m[2][2];
                  if (trace > 0.0) {
                     double k = 0.5 / sqrt(1.0 + trace);
                     pose.pose.orientation.x = k * (vive_pose.m[1][2] - vive_pose.m[2][1]);
                     pose.pose.orientation.y = k * (vive_pose.m[2][0] - vive_pose.m[0][2]);
                     pose.pose.orientation.z = k * (vive_pose.m[0][1] - vive_pose.m[1][0]);
                     pose.pose.orientation.w = 0.25 / k;
                  }
                  else if ((vive_pose.m[0][0] > vive_pose.m[1][1]) && (vive_pose.m[0][0] > vive_pose.m[2][2])) {
                     double k = 0.5 / sqrt(1.0 + vive_pose.m[0][0] - vive_pose.m[1][1] - vive_pose.m[2][2]);
                     pose.pose.orientation.x = 0.25 / k;
                     pose.pose.orientation.y = k * (vive_pose.m[0][1] + vive_pose.m[1][0]);
                     pose.pose.orientation.z = k * (vive_pose.m[2][0] - vive_pose.m[0][2]);
                     pose.pose.orientation.w = k * (vive_pose.m[1][2] - vive_pose.m[2][1]);
                  }
                  else if (vive_pose.m[1][1] > vive_pose.m[2][2]) {
                     double k = 0.5 / sqrt(1.0 + vive_pose.m[1][1] - vive_pose.m[0][0] - vive_pose.m[2][2]);
                     pose.pose.orientation.x = k * (vive_pose.m[0][1] + vive_pose.m[1][0]);
                     pose.pose.orientation.y = 0.25 / k;
                     pose.pose.orientation.z = k * (vive_pose.m[1][2] + vive_pose.m[2][1]);
                     pose.pose.orientation.w = k * (vive_pose.m[2][0] - vive_pose.m[0][2]);
                  }
                  else {
                     double k = 0.5 / sqrt(1.0 + vive_pose.m[2][2] - vive_pose.m[0][0] - vive_pose.m[1][1]);
                     pose.pose.orientation.x = k * (vive_pose.m[2][0] + vive_pose.m[0][2]);
                     pose.pose.orientation.y = k * (vive_pose.m[1][2] + vive_pose.m[2][1]);
                     pose.pose.orientation.z = 0.25 / k;
                     pose.pose.orientation.w = k * (vive_pose.m[0][1] - vive_pose.m[1][0]);
                  }

                  /* fill out our velocity values */
                  geometry_msgs::msg::TwistWithCovariance twist;
                  twist.twist.linear.x = linear_vel.v[0];
                  twist.twist.linear.y = linear_vel.v[1];
                  twist.twist.linear.z = linear_vel.v[2];

                  twist.twist.angular.x = angular_vel.v[0];
                  twist.twist.angular.y = angular_vel.v[1];
                  twist.twist.angular.z = angular_vel.v[2];

                  msg.pose = pose;
                  msg.twist = twist;

                  odometry_out->publish(msg);

               }
            }
         }

         /* sleep for poll_delay milliseconds before polling again */
         std::this_thread::sleep_for(std::chrono::milliseconds(poll_delay));
      }

      /* when this loop exits, we want to destroy our
       * open vr instance */
      vr::VR_Shutdown();
   }

   /* this function is a copy and paste of the previous one, but
    * with the addition of a marker array construction. This 
    * duplication avoids having an extra branch in the previous
    * function, which is kind of a lame reason, but that is the
    * reason I am giving                                       */
   void run_process_visual(void) {

      int remaining_attempts = 100;

retry:
      vr::IVRSystem * vr_pointer;
      vr_pointer = NULL;

      vr::EVRInitError eError;
      eError = vr::VRInitError_None;

      vr_pointer = vr::VR_Init(&eError,vr::VRApplication_Background);

      if (eError != vr::VRInitError_None) {
         --remaining_attempts;
         if (remaining_attempts < 0) {
            RCLCPP_INFO(this->get_logger(),"100 attempts to sync have failed, exiting");
            return;
         }
         RCLCPP_INFO(this->get_logger(),"Failed to find steamvr instance, retrying in 3 seconds");
         std::this_thread::sleep_for(std::chrono::seconds(3));
         goto retry;
      }

      RCLCPP_INFO(this->get_logger(),"OpenVR instance obtained");

      while (true) {
         for (int i = 0; i < 64; ++i) {

            vr::TrackedDevicePose_t trackedDevicePose;
            vr::VRControllerState_t controllerState;

            vr::ETrackedDeviceClass c = vr_pointer->GetTrackedDeviceClass(i);

            if (!vr_pointer->IsTrackedDeviceConnected(i))
               continue;

            if (c == vr::TrackedDeviceClass_GenericTracker) {

               vr_pointer->GetControllerStateWithPose(
                     vr::TrackingUniverseStanding,i,&controllerState,
                     sizeof(controllerState),&trackedDevicePose);

               vr::HmdMatrix34_t vive_pose = trackedDevicePose.mDeviceToAbsoluteTracking;
               vr::HmdVector3_t  linear_vel = trackedDevicePose.vVelocity;
               vr::HmdVector3_t  angular_vel = trackedDevicePose.vAngularVelocity;

               if (trackedDevicePose.bPoseIsValid) {

                  nav_msgs::msg::Odometry msg;
                  msg.header.stamp = this->get_clock()->now();
                  msg.header.frame_id = "base_link";

                  /* we will not fill in the covariance matrix */
                  geometry_msgs::msg::PoseWithCovariance pose;
                  pose.pose.position.x = vive_pose.m[0][3];
                  pose.pose.position.z = vive_pose.m[1][3];
                  pose.pose.position.y = vive_pose.m[2][3];

                  /* convert from rotation matrix to quaternion */
                  double trace = vive_pose.m[0][0] + vive_pose.m[1][1] + vive_pose.m[2][2];
                  if (trace > 0.0) {
                     double k = 0.5 / sqrt(1.0 + trace);
                     pose.pose.orientation.x = k * (vive_pose.m[1][2] - vive_pose.m[2][1]);
                     pose.pose.orientation.y = k * (vive_pose.m[2][0] - vive_pose.m[0][2]);
                     pose.pose.orientation.z = k * (vive_pose.m[0][1] - vive_pose.m[1][0]);
                     pose.pose.orientation.w = 0.25 / k;
                  }
else if ((vive_pose.m[0][0] > vive_pose.m[1][1]) && (vive_pose.m[0][0] > vive_pose.m[2][2])) {
                     double k = 0.5 / sqrt(1.0 + vive_pose.m[0][0] - vive_pose.m[1][1] - vive_pose.m[2][2]);
                     pose.pose.orientation.x = 0.25 / k;
                     pose.pose.orientation.y = k * (vive_pose.m[0][1] + vive_pose.m[1][0]);
                     pose.pose.orientation.z = k * (vive_pose.m[2][0] - vive_pose.m[0][2]);
                     pose.pose.orientation.w = k * (vive_pose.m[1][2] - vive_pose.m[2][1]);
                  }
                  else if (vive_pose.m[1][1] > vive_pose.m[2][2]) {
                     double k = 0.5 / sqrt(1.0 + vive_pose.m[1][1] - vive_pose.m[0][0] - vive_pose.m[2][2]);
                     pose.pose.orientation.x = k * (vive_pose.m[0][1] + vive_pose.m[1][0]);
                     pose.pose.orientation.y = 0.25 / k;
                     pose.pose.orientation.z = k * (vive_pose.m[1][2] + vive_pose.m[2][1]);
                     pose.pose.orientation.w = k * (vive_pose.m[2][0] - vive_pose.m[0][2]);
                  }
                  else {
                     double k = 0.5 / sqrt(1.0 + vive_pose.m[2][2] - vive_pose.m[0][0] - vive_pose.m[1][1]);
                     pose.pose.orientation.x = k * (vive_pose.m[2][0] + vive_pose.m[0][2]);
                     pose.pose.orientation.y = k * (vive_pose.m[1][2] + vive_pose.m[2][1]);
                     pose.pose.orientation.z = 0.25 / k;
                     pose.pose.orientation.w = k * (vive_pose.m[0][1] - vive_pose.m[1][0]);
                  }

                  geometry_msgs::msg::TwistWithCovariance twist;
                  twist.twist.linear.x = linear_vel.v[0];
                  twist.twist.linear.y = linear_vel.v[1];
                  twist.twist.linear.z = linear_vel.v[2];

                  twist.twist.angular.x = angular_vel.v[0];
                  twist.twist.angular.y = angular_vel.v[1];
                  twist.twist.angular.z = angular_vel.v[2];

                  msg.pose = pose;
                  msg.twist = twist;

                  visualization_msgs::msg::Marker marker;
                  marker.ns = "vive_points";
                  marker.id = ++id_num;
                  marker.action = visualization_msgs::msg::Marker::ADD;
                  marker.type = visualization_msgs::msg::Marker::CUBE;
                  builtin_interfaces::msg::Duration lifetime; lifetime.sec = 2;
                  marker.lifetime = lifetime;
                  marker.pose = pose.pose;
                  marker.color.r = 0;
                  marker.color.g = 1;
                  marker.color.b = 0;
                  marker.color.a = 1;
                  marker.scale.x = 0.1;
                  marker.scale.y = 0.1;
                  marker.scale.z = 0.1;
                  marker.header.frame_id = "base_link";
                  marker.header.stamp = this->get_clock()->now();

                  odometry_out->publish(msg);
                  markers_out->publish(marker);

               }
            }
         }
         std::this_thread::sleep_for(std::chrono::milliseconds(poll_delay));
      }

      vr::VR_Shutdown();
   }

};

int main(int argc, char ** argv) {
   rclcpp::init(argc,argv);
   rclcpp::spin(std::make_shared<OpenVRTrackerNode>());
   rclcpp::shutdown();
      return 0;
}
