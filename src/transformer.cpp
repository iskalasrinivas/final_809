#include <string>
#include <sstream>
#include <transformer.h>

Transformer::Transformer(const std::string &topic) : topic_(topic), tf_listener(tf_buffer)
{
   // parent_ = getCameraName();
   // child_ = parent_ + "_child";
   auto ind = topic.rfind('/');
   frame_ = topic.substr(ind + 1, topic.size() - ind) + "_frame";
//   ROS_WARN_STREAM("parent frame is : " << frame_);
}

Transformer::Transformer() : tf_listener(tf_buffer)
{
}

Transformer::~Transformer() {}


geometry_msgs::Pose Transformer::getChildPose(geometry_msgs::Pose pose_wrt_parent)
{
   try
   {
      t_camera_world = tf_buffer.lookupTransform("world", frame_, ros::Time(0));
   }
   catch (tf2::TransformException &ex)
   {
      ROS_WARN("exception");
      ROS_WARN("%s", ex.what());
   }

   ros::Duration(0.01).sleep();

   try
   {
      tf2::doTransform(pose_wrt_parent, pose_wrt_world, t_camera_world);
   }
   catch (tf2::TransformException &ex)
   {
      ROS_WARN("exception while converting child frame pose to world frame");
      ROS_WARN("%s", ex.what());
      ros::Duration(0.01).sleep();
   }
   ros::Duration(0.01).sleep();

   return pose_wrt_world;
}

geometry_msgs::Pose Transformer::getChildPose(std::string cam_name, geometry_msgs::Pose pose_wrt_parent)
{
   frame_ = cam_name + "_frame";
   // ROS_WARN_STREAM("parent frame is : " << frame);
   try
   {
      t_camera_world = tf_buffer.lookupTransform("world", frame_, ros::Time(0));
   }
   catch (tf2::TransformException &ex)
   {
      ROS_WARN("exception");
      ROS_WARN("%s", ex.what());
   }

   ros::Duration(0.01).sleep();

   try
   {
      tf2::doTransform(pose_wrt_parent, pose_wrt_world, t_camera_world);
   }
   catch (tf2::TransformException &ex)
   {
      ROS_WARN("exception while converting child frame pose to world frame");
      ROS_WARN("%s", ex.what());
      ros::Duration(0.01).sleep();
   }
   ros::Duration(0.01).sleep();

   return pose_wrt_world;
}
