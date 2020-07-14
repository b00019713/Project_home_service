#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include "nav_msgs/Odometry.h"
#include <math.h> 
// pick up (-18,3.8) dropoff (1,-4)

// Global variables
bool waiting_for_pickup = true;
bool dropedoff = false;

float pickup[3] = {-18.0,3.8,1.0};
float dropoff[3] = {1,-4,1.0};



// Call back for odometry msgs
void callback_odom(const nav_msgs::Odometry::ConstPtr &msg){
	//Get current position
	float robot_x = msg->pose.pose.position.x;
	float robot_y = msg->pose.pose.position.y;


	if(waiting_for_pickup){
		if(sqrt(pow((pickup[0]-robot_x),2)+pow((pickup[1]-robot_y),2))<0.2){
			waiting_for_pickup=false;
    	}
	}else{
		if(sqrt(pow((dropoff[0]-robot_x),2)+pow((dropoff[1]-robot_y),2))<0.2){
			dropedoff=true;
	    }
	}
}




// Main Function

int main( int argc, char** argv )
{
	// Subscribe to odometry msgs
	ros::Subscriber odom_subscriber = n.subscribe("/odom",10, callback_odom);  



	// Setup the marker
	ros::init(argc, argv, "basic_shapes");
	ros::NodeHandle n;
	ros::Rate r(1);
	ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1); 
	// Set our initial shape type to be a cube
	uint32_t shape = visualization_msgs::Marker::CUBE;
	visualization_msgs::Marker marker;
	// Set the frame ID and timestamp.  See the TF tutorials for information on these.
	marker.header.frame_id = "map";
	marker.header.stamp = ros::Time::now();
	// Set the namespace and id for this marker.  This serves to create a unique ID
	// Any marker sent with the same namespace and id will overwrite the old one
	marker.ns = "cargo";
	marker.id = 0;
	// Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
	marker.type = shape;
	// Set the marker action.  Options are ADD, DELETE
	marker.action = visualization_msgs::Marker::ADD;
	// Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
	marker.pose.position.x = 0;
	marker.pose.position.y = 0;
	marker.pose.position.z = 0;
	marker.pose.orientation.x = 0.0;
	marker.pose.orientation.y = 0.0;
	marker.pose.orientation.z = 0.0;
	marker.pose.orientation.w = 1.0;
	// Set the scale of the marker -- 1x1x1 here means 1m on a side
	marker.scale.x = 0.5;
	marker.scale.y = 0.5;
	marker.scale.z = 0.5;
	// Set the color -- be sure to set alpha to something non-zero!
	marker.color.r = 0.0f;
	marker.color.g = 0.0f;
	marker.color.b = 1.0f;
	marker.color.a = 1.0;
	marker.lifetime = ros::Duration();

  // Publish the marker
  while (marker_pub.getNumSubscribers() < 1)
  {
    if (!ros::ok())
    {
      return 0;
    }
    ROS_WARN("Please create a subscriber to the marker");
    sleep(1);
  }

  while(ros::ok()){
	//Waiting to be picked up -> marker has to be at the pickup zone
    if(waiting_for_pickup){
		marker.action = visualization_msgs::Marker::ADD;
		marker.pose.position.x = pickup[0];
		marker.pose.position.y = pickup[1];
    }
	//The marker has been droped of -> the marker has to be at a drop off zone
	else if(dropedoff){
		marker.action = visualization_msgs::Marker::ADD;
		marker.pose.position.x = dropoff[0];
		marker.pose.position.y = dropoff[1];
	}
	//Not waiting and not droped off yet -> must be in transit -> hide the marker 	
	else{
		marker.action = visualization_msgs::Marker::DELETE;
	}
	marker_pub.publish(marker);
	ros::spinOnce();
  }
  return 0;  
}
