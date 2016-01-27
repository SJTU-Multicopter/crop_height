#include "ros/ros.h"
#include "std_msgs/Float32.h"
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/PoseStamped.h>
#include <cmath>

/* inline function to convert deg into rad */
#define DEG2RAD(x) ((x)*M_PI/180.)
#define RAD2DEG(x) ((x)*180./M_PI)

/* offset of the laser rangefinder */
float offset[] = {0.0f, 0.0f, 0.0f};

/* struct to store quaternions */
struct Quaternion
{
	float q0;
	float q1;
	float q2;
	float q3;
};

/* calculating the inverse of a quaternion */
Quaternion q_inv(const Quaternion& _q)
{
	Quaternion qt;
	qt.q0 = _q.q0;
	qt.q1 = -_q.q1;
	qt.q2 = -_q.q2;
	qt.q3 = -_q.q3;
	return qt;
}

/* calculating the product of two quaternions q*p */
Quaternion q_mult(const Quaternion& _q, const Quaternion& _p)
{
	Quaternion qp;
	qp.q0 = _q.q0*_p.q0 - _q.q1*_p.q1 - _q.q2*_p.q2 - _q.q3*_p.q3;
	qp.q1 = _q.q0*_p.q1 + _q.q1*_p.q0 - _q.q2*_p.q3 + _q.q3*_p.q2;
	qp.q2 = _q.q0*_p.q2 + _q.q1*_p.q3 + _q.q2*_p.q0 - _q.q3*_p.q1;
	qp.q3 = _q.q0*_p.q3 - _q.q1*_p.q2 + _q.q2*_p.q1 + _q.q3*_p.q0;
	return qp;
}

class Scan{
public:
	Scan();
	Quaternion q;
private:
	ros::NodeHandle n;
		ros::Publisher CropDistance;										/* publish the distance to the crop */
		ros::Subscriber scan_sub;											/* subscribe the Lidar data */
		ros::Subscriber pose_sub;											/* subscribe mavros data (pose of the drone) */
	void scanCallBack(const sensor_msgs::LaserScan::ConstPtr& scan);		/* prototype of the callback function for Lidar messages */
	void poseCallBack(const geometry_msgs::PoseStamped::ConstPtr& pose);	/* prototype of the callback function for pose messages */
};

Scan::Scan()
{
	CropDistance = n.advertise<std_msgs::Float32>("/crop_dist", 1);																/* publish the distance to the crop */
	scan_sub = n.subscribe<sensor_msgs::LaserScan>("/scan", 1, &Scan::scanCallBack, this);									/* when the data of Laser rangefinder comes, trigger the callback function */
	pose_sub = n.subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/local", 1, &Scan::poseCallBack, this);		/* subscribe the data of the pose of the drone for correcting the height */
}

void Scan::poseCallBack(const geometry_msgs::PoseStamped::ConstPtr& pose)
{
	/* get the quaternion from mavros */
	q.q1 = pose->pose.orientation.x;
	q.q2 = pose->pose.orientation.y;
	q.q3 = pose->pose.orientation.z;
	q.q0 = pose->pose.orientation.w;
}

void Scan::scanCallBack(const sensor_msgs::LaserScan::ConstPtr& scan)
{
	/* use the beam facing downwards to calculate the average height */
	float sum = 0;												/* sum of available distances detected */
	float intensities = 0;
	int count = 0;												/* number of available distances detected */
	int scan_angle = 30;										/* angle used for calculation in deg */

	float roll = atan2(2*(q.q0*q.q1 + q.q2*q.q3), 1 - 2*(q.q1*q.q1 + q.q2*q.q2));
	float pitch = asin(2*(q.q0*q.q2 - q.q3*q.q1));
			
	/* convert the coordinate of laser rangefinder from body frame into ground frame */
	Quaternion p_body;
	p_body.q0 = 0;
	p_body.q1 = offset[0];
	p_body.q2 = offset[1];
	p_body.q3 = offset[2];

	Quaternion p_ground;
	p_ground = q_mult(q_mult(q_inv(q), p_body), q);

	float z_offset = p_ground.q3;

	/* only take the values within the set range */
	for (int i = 90 - scan_angle/2 - RAD2DEG(pitch); i < 90 + scan_angle/2 - RAD2DEG(pitch); i++)
	{
		/* only take the values between the minimum and maximum value of the laser rangefinder */
		if (scan->ranges[i]>scan->range_min && scan->ranges[i]<scan->range_max)
		{
			float angle = DEG2RAD(i);

			/* the distance to the crop is z in the ground frame */
			sum += scan->ranges[i] * sin(angle + pitch) * cos(roll) * scan->intensities[i];
			intensities += scan->intensities[i];
			count++;
		}		
	}

	/* publish the distance to the crop in Distance */
	std_msgs::Float32 Distance;
	Distance.data = sum / count / intensities;
	CropDistance.publish(Distance);
	ROS_INFO("distance:%f \nroll:%f \npitch%f \n",Distance.data, roll, pitch);
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "Calculate_the_distance_to_the_crop");
	Scan scan;
	ros::spin();
}
