#include "ros/ros.h"
#include "std_msgs/Float32.h"
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <cmath>

/* inline function to convert deg into rad */
#define DEG2RAD(x) ((x)*M_PI/180.)

/* offset of the laser rangefinder */
float offset[] = {0.0f, 0.0f, 0.0f};

float vel_threshold = 0.5f;

float pred_p = 0.5f;

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
	float vel[3];
private:
	ros::NodeHandle n;
		ros::Publisher yaw_pub;
		ros::Publisher dir_pub;
		ros::Publisher CropDistance;										/* publish the distance to the crop */
		ros::Subscriber scan_sub;											/* subscribe the Lidar data */
		ros::Subscriber pose_sub;											/* subscribe mavros data (pose of the drone) */
		ros::Subscriber vel_sub;
	void scanCallBack(const sensor_msgs::LaserScan::ConstPtr& scan);		/* prototype of the callback function for Lidar messages */
	void poseCallBack(const geometry_msgs::PoseStamped::ConstPtr& pose);	/* prototype of the callback function for pose messages */
	void velCallBack(const geometry_msgs::TwistStamped::ConstPtr& velocity);
};

Scan::Scan()
{
	yaw_pub = n.advertise<std_msgs::Float32>("/yaw", 100);
	dir_pub = n.advertise<std_msgs::Float32>("/dir", 100);
	CropDistance = n.advertise<std_msgs::Float32>("/crop_dist", 100);																/* publish the distance to the crop */
	scan_sub = n.subscribe<sensor_msgs::LaserScan>("/scan", 100, &Scan::scanCallBack, this);									/* when the data of Laser rangefinder comes, trigger the callback function */
	pose_sub = n.subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose", 100, &Scan::poseCallBack, this);		/* subscribe the data of the pose of the drone for correcting the height */
	vel_sub = n.subscribe<geometry_msgs::TwistStamped>("/mavros/local_position/velocity", 100, &Scan::velCallBack, this);
}

void Scan::poseCallBack(const geometry_msgs::PoseStamped::ConstPtr& pose)
{
	/* get the quaternion from mavros */
	q.q1 = pose->pose.orientation.x;
	q.q2 = pose->pose.orientation.y;
	q.q3 = pose->pose.orientation.z;
	q.q0 = pose->pose.orientation.w;
}

void Scan::velCallBack(const geometry_msgs::TwistStamped::ConstPtr& velocity)
{
	vel[0] = velocity->twist.linear.x;
	vel[1] = velocity->twist.linear.y;
	vel[2] = velocity->twist.linear.z;
}
void Scan::scanCallBack(const sensor_msgs::LaserScan::ConstPtr& scan)
{
	/* use the beam facing downwards to calculate the average height */
	float sum = 0;												/* sum of available distances detected */
	int count = 0;												/* number of available distances detected */
	int scan_angle = 30;										/* angle used for calculation in deg */
	int pred_angle = 15;
	float pred_sum = 0;
	int pred_count = 0;

	bool _vel_comp_enabled = false;
	bool forward = false;
	bool backward = false;

	float vel_horizon = sqrt(vel[0]*vel[0] + vel[1]*vel[1]);
	if (vel_horizon > vel_threshold)
	{
		_vel_comp_enabled = true;
	}

	std_msgs::Float32 yaw;
	yaw.data = atan2(2*(q.q0*q.q3 + q.q1*q.q2), 1 - 2*(q.q2*q.q2 + q.q3*q.q3));
	std_msgs::Float32 dir;
	dir.data = atan2(vel[0], vel[1]);
	float dir_diff = abs(dir.data - yaw.data);

	yaw_pub.publish(yaw);
	dir_pub.publish(dir);

	if(_vel_comp_enabled)
	{
		if (dir_diff < M_PI/18)
		{
			forward = true;
			backward = false;
			ROS_INFO("Forward");
		}
		else if (dir_diff > M_PI * 17/18)
		{
			forward = false;
			backward = true;
			ROS_INFO("Backward");
		}
	}


	/* only take the values within the set range */
	for (int i = 90 - scan_angle/2; i < 90 + scan_angle/2; i++)
	{
		/* only take the values between the minimum and maximum value of the laser rangefinder */
		if (scan->ranges[i]>scan->range_min && scan->ranges[i]<scan->range_max)
		{
			float angle = DEG2RAD(i);

			/* convert the distances from the laser rangefinder into coordinates in the body frame */
			Quaternion p_body;
			p_body.q0 = 0;
			p_body.q1 = offset[0];
			p_body.q2 = offset[1] + scan->ranges[i] * cos(angle);
			p_body.q3 = offset[2] - scan->ranges[i] * sin(angle);

			/* convert the coordinates in body frame into ground frame */
			Quaternion p_ground;
			p_ground = q_mult(q_mult(q_inv(q), p_body), q);

			/* the distance to the crop is z in the ground frame */
			sum += p_ground.q3;
			count++;
		}		
	}

	if (forward)
	{
		for (int i = 90 - scan_angle/2 - pred_angle; i < 90 - scan_angle/2; i++)
		{
			if (scan->ranges[i]>scan->range_min && scan->ranges[i]<scan->range_max)
			{
				float angle = DEG2RAD(i);

				/* convert the distances from the laser rangefinder into coordinates in the body frame */
				Quaternion p_body;
				p_body.q0 = 0;
				p_body.q1 = offset[0];
				p_body.q2 = offset[1] + scan->ranges[i] * cos(angle);
				p_body.q3 = offset[2] - scan->ranges[i] * sin(angle);

				/* convert the coordinates in body frame into ground frame */
				Quaternion p_ground;
				p_ground = q_mult(q_mult(q_inv(q), p_body), q);

				/* the distance to the crop is z in the ground frame */
				pred_sum += p_ground.q3;
				pred_count++;
			}
		}
	}

	if (backward)
	{
		for (int i = 90 + scan_angle/2; i < 90 + scan_angle/2 + pred_angle; i++)
		{
			if (scan->ranges[i]>scan->range_min && scan->ranges[i]<scan->range_max)
			{
				float angle = DEG2RAD(i);

				/* convert the distances from the laser rangefinder into coordinates in the body frame */
				Quaternion p_body;
				p_body.q0 = 0;
				p_body.q1 = offset[0];
				p_body.q2 = offset[1] + scan->ranges[i] * cos(angle);
				p_body.q3 = offset[2] - scan->ranges[i] * sin(angle);

				/* convert the coordinates in body frame into ground frame */
				Quaternion p_ground;
				p_ground = q_mult(q_mult(q_inv(q), p_body), q);

				/* the distance to the crop is z in the ground frame */
				pred_sum += p_ground.q3;
				pred_count++;
			}
		}
	}

	float dir_dist = sum/count;
	float pred_dist = pred_sum/pred_count;

	/* publish the distance to the crop in Distance */
	std_msgs::Float32 Distance;
	Distance.data = dir_dist;
	CropDistance.publish(Distance);
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "Calculate_the_distance_to_the_crop");
	Scan scan;
	ros::spin();
}
