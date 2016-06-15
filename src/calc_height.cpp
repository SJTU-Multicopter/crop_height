#include "ros/ros.h"
#include "std_msgs/Float32.h"
#include <iostream>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <cmath>

using namespace std;

/* inline function to convert deg into rad */
#ifndef DEG2RAD
#define DEG2RAD(x) ((x)*M_PI/180.)
#endif

#ifndef RAD2DEG
#define RAD2DEG(x) ((x)*180./M_PI)
#endif

/* offset of the laser rangefinder */
float offset[] = {0.09f, -0.25f, 0.0f};

/* proportion of predicted height for estimation */
float pred_p = 0.5f;

float pc = 0.25;
float pg = 0.8;

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
	float l = sqrt(_q.q0*_q.q0 + _q.q1*_q.q1 + _q.q2*_q.q2 + _q.q3*_q.q3);
	qt.q0 = _q.q0/l;
	qt.q1 = -_q.q1/l;
	qt.q2 = -_q.q2/l;
	qt.q3 = -_q.q3/l;
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


void Swap(float a, float b)
{
	float c;
	c = a;
	a = b;
	b = c;
}

void BubbleSort2(float a[], int n)  
{
	cout<<endl<<"sort\n"<<endl;
	int j, k;  
	bool flag;  
  
	k = n;  
	flag = true;  
	while (flag)  
	{  
		flag = false;  
		for (j = 1; j < k; j++)
		{  
			if (a[j - 1] > a[j])  
			{
				// cout<<a[j-1]<<"\t"<<a[j]<<endl;
				// Swap(a[j - 1], a[j]);
				float t = a[j-1];
				a[j-1] = a[j];
				a[j] = t;
				// cout<<a[j-1]<<"\t"<<a[j]<<endl;
				flag = true;  
			}
		}
		k--;  
	}  
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
		ros::Publisher CropDistance;															/* publish the distance to the crop */
		ros::Subscriber scan_sub;																/* subscribe the Lidar data */
		ros::Subscriber pose_sub;																/* subscribe mavros data (pose of the drone) */
		ros::Subscriber vel_sub;																/* subscribe velocity data */
	void scanCallBack(const sensor_msgs::LaserScan::ConstPtr& scan);							/* prototype of the callback function for Lidar messages */
	void poseCallBack(const geometry_msgs::PoseStamped::ConstPtr& pose);						/* prototype of the callback function for pose messages */
	void velCallBack(const geometry_msgs::TwistStamped::ConstPtr& velocity);					/* prototype of the callback function for velocity messages */
};

Scan::Scan()
{
	CropDistance = n.advertise<std_msgs::Float32>("/crop_dist", 5);								/* publish the distance to the crop */
	scan_sub = n.subscribe<sensor_msgs::LaserScan>("/scan", 5, &Scan::scanCallBack, this);					/* when the data of Laser rangefinder comes, trigger the callback function */
	pose_sub = n.subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/local", 5, &Scan::poseCallBack, this);	/* subscribe the data of the pose of the drone for correcting the height */
}

void Scan::poseCallBack(const geometry_msgs::PoseStamped::ConstPtr& pose)
{
	/* get the quaternion from mavros */
	q.q1 = pose->pose.orientation.x;
	q.q2 = pose->pose.orientation.y;
	q.q3 = pose->pose.orientation.z;
	q.q0 = pose->pose.orientation.w;

//	cout<<q.q0<<"\t"<<q.q1<<"\t"<<q.q2<<"\t"<<q.q3<<"\n";
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
	float sum = 0;														/* sum of available distances detected */
	int count = 0;														/* number of available distances detected */
	static int scan_angle = 40;											/* angle used for calculation in deg */
	// static int pred_angle = 25;											/* angle used for prediction in deg */
	// bool forward = false;
	// bool backward = false;
	// float pred_sum = 0;
	// float pred_count = 0;

	float pitch = asin(2 * (q.q0 * q.q2 - q.q3 * q.q1));
	
	// if (pitch > 0.15f)
	// 	forward = true;
	// if (pitch < -0.15f)
	// 	backward = true;
	
	// float dist_pred[pred_angle];

	// if (forward)
	// {
	// 	for (int i = 90 - RAD2DEG(pitch) - scan_angle/2 - pred_angle; i < 90 - scan_angle/2 - RAD2DEG(pitch); i++)
	// 	{
	// 		/* only take the values between the minimum and maximum value of the laser rangefinder */
	// 		if (scan->ranges[i]>scan->range_min && scan->ranges[i]<scan->range_max)
	// 		{
	// 			float angle = DEG2RAD(i);

	// 			/* convert the distances from the laser rangefinder into coordinates in the body frame */
	// 			Quaternion p_body;
	// 			p_body.q0 = 0;
	// 			p_body.q1 = offset[0] + scan->ranges[i] * cos(angle);
	// 			p_body.q2 = offset[1];
	// 			p_body.q3 = offset[2] - scan->ranges[i] * sin(angle);

	// 			/* convert the coordinates in body frame into ground frame */
	// 			Quaternion p_ground;
	// 			p_ground = q_mult(q_mult(q, p_body), q_inv(q));

	// 			/* the distance to the crop is z in the ground frame */
	// 			dist_pred[i - (int)(90 - RAD2DEG(pitch) - scan_angle/2 - pred_angle)] = p_ground.q3;
	// 			pred_count ++;
	// 		}
	// 		else
	// 		{
	// 			dist_pred[i - (int)(90 - RAD2DEG(pitch) - scan_angle/2 - pred_angle)] = 6.0f;
	// 		}
	// 	}
	// }
	
	// if (backward)
	// {
	// 	for (int i = 90 - RAD2DEG(pitch) + scan_angle/2; i < 90 - RAD2DEG(pitch) + scan_angle/2 + pred_angle; i++)
	// 	{
	// 		/* only take the values between the minimum and maximum value of the laser rangefinder */
	// 		if (scan->ranges[i]>scan->range_min && scan->ranges[i]<scan->range_max)
	// 		{
	// 			float angle = DEG2RAD(i);

	// 			 convert the distances from the laser rangefinder into coordinates in the body frame 
	// 			Quaternion p_body;
	// 			p_body.q0 = 0;
	// 			p_body.q1 = offset[0] + scan->ranges[i] * cos(angle);
	// 			p_body.q2 = offset[1];
	// 			p_body.q3 = offset[2] - scan->ranges[i] * sin(angle);

	// 			/* convert the coordinates in body frame into ground frame */
	// 			Quaternion p_ground;
	// 			p_ground = q_mult(q_mult(q, p_body), q_inv(q));

	// 			/* the distance to the crop is z in the ground frame */
	// 			dist_pred[i - (int)(90 - RAD2DEG(pitch) - scan_angle/2 - pred_angle)] = p_ground.q3;
	// 			pred_count ++;
	// 		}
	// 		else
	// 		{
	// 			dist_pred[i - (int)(90 - RAD2DEG(pitch) + scan_angle/2)] = 6.0f;
	// 		}
	// 	}
	// }

	// BubbleSort2(dist_pred, pred_angle);

	// float CDF_pred[pred_angle];

	// for (int i = 0; i < pred_angle; ++i)
	// {
	// 	if (dist_pred[i] <= scan->range_min)
	// 	{
	// 		CDF_pred[i] = 0.0f;
	// 	}
	// 	else if (dist_pred[i] < scan->range_max)
	// 	{
	// 		CDF_pred[i] = (float)(i + 1)/(float)(pred_count);
	// 	}
	// 	else
	// 	{
	// 		CDF_pred[i] = 1.0f;
	// 	}
	// }

	// int crop_pred_start = -1;
	// int crop_pred_end = 0;
	// int ground_pred_start = -1;
	// int ground_pred_end = 0;
	
	// for (int i = 0; i < pred_angle; ++i)
	// {
	// 	if (dist_pred[i] > scan->range_min && dist_pred[i] < scan->range_max)
	// 	{
	// 		if (CDF_pred[i] <= pc)
	// 		{
	// 			if (crop_pred_start == -1)
	// 			{
	// 				crop_pred_start = i;
	// 			}
	// 			crop_pred_end = i;
	// 		}
	// 		if (CDF_pred[i] > pg && CDF_pred[i] < 1)
	// 		{
	// 			if (ground_pred_start == -1)
	// 			{
	// 				ground_pred_start = i;
	// 			}
	// 			ground_pred_end = i;
	// 		}
	// 	}
	// }

	// int crop_pred_med = (int)(crop_pred_start/2 + crop_pred_end/2);
	// float crop_dist_pred = (dist_pred[crop_pred_med - 1] + dist_pred[crop_pred_med + 1]);

	// int ground_pred_med = (int)(ground_pred_start/2 + ground_pred_end/2);
	// float ground_dist_pred = (dist_pred[ground_pred_med - 1] + dist_pred[ground_pred_med + 1]);

	float dist[scan_angle];

	/* only take the values within the set range */
	for (int i = 90 - scan_angle/2 - RAD2DEG(pitch); i < 90 + scan_angle/2 - RAD2DEG(pitch); i++)
	{
		/* only take the values between the minimum and maximum value of the laser rangefinder */
		if (scan->ranges[i]>scan->range_min && scan->ranges[i]<scan->range_max)
		{
			float angle = DEG2RAD(i);

			/* convert the distances from the laser rangefinder into coordinates in the body frame */
			Quaternion p_body;
			p_body.q0 = 0;
			p_body.q1 = offset[0] + scan->ranges[i] * cos(angle);
			p_body.q2 = offset[1];
			p_body.q3 = offset[2] - scan->ranges[i] * sin(angle);

			/* convert the coordinates in body frame into ground frame */
			Quaternion p_ground;
			p_ground = q_mult(q_mult(q, p_body), q_inv(q));

			/* the distance to the crop is z in the ground frame */
			// sum += p_ground.q3;
			count++;
			dist[i - (int)(90 - scan_angle/2 - RAD2DEG(pitch))] = -p_ground.q3;

			//intensities += scan->intensities[i];
		}
		else
		{
			dist[i - (int)(90 - scan_angle/2 - RAD2DEG(pitch))] = 6.001;
		}
	}


	// for (int i = 0; i < scan_angle; ++i)
	// {
	// 	cout<<dist[i]<<"\t";
	// }
	// cout<<endl;

	BubbleSort2(dist, scan_angle);

	for (int i = 0; i < scan_angle; ++i)
	{
		cout<<dist[i]<<"\t";
	}
	cout<<endl;

	float CDF[scan_angle];

	for (int i = 0; i < scan_angle; ++i)
	{
		if (dist[i] <= scan->range_min)
		{
			CDF[i] = 0.0f;
		}
		else if (dist[i] < scan->range_max)
		{
			CDF[i] = (float)(i + 1)/(float)(count);
		}
		else
		{
			CDF[i] = 1.0f;
		}
	}

	int crop_start = -1;
	int crop_end = 0;
	int ground_start = -1;
	int ground_end = 0;
	
	for (int i = 0; i < scan_angle; ++i)
	{
		if (dist[i] > scan->range_min && dist[i] < scan->range_max)
		{
			if (CDF[i] <= pc)
			{
				if (crop_start == -1)
				{
					crop_start = i;
				}
				crop_end = i;
			}
			if (CDF[i] > pg && CDF[i] < 1)
			{
				if (ground_start == -1)
				{
					ground_start = i;
				}
				ground_end = i;
			}
		}
	}

	int crop_med = (int)(crop_start/2 + crop_end/2);
	float crop_dist = (dist[crop_med - 1] + dist[crop_med + 1])/2;

	int ground_med = (int)(ground_start/2 + ground_end/2);
	float ground_dist = (dist[ground_med - 1] + dist[ground_med + 1])/2;

	/* publish the distance to the crop in Distance */
	std_msgs::Float32 Distance;
	// Distance.data = crop_dist * (1 - pred_p) + crop_dist_pred * pred_p;
	Distance.data = -crop_dist;
	
	if (Distance.data == Distance.data || Distance.data < 0)
	{}
	else
	{
		Distance.data = -6.0f;
	}
	
	CropDistance.publish(Distance);
	ROS_INFO("\npitch:\t%f \ndist:\t%f \n",RAD2DEG(pitch), Distance.data);
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "Calculate_the_distance_to_the_crop");
	Scan scan;
	ros::spin();
}
