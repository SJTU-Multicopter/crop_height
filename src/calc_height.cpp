#include "ros/ros.h"
#include "std_msgs/Float32.h"
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point32.h>
#include <cmath>

/* inline function to convert deg into rad */
#ifndef DEG2RAD
#define DEG2RAD(x) ((x)*M_PI/180.)
#endif

#ifndef RAD2DEG
#define RAD2DEG(x) ((x)*180./M_PI)
#endif

/* offset of the laser rangefinder */
float offset[] = {0.09f, -0.25f, 0.0f};

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

	float pre_dist1;
	float pre_dist2;
};

Scan::Scan()
{
	CropDistance = n.advertise<geometry_msgs::Point32>("/crop_dist", 5);																/* publish the distance to the crop */
	scan_sub = n.subscribe<sensor_msgs::LaserScan>("/scan", 5, &Scan::scanCallBack, this);									/* when the data of Laser rangefinder comes, trigger the callback function */
	pose_sub = n.subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/local", 5, &Scan::poseCallBack, this);		/* subscribe the data of the pose of the drone for correcting the height */

	pre_dist1 = 0.0;
	pre_dist2 = -6.0;
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
	float average = 0;												/* average of available distances detected */
	float standard_error = 0;									/* standard_error of available distances detected */
	int count = 0;												/* number of available distances detected */
	int scan_angle = 30;										/* angle used for calculation in deg */
	float values[30];          //to store useful distance point value

	float fdistance = -6.0;
	float confidence1 = 0.0;
	float confidence2 = 0.0;

	float pitch = asin(2 * (q.q0 * q.q2 - q.q3 * q.q1));

	/* only take the values within the set range */
	for (int i = 90 - scan_angle/2 - RAD2DEG(pitch); i < 90 + scan_angle/2 - RAD2DEG(pitch); i++)
	{
		/* only take the values between the minimum and maximum value of the laser rangefinder */
		if (scan->ranges[i] > scan->range_min && scan->ranges[i] < scan->range_max)
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
			//sum += p_ground.q3;
			if(p_ground.q3 > -5.5 && p_ground.q3 < -0.3)
			{
				values[count] = p_ground.q3;
				count++;
			}
		}		
	}

	if(count > 0)
	{
		/**Average**/
		for(int i = 0; i < count; i++)
		{
			sum += values[count];
		}
		average = sum / count;

		/**Varience**/
		float total_varience = 0.0;
		for(int i = 0; i < count; i++)
		{
			total_varience += (values[count] - average) * (values[count] - average);
		}
		standard_error = sqrt(total_varience / n);  //standard error, between(0,5.2), ideal (0,1) if crop is 1m high

		if(standard_error > 1) confidence1 = 0.0;
		else confidence1 = 1.0 - standard_error;
	}
	else average = -6.0;

	/**average in 3 times**/
	fdistance = (pre_dist1 + pre_dist2 + average)/3.0;
	/**max error in 3 times**/
	float max_error = 0.0;
	if(fabs(pre_dist1 - fdistance) > max_error) max_error = fabs(pre_dist1 - fdistance);
	if(fabs(pre_dist2 - fdistance) > max_error) max_error = fabs(pre_dist2 - fdistance);
	if(fabs(average - fdistance) > max_error) max_error = fabs(average - fdistance);
	
	if(max_error > 0.5) confidence2 = 0.0;
	else if(max_error < 0.1) confidence2 = 1.0;
	else
	{
		confidence2 = (max_error-0.1)/0.4;
	}

	/**set vlaues**/
	pre_dist2 = pre_dist1;
	pre_dist1 = average;

	geometry_msgs::Point32 Distance; 
	Distance.x = fdistance;
	Distance.y = confidence1;
	Distance.z = confidence2;

	/* publish the distance to the crop in Distance */
	
	//Distance.x = sum / (count);

	/*if (Distance.data == Distance.data)
	{}
	else
	{
		Distance.data = -6.0f;
	}*/

	CropDistance.publish(Distance);
	ROS_INFO("\npitch:\t%f \ndist:\t%f \n",RAD2DEG(pitch), Distance.x);
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "Calculate_the_distance_to_the_crop");
	Scan scan;
	ros::spin();
}
