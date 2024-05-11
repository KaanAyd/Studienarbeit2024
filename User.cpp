/*
****************************************************************************
**  Modified by Nehal Ranabhatt
**  CarMaker - Version 11.0.1
**  Vehicle Dynamics Simulation Toolkit
**
**  Copyright (C)   IPG Automotive GmbH
**                  Bannwaldallee 60             Phone  +49.721.98520.0
**                  76185 Karlsruhe              Fax    +49.721.98520.99
**                  Germany                      WWW    www.ipg-automotive.com
******************************************************************************
**
** Functions
** ---------
**
** Initialization
**
**	User_Init_First ()
**	User_PrintUsage ()
**	User_ScanCmdLine ()
**
**	User_AppLogFilter ()
**
**	User_Init ()
**	User_Register ()
**	User_DeclQuants ()
**
**	User_Param_Add ()
**	User_Param_Get ()
**
**
** Main TestRun Start/End:
**
**	User_TestRun_Start_atBegin ()
**	User_TestRun_Start_atEnd ()
**	User_TestRun_Start_StaticCond_Calc ()
**	User_TestRun_Start_Finalize ()
**	User_TestRun_RampUp ()
**
**	User_TestRun_End_First ()
**	User_TestRun_End ()
**
**
** Main Cycle:
**
**	User_In ()
**
**	User_DrivMan_Calc ()
** 	User_Traffic_Calc ()
**	User_VehicleControl_Calc ()
**	User_Brake_Calc ()           in Vhcl_Calc ()
**	User_Calc ()
**	User_Check_IsIdle ()
**
**	User_Out ()
**
**
** APO Communication:
**
**	User_ApoMsg_Eval ()
**	User_ApoMsg_Send ()
**
**	User_ShutDown ()
**	User_End ()
**	User_Cleanup ()
**
**
******************************************************************************
*/

#include <Global.h>

#if defined(WIN32)
# include <windows.h>
#endif

#include <stdlib.h>
#include <string.h>
#include <math.h>


#if defined(XENO)
#include <mio.h>
#endif
# include <ioconf.h>

#include <CarMaker.h>
#include <Car/Vehicle_Car.h>

#include <ADASRP.h>

#include <rbs.h>

#include "IOVec.h"
#include "User.h"

#include "Vehicle.h"
#include "Vehicle/Sensor_Object.h"
#include "Vehicle/Sensor_USonicRSI.h"
#include <sensor_msgs/LaserScan.h>
#include "Vehicle/Sensor_LidarRSI.h"

#include "ros/ros.h"
#include "ROS_comm.h"

#include <std_msgs/Float32.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Int64.h>
#include <signal.h>

#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3.h>

#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Range.h>

#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>

#include <iostream>
#include <fstream>
#include <chrono>
#include <algorithm>
#include <iterator>

#include <tf/transform_listener.h>

#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Pose.h>
#include <tf/transform_broadcaster.h>

#include <sensor_msgs/LaserScan.h>
#include <Vehicle/Sensor_LidarRSI.h>

//#include <sensor_msgs/PointCloud2.h>
//#include <pcl_ros/point_cloud.h>
//#include <pcl/point_types.h>
//#include <pcl_conversions/pcl_conversions.h>

//Fzg-Typ
#define TRUCK_ZF 1
#define TRUCK_BRECTER 2
int TruckType = TRUCK_ZF;

ros::Publisher odom_pub;
ros::Publisher velocity_pub;
ros::Publisher yaw_pub ;
ros::Publisher rear_right_roll_pub;
ros::Publisher rear_axis_posX_pub;
ros::Publisher rear_axis_posY_pub;

ros::Publisher steering_angle_pub;
ros::Publisher steering_control_pub;
ros::Publisher velocity_control_pub;
ros::Publisher drive_mode_control_pub;
ros::Publisher brake_acc_control_pub;
ros::Publisher maneuver_time_pub;
ros::Publisher lidar_pub;


// USR
ros::Publisher us_pub_vr;


// Subscriber
ros::Subscriber drive_mode_control_sub;
ros::Subscriber steering_control_sub;
ros::Subscriber velocity_control_sub;
ros::Subscriber brake_acc_control_sub;


std_msgs::Int32 int32_msg;

double current_time, last_time;
double current_time_for_publish, last_time_for_publish;
int counter_for_publish = 0;


//Change Velocity of simulation vehicle
void control_velocity_cb (const std_msgs::Float64& msg)
{
	float velocity2= 0.0;

	velocity2 = msg.data;
	DVA_WriteRequest("Driver.ReCon.Speed", OWMode_Abs, 1000, 0, 0, velocity2, NULL);

}

//Change Drive Mode
void control_drive_mode_cb (const std_msgs::Int32& msg)
{
	int drive_mode = 0;

	drive_mode = msg.data;

	DVA_WriteRequest("Driver.ReCon.DriveMode", OWMode_Abs, 5000, 0, 0, drive_mode, NULL);
}

//Steering Angle of Front Truck Axle in degree
void control_steer_cb (const std_msgs::Float64& msg)
{
	double steering2 = 0;
	double steering2_rad = 0;
	double steering2_wheel_rad = 0;
	double pi = 3.141592654;
	double steering_transform = 16.13168724; // Transform factor from Front Truck axle to steering wheel


	steering2 = msg.data;

	double global_time = ros::Time::now().toSec();
	double global_time_ms = global_time * 1000;

	//std::cout << std::fixed << "Time: [s] " << int(global_time) % 1000 << "," << long(global_time_ms) % 1000 << " Subs " << std::setprecision(3) << steering2 << std::endl; 

	// Umrechnung in rad
	steering2_rad = (steering2 / 180) * pi;

	steering2_wheel_rad = steering2_rad * steering_transform;
	DVA_WriteRequest("DM.Steer.Ang", OWMode_Abs, 5000, 0, 0, steering2_wheel_rad, NULL);

}

/* @@PLUGIN-BEGIN-INCLUDE@@ - Automatically generated code - don't edit! */
/* @@PLUGIN-END@@ */


int UserCalcCalledByAppTestRunCalc = 0;


tUser	User;



/*
** User_Init_First ()
**
** First, low level initialization of the User module
**
** Call:
** - one times at start of program
** - no realtime conditions
**
*/

int
User_Init_First (void)
{
    memset (&User, 0, sizeof(User));

    return 0;
}



/*
** User_PrintUsage ()
**
** Print the user/application specific programm arguments
*/

void
User_PrintUsage (const char *Pgm)
{
    /* REMARK: 1 log statement for each usage line, no line breaks */
    LogUsage("\n");
    LogUsage("Usage: %s [options] [testrun]\n", Pgm);
    LogUsage("Options:\n");

#if defined(CM_HIL)
    {
	const tIOConfig *cf;
	const char *defio = IO_GetDefault();
	LogUsage(" -io %-12s Default I/O configuration (%s)\n", "default",
	    (defio!=NULL && strcmp(defio, "none")!=0) ? defio : "minimal I/O");
	for (cf=IO_GetConfigurations(); cf->Name!=NULL; cf++)
	    LogUsage(" -io %-12s %s\n", cf->Name, cf->Description);
    }
#endif
}



/*
** User_ScanCmdLine ()
**
** Scan application specific command line arguments
**
** Return:
** - argv: last unscanned argument
** - NULL: error or unknown argument
*/

char **
User_ScanCmdLine (int argc, char **argv)
{
    const char *Pgm = argv[0];

    /* I/O configuration to be used in case no configuration was
       specified on the command line. */
    IO_SelectDefault("default" /* or "demoapp", "demorbs,demofr" etc. */);

    while (*++argv) {
	if (strcmp(*argv, "-io") == 0 && argv[1] != NULL) {
	    if (IO_Select(*++argv) != 0)
		return NULL;
	} else if (strcmp(*argv, "-h") == 0 || strcmp(*argv, "-help") == 0) {
	    User_PrintUsage(Pgm);
	    SimCore_PrintUsage(Pgm); /* Possible exit(), depending on CM-platform! */
	    return  NULL;
	} else if ((*argv)[0] == '-') {
	    LogErrF(EC_General, "Unknown option '%s'", *argv);
	    return NULL;
	} else {
	    break;
	}
    }

    return argv;
}



/*
** User_Init ()
**
** Basic initialization of the module User.o
**
** Call:
** - once at program start
** - no realtime conditions
*/


//ROS Initialization

void initialize_ros_comm() {
	int argc = 0;
	char **argv;
	ros::init(argc, argv, "initialise_node");
	ROS_INFO("ros master started!");

	ros::NodeHandle nh;
	ROS_INFO("initialized nodehandler");

	current_time_for_publish = DrivMan.ActualMan.Time;
	last_time_for_publish = DrivMan.ActualMan.Time;

	//Publisher
	odom_pub = nh.advertise<nav_msgs::Odometry>("/odom", 1000);
	velocity_pub= nh.advertise<std_msgs::Float64>("/velocity", 1000);
	yaw_pub= nh.advertise<std_msgs::Float64>("/yaw", 1000);
	rear_right_roll_pub = nh.advertise<std_msgs::Float64>("/rear_right_roll", 100);
	rear_axis_posX_pub = nh.advertise<std_msgs::Float64>("/rear_axis_posX", 1000);
	rear_axis_posY_pub = nh.advertise<std_msgs::Float64>("rear_axis_posY", 1000);

	drive_mode_control_pub = nh.advertise<std_msgs::Int32>("/drive_mode_control", 1000);
	velocity_control_pub = nh.advertise<std_msgs::Float64>("/velocity_control", 1000);
	steering_angle_pub = nh.advertise<std_msgs::Float64>("/steering_angle", 1000);
	steering_control_pub = nh.advertise<std_msgs::Float64>("/steering_control", 1000);

	maneuver_time_pub = nh.advertise<std_msgs::Float64>("/maneuver_time", 1000);

	// USR
	us_pub_vr = nh.advertise < sensor_msgs::Range > ("/sensor_dist_vr", 10);
		
	//lidar
	lidar_pub = nh.advertise<sensor_msgs::LaserScan>("/scan", 50);
	
	//Subscriber
	ROS_INFO("advertised publisher");
	drive_mode_control_sub = nh.subscribe("/drive_mode_control", 1000, control_drive_mode_cb);
	steering_control_sub = nh.subscribe("/steering_control", 1000, control_steer_cb);
	velocity_control_sub = nh.subscribe("/velocity_control", 1000, control_velocity_cb);
	ROS_INFO("subscribing");

}

void rear_right_roll_publisher(void) { 
	std_msgs::Float64  msg;
	msg.data = Car.Tire[3].rot;	
	rear_right_roll_pub.publish(msg);
	return;	

}

void rear_axis_posX_publisher(void) { 
	std_msgs::Float64  msg;
	msg.data = Vehicle.FrX.t_0[0];	
	rear_axis_posX_pub.publish(msg);
	return;	
}

void rear_axis_posY_publisher(void) { 
	std_msgs::Float64  msg;
	msg.data = Vehicle.FrX.t_0[1];	
	rear_axis_posY_pub.publish(msg);
	return;	
}

void velocity_publisher(void){

	std_msgs::Float64 msg;
	msg.data =Vehicle.v;
	velocity_pub.publish(msg);

	return;

}

void yaw_publisher(void){

	std_msgs::Float64 msg;
	msg.data =Vehicle.Yaw;
	yaw_pub.publish(msg);

	return;

}

int odom_publisher(void) {

	tf::TransformBroadcaster odom_broadcaster;

	static tf::TransformBroadcaster br;
	tf::Transform transform;
	transform.setOrigin(tf::Vector3(3.0, 0.3, -0.8));
	tf::Quaternion quaternion;
	transform.setRotation(tf::createQuaternionFromYaw(1));
	br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "base","odom"));



	double x = Vehicle.PoI_Pos[0];
	double y = Vehicle.PoI_Pos[1];
	double th = Vehicle.PoI_Pos[2];

	double vx = Vehicle.PoI_Vel_1[0];
	double vy = Vehicle.PoI_Vel_1[1];
	double vth = Vehicle.PoI_Vel_1[2];

	ros::Time current_time, last_time;
	current_time = ros::Time::now();
	last_time = ros::Time::now();

	current_time = ros::Time::now();

	//compute odometry in a typical way given the velocities of the robot
	double dt = (current_time - last_time).toSec();
	double delta_x = (vx * cos(th) - vy * sin(th)) * dt;
	double delta_y = (vx * sin(th) + vy * cos(th)) * dt;
	double delta_th = vth * dt;

	x += delta_x;
	y += delta_y;
	th += delta_th;

	//since all odometry is 6DOF we'll need a quaternion created from yaw
	geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);

	//first, we'll publish the transform over tf
	geometry_msgs::TransformStamped odom_trans;
	odom_trans.header.stamp = current_time;
	odom_trans.header.frame_id = "odom";
	odom_trans.child_frame_id = "base_link";

	odom_trans.transform.translation.x = x;
	odom_trans.transform.translation.y = y;
	odom_trans.transform.translation.z = 0.0;
	odom_trans.transform.rotation = odom_quat;

	//send the transform
	odom_broadcaster.sendTransform(odom_trans);

	//next, we'll publish the odometry message over ROS
	nav_msgs::Odometry odom;
	odom.header.stamp = current_time;
	odom.header.frame_id = "odom";

	//set the position
	odom.pose.pose.position.x = x;
	odom.pose.pose.position.y = y;
	odom.pose.pose.position.z = 0.0;
	odom.pose.pose.orientation = odom_quat;

	//set the velocity
	odom.child_frame_id = "base_link";
	odom.twist.twist.linear.x = vx;
	odom.twist.twist.linear.y = vy;
	odom.twist.twist.angular.z = vth;

	//publish the message
	odom_pub.publish(odom);
	return 0;
}

void steering_angle_publisher(void) {
	
	double pi = 3.141592654;
	double steering_transform = 16.13168724; // Transform factor from Front Truck axle to steering wheel

	std_msgs::Float64 msg;
	msg.data = (Vehicle.Steering.Ang * (180 / steering_transform)) / pi;	
	steering_angle_pub.publish(msg);

	double global_time = ros::Time::now().toSec();
	double global_time_ms = global_time * 1000;

	//std::cout << std::fixed << "Time: [s] " << int(global_time) % 1000 << "," << long(global_time_ms) % 1000 << " Pub IPG: " << msg.data << std::endl;

	return;	

}

void us_publisher(void) {

//******************************************************************************
//Sens front right
	sensor_msgs::Range range_msg_right;
	static tf::TransformBroadcaster br;
	tf::Transform transform;
	//transform.setOrigin(tf::Vector3(5.65, -1.10, 0.7));
	//transform.setRotation(tf::createQuaternionFromYaw(1));
	//br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "base_link","sensor_dist_vr"));

	br.sendTransform(
      	tf::StampedTransform(
        tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0.0, 0.0, 0.3)),
        ros::Time::now(),"base_link", "sensor_dist_vr"));

	range_msg_right.header.frame_id = "sensor_dist_vr";
	range_msg_right.header.stamp = ros::Time::now();
	range_msg_right.radiation_type = 0;
	range_msg_right.field_of_view = 0.1;
	range_msg_right.min_range = 0.00;
	range_msg_right.max_range = 6.47;
	range_msg_right.range =1+(USonicRSI->ReceiverUSonic->DetPoints->TimeOfFlight)*(343/2);
	us_pub_vr.publish(range_msg_right);

	return;
}

void lidar_publisher(void){

	size_t num_scanpoints = LidarRSI->nScanPoints;
	//ROS_INFO("%d",num_scanpoints);
	//size_t num_scanpoints = 100;
	double angle_min = 1.57; // -90 degrees
	double angle_max = -1.57;  // 90 degrees
	//double angle_increment = 0.0314;
	double angle_increment = 3.14/(num_scanpoints+1);
	double time_increment = 3.70370362361e-05;
	double range_min = 0.0;
	double range_max = 100.0;

	//ranges and intensities pointers and allocate memories.
	double *ranges = new double[num_scanpoints];
	double *intensities = new double[num_scanpoints];


	for(size_t i = 0; i < num_scanpoints/4; ++i){
		ranges[i] = (((LidarRSI->ScanPoint[i].TimeOF)/2)*0.3);
		intensities[i] = LidarRSI->ScanPoint[i].Intensity;
	}

		ros::Time scan_time;
		scan_time = ros::Time::now();

		// Create a LaserScan Message
		sensor_msgs::LaserScan scan;

		static tf::TransformBroadcaster br;
		tf::Transform transform;
				
		br.sendTransform(
      		tf::StampedTransform(
        	tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0.0, 0.0, 0.3)),
        	ros::Time::now(),"base_link", "lidar_frame"));
		
		scan.header.stamp = scan_time;
		scan.header.frame_id = "lidar_frame";
		scan.angle_min = angle_min;
		scan.angle_max = angle_max;
		//scan.angle_increment = angle_increment;
		//scan.angle_increment = 0.004;
		scan.time_increment = time_increment;
        	scan.range_min = range_min;
		scan.range_max = range_max;

		scan.ranges.resize(num_scanpoints/4);
		scan.intensities.resize(num_scanpoints/4);

		for(uint64_t i = 0; i < num_scanpoints/4; ++i){
			scan.ranges[i] = ranges[i];
			scan.intensities[i] = intensities[i];
		}

	lidar_pub.publish(scan);

	delete ranges;
	delete intensities;

	return;


}


void maneuver_time_publisher(void) { 
	std_msgs::Float64  msg;
	msg.data = DrivMan.ActualMan.Time ;	
	maneuver_time_pub.publish(msg);
	return;	

}

//Reset Topics
void reset_drive_mode_control_publisher(void){

	std_msgs::Int32 msg;
	msg.data = 2;
	drive_mode_control_pub.publish(msg);

	return;

}

void reset_velocity_control_publisher(void){

	std_msgs::Float64 msg;
	msg.data = 0.0;
	velocity_control_pub.publish(msg);

	return;

}

void reset_steering_control_publisher(void){

	std_msgs::Float64 msg;
	msg.data = 0.0;
	steering_control_pub.publish(msg);

	return;
//std::cout << "Time since last pub: " << (current_time - last_time_for_publish) << " Pub Counter: " << counter_for_publish << std::endl;
}

void reset_ros_topics()
{
	reset_drive_mode_control_publisher();
	reset_velocity_control_publisher();
	reset_steering_control_publisher();
}

int
User_Init (void)
{
	initialize_ros_comm();

	//start&connect    
	return 0;
}



int
User_Register (void)
{

    /* @@PLUGIN-BEGIN-REGISTER@@ - Automatically generated code - don't edit! */
    /* @@PLUGIN-END@@ */

    return 0;
}



/*
** User_DeclQuants ()
**
** Add user specific quantities to the dictionary
**
** Call:
** - once at program start
** - no realtime conditions
*/

void
User_DeclQuants (void)
{
    int i;

    for (i=0; i<N_USEROUTPUT; i++) {
	char sbuf[32];
	sprintf (sbuf, "UserOut_%02d", i);
	DDefDouble (NULL, sbuf, "", &User.Out[i], DVA_IO_Out);
    }
    RBS_DeclQuants();
}


/*
** User_Param_Add ()
**
** Update all modified application specific parameters in the test stand
** parameter file (ECUParameters).
**
** If the variable SimCore.TestRig.ECUParam.Modified set to 1 somewhere else
** CarMaker calls this function to let the user add or change all necessary
** entries before the file is written.
** So, if writing the ECUParam file is necessary, set ECUParam.Modified to 1.
** The next TestRun start or end, CarMaker calls this function and writes
** the file to the harddisk.
**
** Call:
** - in a separate thread (no realtime contitions)
** - when starting a new test run
*/

int
User_Param_Add (void)
{
#if defined(CM_HIL)
    /* ECU parameters */
    if (SimCore.TestRig.ECUParam.Inf == NULL)
	return -1;
#endif

    return 0;
}



/*
** User_Param_Get ()
**
** Update all modified application specific parameters from the test stand
** parameter file (ECUParameters).
**
** Call:
** - in a separate thread (no realtime conditions)
** - if User_Param_Get() wasn't called
** - when starting a new test run, if
**   - the files SimParameters and/or
**   - ECUParameters
**   are modified since last reading
**
** return values:
**  0	ok
** -1	no testrig parameter file
** -2	testrig parameter error
** -3	i/o configuration specific error
** -4	no simulation parameters
** -5	simulation parameters error
** -6	FailSafeTester parameter/init error
*/

int
User_Param_Get (void)
{
    int rv = 0;

#if defined(CM_HIL)
    /*** testrig / ECU parameters */
    if (SimCore.TestRig.ECUParam.Inf == NULL)
	return -1;

    if (IO_Param_Get(SimCore.TestRig.ECUParam.Inf) != 0)
	rv = -2;
#endif

    /*** simulation parameters */
    if (SimCore.TestRig.SimParam.Inf == NULL)
	return -4;

    return rv;
}



/*
** User_TestRun_Start_atBegin ()
**
** Special things before a new simulation starts like
** - reset user variables to their default values
** - reset counters
** - ...
**
** Call:
** - in separate thread (no realtime conditions)
** - when starting a new test run
** - after (standard) infofiles are read in
** - before reading parameters for Environment, DrivMan, Car, ...
**   the models are NOT in the simulation-can-start-now state
**   (after Start(), before StaticCond())
*/

int
User_TestRun_Start_atBegin (void)
{
    int rv = 0;
    int i;
    // Zurücksetzen der Ros Topics velocity_control und steering_control auf 0, um eine initiale Bewegung zu vermeiden 
    reset_ros_topics();

    for (i=0; i<N_USEROUTPUT; i++)
	User.Out[i] = 0.0;


    if (IO_None)
	return rv;

#if defined(CM_HIL)
    if (FST_New(SimCore.TestRig.ECUParam.Inf) != 0)
	rv = -6;
#endif

    return rv;
}




/*
** User_TestRun_Start_atEnd ()
**
** Special things before a new simulation starts like
** - reset user variables to there default values
** - reset counters
** - ...
**
** Call:
** - in separate thread (no realtime conditions)
** - when starting a new test run
** - at the end, behind reading parameters for Environment, DrivMan,
**   Car, ...
**   the models are NOT in the simulation-can-start-now state
**   (after Start(), before StaticCond())
*/

int
User_TestRun_Start_atEnd (void)

{
	                      
	
#if defined(XENO)
   	IOConf_DeclQuants();
#endif
    return 0;
}



/*
** User_TestRun_Start_StaticCond_Calc ()
**
** called in non RT context
*/

int
User_TestRun_Start_StaticCond_Calc (void)
{
    return 0;
}



/*
** User_TestRun_Start_Finalize ()
**
** called in RT context
*/

int
User_TestRun_Start_Finalize (void)
{
    return 0;
}



/*
** User_TestRun_RampUp ()
**
** Perform a smooth transition of variables (e.g. I/O)
** from their current state  to the new testrun.
** This function is called repeatedly, once during each cycle, until
** it returns true (or issues an error message), so the function should
** return true if transitioning is done, false otherwise.
**
** In case of an error the function should issue an apropriate
** error message and return false;
**
** Called in RT context, in state SCState_StartSim,
** after preprocessing is done, before starting the engine.
** Please note, that in this early initialization state no calculation
** of the vehicle model takes place.
*/

int
User_TestRun_RampUp (double dt)
{
    int IsReady = 1;

    return IsReady;
}



/*
** User_TestRun_End_First ()
**
** Invoked immediately after the end of a simulation is initiated,
** but before data storage ends and before transitioning into SCState_Idle.
** - Send Scratchpad-note
** - ...
**
** Call:
** - in main task, in the main loop (real-time conditions!)
** - when a test run is finished (SimCore.State is SCState_End)
*/

int
User_TestRun_End_First (void)
{
    return 0;
}



/*
** User_TestRun_End ()
**
** Special things after the end of a simulation like
** - switch off an air compressor
** - Write something to a file
** - ...
**
** Call:
** - in separate thread (no realtime conditions)
** - when a test run is finished (SimCore.State is SCState_End<xyz>)
*/

int
User_TestRun_End (void)
{
    return 0;
}



/*
** User_In ()
**
** Assign quantities of the i/o vector to model variables
**
** Call:
** - in the main loop
** - pay attention to realtime condition
** - just after IO_In()
*/

void
User_In (const unsigned CycleNo)
{
    if (SimCore.State != SCState_Simulate)
	return;
}



/*
** User_DrivMan_Calc ()
**
** called
** - in RT context
** - after DrivMan_Calc()
*/

int
User_DrivMan_Calc (double dt)
{
    /* Rely on the Vehicle Operator within DrivMan module to get
       the vehicle in driving state using the IPG's
       PowerTrain Control model 'Generic' or similar */
    if (Vehicle.OperationState != OperState_Driving)
	return 0;

    return 0;
}


/*
** User_VehicleControl_Calc ()
**
** called
** - in RT context
** - after VehicleControl_Calc()
*/

int
User_VehicleControl_Calc (double dt)
{
    /* Rely on the Vehicle Operator within DrivMan module to get
       the vehicle in driving state using the IPG's
       PowerTrain Control model 'Generic' or similar */
    if (Vehicle.OperationState != OperState_Driving)
	return 0;

    return 0;
}



/*
** User_Brake_Calc ()
**
** called
** - in RT context
** - after Brake_Calc() in Vhcl_Calc()
*/

int
User_Brake_Calc (double dt)
{
    /* Modify the total brake torque from the brake system model Brake.Trq_tot[]
       or the target drive source torque from the brake control unit
       Brake.HydBrakeCU_IF.Trq_DriveSrc_trg[]
    */

    return 0;
}



/*
** User_Traffic_Calc ()
**
** called
** - in RT context
** - after Traffic_Calc()
*/

int
User_Traffic_Calc (double dt)
{
    if (SimCore.State != SCState_Simulate)
	return 0;

    return 0;
}



/*
** User_Calc ()
**
** called in RT context
*/

int
User_Calc (double dt)
{
    /* Starting with CM 6.0 User_Calc() will be invoked in EVERY simulation
       state. Uncomment the following line in order to restore the behaviour
       of CM 5.1 and earlier. */
    /*if (!UserCalcCalledByAppTestRunCalc) return 0;*/

    return 0;
}



/*
** User_Check_IsIdle ()
**
** Checking, if the simulation model is in idle conditions (stand still,
** steeringwheel angle zero, cluch pedal pressed, ...).
** If reached idle state, the calculation of vehicle model and driving
** manoevers is stopped.
** Ready for start new simulation.
**
** Return:
** 1  idle state reached
** 0  else
**
** Call:
** - in main task, in the main loop
** - pay attention to realtime condition
** - while SimCore.State==SCState_EndIdleGet
*/

int
User_Check_IsIdle (int IsIdle)
{
    double val;

    /*** ECU / carmodel signals */

    /* vehicle and wheels: stand still */
    val = 0.5*kmh2ms;
    if (Vehicle.v > val
     || fabs(Vehicle.Wheel[0]->vBelt) > val || fabs(Vehicle.Wheel[1]->vBelt) > val
     || fabs(Vehicle.Wheel[2]->vBelt) > val || fabs(Vehicle.Wheel[3]->vBelt) > val) {
	IsIdle = 0;
    }

    /* SteerAngle: drive  straight forward position */
    val = 1.0*deg2rad;
    if (Vehicle.Steering.Ang > val || Vehicle.Steering.Ang < -val)
	IsIdle = 0;

    return IsIdle;
}



/*
** User_Out ()
**
** Assigns model quantities to variables of the i/o vector
**
** call:
** - in the main loop
** - pay attention to realtime condition
** - just before IO_Out();
*/

void
User_Out (const unsigned CycleNo)
{
    RBS_OutMap(CycleNo);
#if defined(XENO)
   IOConf_OutMap();
#endif

    if (SimCore.State != SCState_Simulate)
	return;

    //ros::Rate loop_rate(60);
	current_time = DrivMan.ActualMan.Time;
	

	bool publish_values = counter_for_publish == 50;


	if (publish_values) {
		
		//std::cout << "Time since last pub: " << (current_time - last_time_for_publish) << " Pub Counter: " << counter_for_publish << std::endl;
		
		odom_publisher();
		velocity_publisher();
		rear_right_roll_publisher();
		yaw_publisher();
		us_publisher();
		rear_axis_posX_publisher();
		rear_axis_posY_publisher();
		steering_angle_publisher();
		maneuver_time_publisher();
		lidar_publisher();
		
		last_time_for_publish = DrivMan.ActualMan.Time;
		counter_for_publish = 0;
	}

	counter_for_publish += 1;
	

	ros::spinOnce();
}



/*
** User_ApoMsg_Eval ()
**
** Communication between the application and connected GUIs.
** Evaluate messages from GUIs
**
** Call:
** - in the main loop
** - pay attention to realtime condition
** - near the end of the main loop, if the function SimCore_ApoMsg_Eval()
**    skips the message
**
** Return:
**   0 : message evaluated
**  -1 : message not handled
*/

int
User_ApoMsg_Eval (int Ch, char *Msg, int len, int who)
{
#if defined(CM_HIL)
    /*** FailSafeTester */
    if (Ch == ApoCh_CarMaker) {
	if (FST_ApoMsgEval(Ch, Msg, len) <= 0)
	    return 0;
    }
#endif
    return -1;
}



/*
** User_ApoMsg_Send ()
**
** Communication between the application and connected GUIs.
** Sends messages to GUIs
**
** Call:
** - near the end of the main loop, in MainThread_FinishCycle()
** - pay attention to realtime condition
*/

void
User_ApoMsg_Send (double T, const unsigned CycleNo)
{
}



/*
** User_ShutDown ()
**
** Prepare application for shut down
**
** Call:
** - at end of program
** - no realtime conditions
*/

int
User_ShutDown (int ShutDownForced)
{
    int IsDown = 0;

    /* Prepare application for shutdown and return that
       shutdown conditions are reached */
    if (1) {
	IsDown = 1;
    }

    return IsDown;
}



/*
** User_End ()
**
** End all models of the user module
**
** Call:
** - one times at end of program
** - no realtime conditions
*/

int
User_End (void)
{
    return 0;
}



/*
** User_Cleanup ()
**
** Cleanup function of the User module
**
** Call:
** - one times at end of program, just before exit
** - no realtime conditions
*/

void
User_Cleanup (void)
{
}
