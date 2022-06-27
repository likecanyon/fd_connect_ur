#include "ros/ros.h"

#include "haptic_ros_driver/HapticDevice.h"
#include <ur_rtde/rtde_control_interface.h>
#include <ur_rtde/rtde_receive_interface.h>
#include <thread>
#include <chrono>
#include "haptic_ros_driver/dhdc.h"
#include <Eigen/Dense>
#include <cmath>
#include "iir_filters/Iir.h"
#include <fstream>
using namespace ur_rtde;
using namespace std::chrono;

/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */
int main(int argc, char *argv[])
{
	std::ofstream outfile1;
	std::ofstream outfile2;
	outfile1.open("afile1.dat");
	outfile2.open("afile2.dat");

	ros::init(argc, argv, "haptic_ros_driver");

	ros::NodeHandle nh("~");

	//连接UR
	RTDEControlInterface rtde_control("192.168.3.101");
	RTDEReceiveInterface rtde_receive("192.168.3.101");

	HapticDevice haptic_dev(nh, false); //实例化HapticDevice，haptic_dev是一个HapticDevice类的对象


	std::vector<double> TcpPose = rtde_receive.getActualTCPPose();
	std::vector<double> TcpPoseInit = rtde_receive.getActualTCPPose();
	std::vector<std::vector<double>> FBuffer(3, std::vector<double>(5, 0.0));
	std::vector<double> FBufferAve(3, 0.0);
	std::vector<double> ForceOffset{-0.237403,-0.0920403,-0.182516};
	int button0_state_ = dhdGetButton(0);

	std::vector<double> InitQ{0.0, -1.57, -1.57, -1.57, 1.57, 0}; // home:0,-90,0,-90,0,0
	rtde_control.moveJ(InitQ);
	haptic_dev.Start();

	ros::AsyncSpinner spinner(3);
	spinner.start();

	haptic_dev.keep_alive_ = true;

	haptic_dev.keep_alive_;

	double current_position[3] = {0.0, 0.0, 0.0};
	double current_position_Init[3] = {0.0, 0.0, 0.0};
	double current_orientRad[3] = {0.0, 0.0, 0.0};
	double current_orientRad_Init[3] = {0.0, 0.0, 0.0};
	double Omega_matrix[3][3];
	double Omega_matrixInit[3][3];
	Eigen::Matrix3d Omega_matrixEigen;
	Eigen::Matrix3d Omega_matrixInitEigen;
	Eigen::Matrix3d DeltaR;
	Eigen::Matrix3d Tcp_Rotation;
	Eigen::Vector3d axis;
	double alpha{0.0};
	Eigen::Vector3d axisInit;
	double alphaInit{0.0};
	Eigen::Matrix3d Tcp_Rotation_Init;
	Eigen::AngleAxisd K;
	Eigen::Vector3d Khat;
	std::vector<double> TcpForce(6, 0.0);

	// filter function
	//
	//  init filter
	float fx, fy, fz;
	float scaling = 0.1;
	Iir::Butterworth::LowPass<2> f1, f2, f3;	 // NOTE： here order should replaced by a int number!
	const float samplingrate = 200;				 // Hz
	const float cutoff_frequency = 5;			 // Hz
	f1.setup(2, samplingrate, cutoff_frequency); // NOTE： here order should replaced by a int number!
	f2.setup(2, samplingrate, cutoff_frequency); // NOTE： here order should replaced by a int number!
	f3.setup(2, samplingrate, cutoff_frequency); // NOTE： here order should replaced by a int number!
	//
	//

	Tcp_Rotation_Init.setIdentity();

	double deltaZ = 0;
	bool MotionDir = true;
	while (ros::ok() && (haptic_dev.keep_alive_ == true))
	{

		button0_state_ = dhdGetButton(0);
		if (button0_state_)

		{
			//获得omega的新位姿
			dhdGetPosition(&current_position[0], &current_position[1], &current_position[2]);
			dhdGetOrientationFrame(Omega_matrix);
			TcpForce = rtde_receive.getActualTCPForce();
			for (int i = 0; i < 3; i++)
			{
				TcpForce[i] = TcpForce[i] - ForceOffset[i];
			}
			outfile1 << TcpForce[0] << " " << TcpForce[1] << " " << TcpForce[2] << std::endl;

			// Realtime filtering sample by sample
			fx = f1.filter(TcpForce[0]) * scaling;
			fy = f2.filter(TcpForce[1]) * scaling;
			fz = f3.filter(TcpForce[2]) * scaling;
			outfile2 << fx << " " << fy << " " << fz << std::endl;

			FBuffer[0][0] = FBuffer[0][1];
			FBuffer[0][1] = FBuffer[0][2];
			FBuffer[0][2] = FBuffer[0][3];
			FBuffer[0][3] = FBuffer[0][4];
			// FBuffer[0][4] = FBuffer[0][5];
			// FBuffer[0][5] = FBuffer[0][6];
			FBuffer[0][4] = TcpForce[0];

			FBuffer[1][0] = FBuffer[1][1];
			FBuffer[1][1] = FBuffer[1][2];
			FBuffer[1][2] = FBuffer[1][3];
			FBuffer[1][3] = FBuffer[1][4];
			// FBuffer[1][4] = FBuffer[1][5];
			// FBuffer[1][5] = FBuffer[1][6];
			FBuffer[1][4] = TcpForce[1];

			FBuffer[2][0] = FBuffer[2][1];
			FBuffer[2][1] = FBuffer[2][2];
			FBuffer[2][2] = FBuffer[2][3];
			FBuffer[2][3] = FBuffer[2][4];
			// FBuffer[2][4] = FBuffer[2][5];
			// FBuffer[2][5] = FBuffer[2][6];
			FBuffer[2][4] = TcpForce[2];
			for (int i = 0; i < 3; i++)
			{
				double sum = 0;

				for (int j = 0; j < 5; j++)
				{
					sum = sum + FBuffer[i][j];
				}
				sum = sum / 5;
				FBufferAve[i] = {sum};
			}
			// std::cout << "the TcpFore is :" << TcpForce[0] << " " << TcpForce[1] << " " << TcpForce[2] << std::endl;
			//计算位移
			TcpPose[2] = TcpPoseInit[2] + current_position[2] - current_position_Init[2];
			TcpPose[1] = TcpPoseInit[1] + current_position[1] - current_position_Init[1];
			TcpPose[0] = TcpPoseInit[0] + current_position[0] - current_position_Init[0];
			//计算姿态
			//首先计算deltaR，deltaR=omega新位姿*omegaInit位姿的转置

			Omega_matrixEigen << Omega_matrix[0][0], Omega_matrix[0][1], Omega_matrix[0][2],
				Omega_matrix[1][0], Omega_matrix[1][1], Omega_matrix[1][2],
				Omega_matrix[2][0], Omega_matrix[2][1], Omega_matrix[2][2];

			DeltaR = Omega_matrixEigen * Omega_matrixInitEigen.transpose();
			//新的tcp姿态=deltaR*tcpinit姿态,进而得到TcpPose[3],TcpPose[4],TcpPose[5]
			Tcp_Rotation = DeltaR * Tcp_Rotation_Init;
			K.fromRotationMatrix(Tcp_Rotation);
			Khat = K.axis();
			alpha = K.angle();
			TcpPose[3] = Khat[0] * alpha;
			TcpPose[4] = Khat[1] * alpha;
			TcpPose[5] = Khat[2] * alpha;
			//将TcpPose发给机器人，机器人开始运动

			// if (FBuffer[0][0] != 0)
			{
				// dhdSetForce(FBufferAve[0], FBufferAve[1], FBufferAve[2], -1);
				dhdSetForce(fx, fy, fz, -1);
			}
			// dhdSetForce(TcpForce[0], TcpForce[1], TcpForce[2], -1);

			rtde_control.servoL(TcpPose, 0, 0, 0.005, 0.05, 300);
		}
		else
		{
			//获得Init数据，机器人TCP和主手位姿
			dhdGetPosition(&current_position_Init[0], &current_position_Init[1], &current_position_Init[2]);
			TcpPoseInit = rtde_receive.getActualTCPPose();
			dhdGetOrientationFrame(Omega_matrixInit);
			Omega_matrixInitEigen << Omega_matrixInit[0][0], Omega_matrixInit[0][1], Omega_matrixInit[0][2],
				Omega_matrixInit[1][0], Omega_matrixInit[1][1], Omega_matrixInit[1][2],
				Omega_matrixInit[2][0], Omega_matrixInit[2][1], Omega_matrixInit[2][2];

			//计算机器人Init姿态
			axisInit = {TcpPoseInit[3], TcpPoseInit[4], TcpPoseInit[5]};
			axisInit.normalize();
			alphaInit = TcpPoseInit[3] / axisInit[0];
			Tcp_Rotation_Init = Eigen::AngleAxisd(alphaInit, axisInit);
			dhdSetForceAndGripperForce(0, 0, 0, 0.0);

			// std::cout << "current_position_Init is :" << current_position_Init[0] << " " << current_position_Init[1] << " " << current_position_Init[2] << std::endl;
		}

		ros::Duration(0.005).sleep();
	}
	// process finished.
	haptic_dev.keep_alive_ = false;
	spinner.stop();

	return 0;
}
