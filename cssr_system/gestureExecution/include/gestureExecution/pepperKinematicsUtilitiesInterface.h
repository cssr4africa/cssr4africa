/*  pepperKinematicsUtilitiesInterface.h
*
*
* Author: Adedayo Akinade
* Date: January 06, 2024
* Version: v1.0
*
* Copyright (C) 2023 CSSR4Africa Consortium
*
* This project is funded by the African Engineering and Technology Network (Afretec)
* Inclusive Digital Transformation Research Grant Programme.
*
* Website: www.cssr4africa.org
*
* This program comes with ABSOLUTELY NO WARRANTY.
*/

#ifndef PEPPER_KINEMATICS_UTILITIES_INTERFACE_H
#define PEPPER_KINEMATICS_UTILITIES_INTERFACE_H

#define ROS  

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <cmath>  
#include <ctype.h>
#include <iostream>
#include <math.h>
#include <vector>
#include <complex>

using namespace std;


#define LEFT_ARM                0  // Left arm 
#define RIGHT_ARM               1  // Right arm

// Pepper robot links lengths (mm)
#define ELBOW_OFFSET_Y          15.0
#define ELBOW_OFFSET_Z          0.13
#define SHOULDER_OFFSET_X       -57.0
#define SHOULDER_OFFSET_Y       149.74
#define SHOULDER_OFFSET_Z       86.2
#define UPPER_ARM_LENGTH        181.20
#define LOWER_ARM_LENGTH        150.0
#define HAND_OFFSET_X           69.5
#define HAND_OFFSET_Y           23.60
#define HAND_OFFSET_Z           30.3

#define TORSO_HEIGHT            820.0

#define PI M_PI      // definition of PI from math.h





/*  --------------------------------------------------
            CONVERSION FUNCTIONS 
    -------------------------------------------------- 
*/

/* 
 *   Function to convert radians to degrees
 *   This function converts the angle in radians to degrees
 *
 * @param:
 *   radians: the angle in radians
 *
 * @return:
 *   the angle in degrees
 */
double degrees(double radians);

/* 
 *   Function to convert degrees to radians
 *   This function converts the angle in degrees to radians
 *
 * @param:
 *   degrees: the angle in degrees
 *
 * @return:
 *   the angle in radians
 */
double radians(double degrees);





/*  --------------------------------------------------
            INVERSE KINEMATICS FUNCTIONS 
    -------------------------------------------------- 
*/

/* 
 *   Function to get the position of the partial arm end-effector (elbow)
 *   given the angles of the shoulder pitch and shoulder roll. This function calculates the 3-D position of the elbow
 *   based on the forward kinematics of the arm chain. This is updated in the position array.
 *
 * @param:
 *   arm: the arm to calculate the position for (RIGHT_ARM or LEFT_ARM)
 *   theta_1: the angle of the shoulder pitch
 *   theta_2: the angle of the shoulder roll
 *   position: the array to store the calculated position of the elbow
 *
 * @return:
 *   None
 */
void get_elbow_position(int arm, double theta_1, double theta_2, double* position);

/* 
 *   Function to get the position of the arm end-effector (wrist)
 *   given the angles of the shoulder pitch and shoulder roll. This function calculates the 3-D position of the wrist
 *   based on the forward kinematics of the arm chain. This is updated in the position array.
 *
 * @param:
 *   arm: the arm to calculate the position for (RIGHT_ARM or LEFT_ARM)
 *   theta_1: the angle of the shoulder pitch
 *   theta_2: the angle of the shoulder roll
 *   position: the array to store the calculated position of the wrist
 *
 * @return:
 *   None
*/
void get_wrist_position(int arm, double theta_1, double theta_2, double* position);

/* 
 *   Function that returns the partial angles ShoulderPitch and ShoulderRoll given the 3d position of the elbow
 *   The function calculates the partial angles of the arm chain based on the position of the elbow
 *  
 * @param:
 *   arm: the arm to calculate the position for (RIGHT_ARM or LEFT_ARM)
 *   elbow_x: the x position of the elbow
 *   elbow_y: the y position of the elbow
 *   elbow_z: the z position of the elbow
 *   theta_1: the shoulder pitch angle to be updated
 *   theta_2: the shoulder roll angle to be updated
 *
 * @return:
 *   None
 */
void get_arm_shoulder_angles(int arm, double ex, double ey, double ez, double* theta1, double* theta2);

/* 
 *   Function that returns the ElbowRoll angle given the ShoulderPitch and ShoulderRoll angles and the 3d position of the wrist
 *   The function calculates the ElbowRoll angle of the arm chain based on the position of the wrist
 *
 * @param:
 *   arm: the arm to calculate the position for (RIGHT_ARM or LEFT_ARM)
 *   shoulder_pitch: the shoulder pitch angle
 *   shoulder_roll: the shoulder roll angle
 *   wrist_x: the x position of the wrist
 *   wrist_y: the y position of the wrist
 *   wrist_z: the z position of the wrist
 *   elbow_roll: the elbow roll angle to be updated
 *
 * @return:
 *   None
 */
void get_arm_elbow_roll_angle(int arm, double shoulder_pitch, double shoulder_roll, double wrist_x, double wrist_y, double wrist_z, double* elbow_roll);

/* 
 *   Function that returns the ElbowYaw angle given the ShoulderPitch, ShoulderRoll and ElbowRoll angles and the 3d position of the wrist
 *   The function calculates the ElbowYaw angle of the arm chain based on the position of the wrist
 *
 * @param:
 *   arm: the arm to calculate the position for (RIGHT_ARM or LEFT_ARM)
 *   shoulder_pitch: the shoulder pitch angle
 *   shoulder_roll: the shoulder roll angle
 *   elbow_roll: the elbow roll angle
 *   wrist_x: the x position of the wrist
 *   wrist_y: the y position of the wrist
 *   wrist_z: the z position of the wrist
 *   elbow_yaw: the elbow yaw angle to be updated
 * 
 * @return:
 *   None
 */
void get_arm_elbow_yaw_angle(int arm, double shoulder_pitch, double shoulder_roll, double elbow_roll, double wrist_x, double wrist_y, double wrist_z, double* elbow_yaw);

/* 
 *   Function that returns the ElbowYaw and ElbowRoll angles given the shoulderPitch and ShoulderRoll angles and the 3d position of the wrist position
 *   The function calculates the ElbowYaw and ElbowRoll angles of the arm chain based on the position of the wrist
 *
 * @param:
 *   arm: the arm to calculate the position for (RIGHT_ARM or LEFT_ARM)
 *   shoulder_pitch: the shoulder pitch angle
 *   shoulder_roll: the shoulder roll angle
 *   wrist_x: the x position of the wrist
 *   wrist_y: the y position of the wrist
 *   wrist_z: the z position of the wrist
 *   elbow_roll: the elbow roll angle to be updated
 *   elbow_yaw: the elbow yaw angle to be updated
 *
 * @return:
 *   None
 */
void get_arm_elbow_angles(int arm, double shoulder_pitch, double shoulder_roll, double wrist_x, double wrist_y, double wrist_z, double* elbow_yaw, double* elbow_roll);

/* 
 *   Function that returns all 4 angles of the arm chain given the 3d position of the elbow and wrist 
 *   The function calculates the ShoulderPitch, ShoulderRoll, ElbowYaw and ElbowRoll angles of the arm chain
 *   In this implementation, the ElbowYaw and ElbowRoll angles are not calculated, since they are fixed values for
 *   the Pepper robot to point with the palm facing upwards. Thus only the ShoulderPitch and ShoulderRoll angles are calculated
 *  
 * @param:
 *   arm: the arm to calculate the position for (RIGHT_ARM or LEFT_ARM)
 *   elbow_x: the x position of the elbow
 *   elbow_y: the y position of the elbow
 *   elbow_z: the z position of the elbow
 *   wrist_x: the x position of the wrist
 *   wrist_y: the y position of the wrist
 *   wrist_z: the z position of the wrist
 *   shoulderPitch: the shoulder pitch angle to be updated 
 *   shoulderRoll: the shoulder roll angle to be updated
 *   elbowYaw: the elbow yaw angle to be updated
 *   elbowRoll: the elbow roll angle to be updated
 *
 * @return:
 *   None
 */
void get_arm_angles(int arm, double elbow_x, double elbow_y, double elbow_z, double wrist_x, double wrist_y, double wrist_z, double* shoulder_pitch, double* shoulder_roll, double* elbow_yaw, double* elbow_roll);

/* 
 *   Function that returns the head angles given the head end-effector position (BottomCamera)
 *   The function calculates the head yaw and head pitch angles of the head chain
 *
 * @param:
 *   camera_x: the x position of the head end-effector
 *   camera_y: the y position of the head end-effector
 *   camera_z: the z position of the head end-effector
 *   head_yaw: the head yaw angle to be updated
 *   head_pitch: the head pitch angle to be updated
 *
 * @return:
 *   None
 */
void get_head_angles(double camera_x, double camera_y, double camera_z, double* head_yaw, double* head_pitch);




#endif // PEPER_KINEMATICS_UTILITIES_INTERFACE_H