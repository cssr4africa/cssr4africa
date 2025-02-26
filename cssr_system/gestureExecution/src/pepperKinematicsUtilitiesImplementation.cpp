/*   pepperKinematicsUtilities.cpp
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

#include <gestureExecution/pepperKinematicsUtilitiesInterface.h>





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
double degrees(double radians){
   double degrees = radians * (double) 180.0 / (double) M_PI; // David Vernon ... cast to float
   return degrees;
}

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
double radians(double degrees){
   double radians = degrees / ((double) 180.0 / (double) M_PI); // David Vernon ... cast to float
   return radians;
}
 




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
void get_elbow_position(int arm, double theta_1, double theta_2, double* position){
   // Define the lengths of the arm segments
   double l_1 = -57.0;
   double l_2 = -149.74;
   double l_3 = 86.82;
   double l_4 = 181.2;
   double l_5 = -15.0;
   double l_6 = 0.13;

   // Update the lengths of the arm segments based on the arm
   if (arm == RIGHT_ARM){
      l_2 = -149.74;
      l_5 = -15.0;
   }
   else if (arm == LEFT_ARM){
      l_2 = 149.74;
      l_5 = 15.0;
   }

   // Obtain the trigonometric values of the angles
   double sin_theta_1 = sin(theta_1);
   double cos_theta_1 = cos(theta_1);
   double sin_theta_2 = sin(theta_2);
   double cos_theta_2 = cos(theta_2);

   // Estimate the x position of the elbow
   double f_1 = l_6 * sin_theta_1;
   double f_2 = l_4 * cos_theta_2;
   double f_3 = l_5 * sin_theta_2;
   double f_4 = f_2 - f_3;
   double f_5 = cos_theta_1 * f_4;
   double position_x = l_1 + f_1 + f_5;

   // Estimate the y position of the elbow
   f_1 = l_5 * cos_theta_2;
   f_2 = l_4 * sin_theta_2;
   double position_y = l_2 + f_1 + f_2;

   // Estimate the z position of the elbow
   f_1 = l_6 * cos_theta_1;
   f_2 = sin_theta_1 * f_4;
   double position_z = l_3 + f_1 - f_2;

   // Update the position array with the calculated values
   position[0] = position_x;
   position[1] = position_y;
   position[2] = position_z;
}

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
void get_wrist_position(int arm, double theta_1, double theta_2, double* position){
   // Define the lengths of the arm segments
   double l_1 = -57.0;
   double l_2 = -57.0;
   double l_3 = 86.82;
   double l_4 = 181.2;
   double l_5 = -57.0;
   double l_6 = 0.13;

   // Update the lengths of the arm segments based on the arm
   if (arm == RIGHT_ARM){
      l_2 = -149.74;
      l_5 = -15.0;
   }
   else if (arm == LEFT_ARM){
      l_2 = 149.74;
      l_5 = 15.0;
   }

   // Obtain the trigonometric values of the angles
   double sin_theta_1 = sin(theta_1);
   double cos_theta_1 = cos(theta_1);
   double sin_theta_2 = sin(theta_2);
   double cos_theta_2 = cos(theta_2);

   // Estimate the x position of the wrist
   double f_1 = l_6 * sin_theta_1;
   double f_2 = l_4 * cos_theta_2;
   double f_3 = l_5 * sin_theta_2;
   double f_4 = f_2 - f_3;
   double f_5 = cos_theta_1 * f_4;
   double position_x = l_1 + f_1 + f_5;

   // Estimate the y position of the wrist
   f_1 = l_5 * cos_theta_2;
   f_2 = l_4 * sin_theta_2;
   double position_y = l_2 + f_1 + f_2;

   // Estimate the z position of the wrist
   f_1 = l_6 * cos_theta_1;
   f_2 = sin_theta_1 * f_4;
   double position_z = l_3 + f_1 - f_2;

   // Update the position array with the calculated values
   position[0] = position_x;
   position[1] = position_y;
   position[2] = position_z;

   // // # formulas derived from the FK matrix 
   // position[0] = l1 + l6*sin(theta1) + (cos(theta1)*(l4*cos(theta2)) - l5*sin(theta2));
   // position[1] = l2 + l5*cos(theta2) + l4*sin(theta2);
   // position[2] = l3 + l6*cos(theta1) - (sin(theta1)*(l4*cos(theta2)) - l5*sin(theta2));

}

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
void get_arm_shoulder_angles(int arm, double elbow_x, double elbow_y, double elbow_z, double* theta_1, double* theta_2){
   // Define the lengths of the arm segments
   double l_1 = -57.0;
   double l_2 = 0.0;
   double l_3 = 86.82;
   double l_4 = 181.2;
   double l_5 = 0.0;
   double l_6 = 0.13;

   // Define variables to store the shoulder pitch and shoulder roll angles
   double shoulder_roll = 0.0;
   double shoulder_pitch = 0.0;

   // Update the lengths of the arm segments based on the arm
   if (arm == RIGHT_ARM){
      l_2 = -149.74;
      l_5 = -15.0;
   }
   else if (arm == LEFT_ARM){
      l_2 = 149.74;
      l_5 = 15.0;
   } 

   // Calculate the shoulder_roll or theta_2 
   double f_1 = elbow_y - l_2;
   double f_2 = sqrt(pow(l_4, 2) + pow(l_5, 2));
   double f_3 = asin(f_1 / f_2);
   double f_4 = atan(l_5 / l_4);
   double t_2_temp = f_3 - f_4;

   if (arm == RIGHT_ARM){
      if ((t_2_temp + f_4) > (-PI/2 - f_4)){
         shoulder_roll = t_2_temp;
      }
      else{
         shoulder_roll = -PI - f_3 - f_4;
         if (shoulder_roll < -1.5630){
               shoulder_roll = t_2_temp;
         }
      }
      // check if solution is within Peppers range
      if ((shoulder_roll < -1.58) || (shoulder_roll >= -0.0087)){
         shoulder_roll = -0.0087;
      }
   }

   else if (arm == LEFT_ARM){
      if (t_2_temp + f_4 < PI/2 - f_4){
         shoulder_roll = t_2_temp;
      }
      else{
         shoulder_roll = PI - f_3 - f_4;
         if (shoulder_roll > 1.5630){
               shoulder_roll = t_2_temp;
         }
      }
      // check if solution is within Peppers range
      if ((shoulder_roll > 1.58) || (shoulder_roll <= 0.0087)){
         shoulder_roll = 0.0087;
      }
   }

   // Calculate shoulder_pitch or theta_1
   double n = (l_4 * cos(shoulder_roll)) - (l_5 * sin(shoulder_roll));
   f_1 = elbow_x - l_1;
   f_2 = elbow_z - l_3;
   f_3 = atan2(f_1, f_2);
   f_4 = pow(f_1, 2) + pow(f_2, 2) - pow(l_6, 2);
   f_4 = sqrt(f_4);
   f_4 = atan2(f_4, l_6);
   double t_1_1 = f_3 - f_4;
   f_3 = (l_6 * f_1) - (n * f_2);
   f_4 = (l_6 * f_2) + (n * f_1);
   double t_1_2 = atan2(f_3, f_4);

   // check if solution is within Peppers range
   if ((t_1_1 < -2.1) || (t_1_1 > 2.1)){
      t_1_1 = NAN;
   }

   if ((t_1_2 < -2.1) || (t_1_2 > 2.1)){
      t_1_2 = NAN;
   }

   // calculate the position of the elbow using the two solutions obtained for theta_1 above
   double positions_1[3];
   double positions_2[3];
   get_elbow_position(arm, t_1_1, shoulder_roll, positions_1);
   get_elbow_position(arm, t_1_2, shoulder_roll, positions_2);

   // check which of the solutions is closer to the position of the elbow 
   double dist_1 = sqrt(pow((positions_1[0] - elbow_x), 2) + pow((positions_1[1] - elbow_y), 2) + pow((positions_1[2] - elbow_z), 2));
   double dist_2 = sqrt(pow((positions_2[0] - elbow_x), 2) + pow((positions_2[1] - elbow_y), 2) + pow((positions_2[2] - elbow_z), 2));

   if ((dist_1 <= dist_2) || (isnan(dist_2))){
      shoulder_pitch = t_1_1;
   }
   else if ((dist_1 > dist_2) || (isnan(dist_1))){
      shoulder_pitch = t_1_2;
   }

   // update the values of the shoulder pitch and shoulder roll   
   *theta_1 = shoulder_pitch;
   *theta_2 = shoulder_roll;
}

        
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
void get_arm_elbow_roll_angle(int arm, double shoulder_pitch, double shoulder_roll, double wrist_x, double wrist_y, double wrist_z, double* elbow_roll){
   // Define the lengths of the arm segments
   double l_1 = -57.0;
   double l_2 = -149.74;
   double l_3 = 86.82;
   double l_4 = 150;
   double d_3 = 181.2;
   double z_3 = 0.13;
   double t_2 = shoulder_roll;
   double alpha = radians(9);    // 9 degrees

   // Update the lengths of the arm segments based on the arm
   if (arm == RIGHT_ARM){
      l_2 = -149.74;
   }
   else if (arm == LEFT_ARM){
      l_2 = 149.74;
   }

   // Calculate the ElbowRoll angle
   t_2 = t_2 - M_PI/2;
   double term_3 = ((wrist_x - l_1) * sin(shoulder_pitch)) + ((wrist_z - l_3) * cos(shoulder_pitch)) - z_3;
   double term_4 = ((wrist_z - l_3) * sin(shoulder_pitch) * sin(t_2)) + ((wrist_y - l_2) * cos(t_2)) - d_3 - ((wrist_x - l_1) * sin(t_2) * cos(shoulder_pitch));
   double term_2 = (sin(alpha) * term_3) + (cos(alpha) * term_4);
   double term_1 = (1.0 / l_4) * term_2;

   if (term_1 > 1.0){
      *elbow_roll = NAN;
   }
   else{
      if (arm == RIGHT_ARM){
         *elbow_roll = acos(term_1);
      }
      else if (arm == LEFT_ARM){
         *elbow_roll = -acos(term_1);
      }
   }

   // check if solution is within Pepper's range
   if (arm == RIGHT_ARM){
      if ((*elbow_roll > 1.58) || (*elbow_roll < 0.0087) || isnan(*elbow_roll)){
         *elbow_roll = 0.0087;
      }
   }
   else if (arm == LEFT_ARM){
      if ((*elbow_roll > -0.0087) || (*elbow_roll < -1.58) || isnan(*elbow_roll)){
         *elbow_roll = -0.0087;
      }
   }
}

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
void get_arm_elbow_yaw_angle(int arm, double shoulder_pitch, double shoulder_roll, double elbow_roll, double wrist_x, double wrist_y, double wrist_z, double* elbow_yaw){
   // Define the lengths of the arm segments
   double l_1 = -57.0;
   double l_2 = -149.74;
   double l_3 = 86.82;
   double a_3 = 15.0;
   double d_3 = 181.2;
   double d_5 = 150;
   double t_2 = shoulder_roll;
   double alpha = radians(9);    // 9 degrees

   // Update the lengths of the arm segments based on the arm
   if (arm == RIGHT_ARM){
      l_2 = -149.74;
      a_3 = 15.0;
   }
   else if (arm == LEFT_ARM){
      l_2 = 149.74;
      a_3 = -15.0;
   }

   // Calculate the ElbowYaw angle
   t_2 = t_2 - M_PI/2;

   complex<double> a_term = (d_3 + (d_5 * cos(alpha) * cos(elbow_roll)) + (cos(shoulder_pitch) * sin(t_2) * (wrist_x - l_1)) - (sin(shoulder_pitch) * sin(t_2) * (wrist_z - l_3)) - (cos(t_2) * (wrist_y - l_2))) / (d_5 * sin(elbow_roll) * sin(alpha));
   complex<double> b_term = ((cos(t_2) * sin(shoulder_pitch) * (wrist_z - l_3)) + a_3 - (cos(shoulder_pitch) * cos(t_2) * (wrist_x - l_1)) - (sin(t_2) * (wrist_y - l_2))) / (d_5 * sin(elbow_roll));

   if(imag(a_term) == 0 && imag(b_term) == 0){
      *elbow_yaw = atan2(real(a_term), real(b_term));
   }
   else{
      *elbow_yaw = 0.0;
   }
}


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
void get_arm_elbow_angles(int arm, double shoulder_pitch, double shoulder_roll, double wrist_x, double wrist_y, double wrist_z, double* elbow_yaw, double* elbow_roll){
   // Calculate the elbow roll and elbow yaw angles
   get_arm_elbow_roll_angle(arm, shoulder_pitch, shoulder_roll, wrist_x, wrist_y, wrist_z, elbow_roll);
   get_arm_elbow_yaw_angle(arm, shoulder_pitch, shoulder_roll, *elbow_roll, wrist_x, wrist_y, wrist_z, elbow_yaw);
}

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
void get_arm_angles(int arm, double elbow_x, double elbow_y, double elbow_z, double wrist_x, double wrist_y, double wrist_z, double* shoulder_pitch, double* shoulder_roll, double* elbow_yaw, double* elbow_roll){
   // Calculate the shoulder pitch and shoulder roll angles
   get_arm_shoulder_angles(arm, elbow_x, elbow_y, elbow_z, shoulder_pitch, shoulder_roll);

   // Calculate the elbow yaw and elbow roll angles - uncomment if need to calculate these angles
   // get_arm_elbow_angles(arm, *shoulder_pitch, *shoulder_roll, wrist_x, wrist_y, wrist_z, elbow_yaw, elbow_roll);
}

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
void get_head_angles(double camera_x, double camera_y, double camera_z, double* head_yaw, double* head_pitch){
   // Define the lengths of the head chain
   double l_1 = -38.0;
   double l_2 = 169.9;
   double l_3 = 93.6;
   double l_4 = 61.6;

   // Update the head yaw and head pitch angles
   *head_yaw = atan2(camera_y, (camera_x - l_1));
   *head_pitch = asin((l_2 - camera_z) / sqrt(pow(l_4, 2) + pow(l_3, 2))) + atan(l_4 / l_3);

   // Check if the calculated angles fall within Pepper's range, if not set the angles to 0
   if ((isnan(*head_yaw)) || (*head_yaw < -2.1) || (*head_yaw > 2.1)){
      *head_yaw = 0.0;
   }
   // if (isnan(*headPitch) || *headPitch < -0.71 || *headPitch > 0.45)
   if ((isnan(*head_pitch)) || (*head_pitch < -0.71) || (*head_pitch > 0.6371)){
      *head_pitch = 0.0;
   }
}
