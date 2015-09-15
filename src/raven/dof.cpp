/* Raven 2 Control - Control software for the Raven II robot
 * Copyright (C) 2005-2012  H. Hawkeye King, Blake Hannaford, and the University of Washington BioRobotics Laboratory
 *
 * This file is part of Raven 2 Control.
 *
 * Raven 2 Control is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Raven 2 Control is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with Raven 2 Control.  If not, see <http://www.gnu.org/licenses/>.
 */

/*
 *  NOT CURRENTLY USED for mpos - state estimate is used instead
 *     Processes encoder values to motor positions
 *     processEncVal - process an encoder value from a USB packet
 *     encToJPos - go from an encoder value to a Joint position
 *
 * Kenneth Fodero
 * Biorobotics Lab
 * 2005
 *
 */

#include "dof.h"

extern struct DOF_type DOF_types[];

/**
 * processEncVal - reads an encoder value from a USB packet buffer
 *   and returns the integer result
 *
 * \param buffer[] - the USB packet buffer
 * \param channel - the encoder channel to load
 *
 * \return the resulting encoder value
 */
int processEncVal(unsigned char buffer[], int channel)
{
  int result;

  // encoder value is in bytes 3,4,and 5 of the usb in-packet
  // see atmel_code/main.c: in_packet() for more info
  result = (buffer[3*channel+5]<<16) | (buffer[3*channel+4]<<8) |
           (buffer[3*channel+3]);

  //Handle negative values by padding result with ones
  if (result >> 23) {
    result = result | 0xFF000000;
  }
#ifdef RAVEN_I
  return result;
#else
  return (-1*result);
#endif
}

/**
 * encToMPos - converts an encoder count to motor position. This function sets the mpos parameter of the joint structure
 *
 * \param joint pointer to degree of freedom to work on
 *
 */
void encToMPos(struct DOF *joint)
{
  //MPos is just the motor angle
  joint->mpos = encToMPos2(joint);

}

/**
 * encToJPos - converts a joint encoder count to joint position. This function sets the mpos parameter of the joint structure
 *
 * \param joint pointer to degree of freedom to work on
 *
 */
void encToJPos(struct DOF *joint)
{
  int normEncJ = joint->enc_val_joint - joint->enc_offset_joint;
  float joint_offset = 0;
  float enc_res = J_ENC_CNT_PER_REV; //default to safe value

  //joint offsets are explained at the end of the Kinematics White Paper
  //if (joint->type == SHOULDER)
  //  joint_offset = -25 DEG2RAD; //counts per radian


  if ((joint->type % 8 == 0) || (joint->type % 8 ==  1)){
	  enc_res = (float)J_ENC_CNT_PER_REV / 360; //counts per radian

  }
  else if (joint->type % 8 == 2)
	  enc_res = -1 * L_ENC_CNT_PER_M;

  joint->jpos_joint = (float)normEncJ / enc_res ;//+ joint_offset;

}


/**
* Returns an angle corresponding to encoder value contained in joint parameter. Function normalizes values so that they are returned measured from start position
*  \param joint Pointer to structure containing joint info
*  \return angle of the encoder
*/
float encToMPos2(struct DOF *joint)
{
  float motorAngle;
  int normEnc;

  //find the joint angle according to the joint encoder


  //Adjust encoder value - based on start position
  normEnc =  normalizeEncCnt(joint);

  //Determine motor angle
  motorAngle = (2*PI) * (1/((float)ENC_CNTS_PER_REV)) * normEnc;


  //MPos is just the motor angle
  return motorAngle;
}

/**
 * normalizeEncCnt - adjusts encVal based on end pos.
 *
 * \param dof the DOF structure being calculated
 *
 * \return normalized encoder value
 */
int normalizeEncCnt(struct DOF *joint)
{
  return (joint->enc_val - joint->enc_offset);
}

