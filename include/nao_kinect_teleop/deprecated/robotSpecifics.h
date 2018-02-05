/*                                                                               
 * Copyright 2016
 * J.Avalos, S.Cortez, O.Ramos. 
 * Universidad de Ingenieria y Tecnologia - UTEC
 *
 * This file is part of nao_kinect_teleop.
 * nao_kinect_teleop is free software: you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public License
 * as published by the Free Software Foundation, either version 3 of
 * the License, or (at your option) any later version.
 * nao_kinect_teleop is distributed in the hope that it will be
 * useful, but WITHOUT ANY WARRANTY; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU Lesser General Public License for more details. You should
 * have received a copy of the GNU Lesser General Public License along
 * with nao_kinect_teleop. If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef ROBOT_SPECIFICS_H
#define ROBOT_SPECIFICS_H


// Initial configuration with both hands to the front
const double NAO_INIT_CONFIG1[42] = {
  0.0, 0.0,
  0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
  1.15,  0.10, -1.4, -0.79, 0.0, 
  0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
  0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
  1.15, -0.10,  1.4,  0.79, 0.0,
  0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0
};

/// Map from joint sensor (read from joint_states) to the reduced RBDL parsed
/// model (excluding fingers)
const unsigned int jidx[26] = 
{
  0, 1, 8, 9, 10, 11, 12, 19, 2, 3, 4, 5, 6, 7,
  22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 39
};

/// Map from full RBDL model to the reduced RBDL model (excluding fingers).
/// Indicate a 'finger' with the [arbitrary] value 100.
const unsigned int ridx[42] = 
{
  0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 
  100, 100, 100, 100, 100, 100, 13, 100, 100, 
  14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24,
  100, 100, 100, 100, 100, 100, 25, 100, 100
};


// Map of joints to names for nao (TODO: Check this)
const unsigned int RWRIST = 33;
const unsigned int LWRIST = 13;
const unsigned int RELBOW = 31;
const unsigned int LELBOW = 11;
const unsigned int RANKLE = 28;
const unsigned int LANKLE = 8;
const unsigned int HEAD = 2;

const unsigned int RGRIPPER = 40;
const unsigned int LGRIPPER = 20;

const std::string FLOATING_BASE_NAMES[6] 
      = {"PX", "PY", "PZ", "RX", "RY", "RZ"};

#endif
