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


#include <nao_kinect_teleop/tools.hpp>


JointSensors::JointSensors()
  : msg_(new sensor_msgs::JointState)
{
}

void JointSensors::readJointSensors(const sensor_msgs::JointState::ConstPtr& msg)
{
  msg_ = msg;
}

sensor_msgs::JointState::ConstPtr JointSensors::sensedValue()
{
  return msg_;
}

void JointSensors::getSensedJointsRBDL(Eigen::VectorXd& jsensed,
                                       const unsigned int& ndof)
{
  for (unsigned int i=0; i<ndof; ++i)
  {
    // The index 'jidx' maps joint names from the sensed values to the internal
    // sensed values used with RBDL model (42 dofs, joint order different)
    jsensed[jidx[i]] = msg_->position[i];
  }
}



void reducedJointModel(std::vector<float>& jreduced,
                       const Eigen::VectorXd& jfull)
{
  for (unsigned int i=0; i<jfull.size(); ++i)
  {
    if (ridx[i] != 100)
      jreduced[ridx[i]] = jfull[i];
  }
}
