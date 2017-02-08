# Copyright: (C) 2017 iCub Facility - Istituto Italiano di Tecnologia
# Authors: NGUYEN Dong Hai Phuong
# CopyPolicy: Released under the terms of the GNU GPL v2.0.
#
# Kinematic_Module.thrift

struct Vector {
  1: list<double> content;
} (
  yarp.name = "yarp::sig::Vector"
  yarp.includefile="yarp/sig/Vector.h"
)

/**
* Kinematic_Module_IDL
*
* IDL Interface to \ref reachingSupervisor services.
*/
service Kinematic_Module_IDL
{

  bool set_table_high(1:double _high);

  double get_table_high();

  bool set_traj_time(1: double _trajTime)

  double get_traj_time();

  bool point(1:Vector _targetPos);

  bool push(1:Vector _targetPos, 2:Vector _binLoc);

  bool resume();
   
  bool stop();


}
