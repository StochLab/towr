#ifndef TOWR_TOWR_ROS_INCLUDE_TOWR_ROS_STOCHLITE_MODEL_H_
#define TOWR_TOWR_ROS_INCLUDE_TOWR_ROS_STOCHLITE_MODEL_H_

#include <towr/models/kinematic_model.h>
#include <towr/models/single_rigid_body_dynamics.h>
#include <towr/models/endeffector_mappings.h>

namespace towr {

/**
 * @brief The Kinematics of the quadruped robot Stochlite.
 */
class StochliteKinematicModel : public KinematicModel {
public:
  StochliteKinematicModel () : KinematicModel(4)
  {
    const double x_nominal_b = 0.0 + 0.16695;
    const double y_nominal_b = 0.05 + 0.0964;
    const double z_nominal_b = -0.25;

    nominal_stance_.at(LF) <<  x_nominal_b,   y_nominal_b, z_nominal_b;
    nominal_stance_.at(RF) <<  x_nominal_b,  -y_nominal_b, z_nominal_b;
    nominal_stance_.at(LH) << -x_nominal_b,   y_nominal_b, z_nominal_b;
    nominal_stance_.at(RH) << -x_nominal_b,  -y_nominal_b, z_nominal_b;

    max_dev_from_nominal_ << 0.1, 0.1, 0.03;
  }
};

/**
 * @brief The Dynamics of the quadruped robot Stochlite.
 */
class StochliteDynamicModel : public SingleRigidBodyDynamics {
public:
  // Below is from stochlite_mpc
  // StochliteDynamicModel() : SingleRigidBodyDynamics(1.530,
  //                     0.0071959, 0.0208199, 0.0266005, 0, 0, 0,
  //                     4) {}
  // Below is from urdf
  StochliteDynamicModel() : SingleRigidBodyDynamics(1.530, //3.237,
                      0.00575867, 0.02534052, 0.03025755, 7.19E-06, 3.71E-06, 1.38E-06,
                      4) {}
};

} /* namespace towr */

#endif /* TOWR_TOWR_ROS_INCLUDE_TOWR_ROS_HYQ_MODEL_H_ */
