#ifndef DUTYED_TF_PUB_WITH_DISTURBANCE__THRUSTER_WRENCH_HPP_
#define DUTYED_TF_PUB_WITH_DISTURBANCE__THRUSTER_WRENCH_HPP_

#include <vector>

#include "dutyed_tf_pub_with_disturbance/mmg_doyle_model.hpp"
#include "dutyed_tf_pub_with_disturbance/t200_model.hpp"

namespace njord
{
namespace sim
{

struct SimThrusterGeometry
{
  double x{0.0};
  double y{0.0};
  double angle_rad{0.0};
};

PlanarInput dutiesToPlanarInput(
  const std::vector<double> & duties,
  const std::vector<SimThrusterGeometry> & geometry,
  const T200Model & thrust_model);

PlanarInput forcesToPlanarInput(
  const std::vector<double> & forces,
  const std::vector<SimThrusterGeometry> & geometry);

}  // namespace sim
}  // namespace njord

#endif  // DUTYED_TF_PUB_WITH_DISTURBANCE__THRUSTER_WRENCH_HPP_
