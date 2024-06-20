#pragma once

#include <array>
#include <franka/model.h>
#include <kdl/chaindynparam.hpp>
#include <memory>
#include <string>
#include <urdf/model.h>

/**
 * Calculates poses of links and dynamic properties of the robot.
 *
 * This implementation of @ref ModelBase uses KDL as backend for calculating
 * dynamic and kinematic properties of the robot.
 */
class ModelKDL {
public:
  /**
   * Default constructor.
   * Creates an empty ModelKDL object.
   */
  ModelKDL() = default;
  /**
   * Create a new implementation for the \ref ModelBase with KDL as backend
   *
   * @param[in] model the URDF from which to interprete the kinematic chain
   * @param[in] root the link name of the root of the chain
   * @param[in] tip the link name of the tip of the chain
   * @throws std::invalid_argument when either `root` or `tip` cannot be found
   * in the URDF
   */
  ModelKDL(const urdf::Model &model, const std::string &root,
           const std::string &tip);

  /**
   * Calculates the gravity vector. Unit: \f$[Nm]\f$.
   *
   * @param[in] q Joint position.
   * @param[in] m_total Weight of the attached total load including end
   * effector. Unit: \f$[kg]\f$.
   * @param[in] F_x_Ctotal Translation from flange to center of mass of the
   * attached total load. Unit: \f$[m]\f$.
   * @param[in] gravity_earth Earth's gravity vector. Unit: \f$\frac{m}{s^2}\f$.
   *
   * @return Gravity vector.
   */
  std::array<double, 7>
  gravity(const std::array<double, 7> &q,
          const std::array<double, 3> &gravity_earth) const;

private:
  static int segment(franka::Frame frame);
  static std::string strError(const int error);

  KDL::Chain chain_;
  double singularity_threshold_;
};
