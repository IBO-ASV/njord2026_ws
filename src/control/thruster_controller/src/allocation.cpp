#include "thruster_controller/allocation.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace njord
{
namespace thruster_controller
{
namespace
{

std::vector<double> solveRegularizedLeastSquares(
  const std::vector<std::vector<double>> & a,
  const std::vector<double> & b,
  double lambda)
{
  const std::size_t rows = a.size();
  const std::size_t cols = a.front().size();
  std::vector<std::vector<double>> ata(cols, std::vector<double>(cols, 0.0));
  std::vector<double> atb(cols, 0.0);

  for (std::size_t i = 0; i < cols; ++i) {
    for (std::size_t j = 0; j < cols; ++j) {
      for (std::size_t k = 0; k < rows; ++k) {
        ata[i][j] += a[k][i] * a[k][j];
      }
    }
    ata[i][i] += lambda;
    for (std::size_t k = 0; k < rows; ++k) {
      atb[i] += a[k][i] * b[k];
    }
  }

  for (std::size_t i = 0; i < cols; ++i) {
    std::size_t pivot = i;
    for (std::size_t r = i + 1U; r < cols; ++r) {
      if (std::abs(ata[r][i]) > std::abs(ata[pivot][i])) {
        pivot = r;
      }
    }
    if (pivot != i) {
      std::swap(ata[i], ata[pivot]);
      std::swap(atb[i], atb[pivot]);
    }

    const double diag = ata[i][i];
    if (std::abs(diag) < 1e-12) {
      continue;
    }
    for (std::size_t r = i + 1U; r < cols; ++r) {
      const double factor = ata[r][i] / diag;
      for (std::size_t c = i; c < cols; ++c) {
        ata[r][c] -= factor * ata[i][c];
      }
      atb[r] -= factor * atb[i];
    }
  }

  std::vector<double> x(cols, 0.0);
  for (int i = static_cast<int>(cols) - 1; i >= 0; --i) {
    double rhs = atb[static_cast<std::size_t>(i)];
    for (std::size_t c = static_cast<std::size_t>(i) + 1U; c < cols; ++c) {
      rhs -= ata[static_cast<std::size_t>(i)][c] * x[c];
    }
    const double diag = ata[static_cast<std::size_t>(i)][static_cast<std::size_t>(i)];
    if (std::abs(diag) >= 1e-12) {
      x[static_cast<std::size_t>(i)] = rhs / diag;
    }
  }
  return x;
}

std::vector<std::vector<double>> allocationMatrix(
  const std::vector<ThrusterGeometry> & thrusters)
{
  std::vector<std::vector<double>> matrix(3, std::vector<double>(thrusters.size(), 0.0));
  for (std::size_t i = 0; i < thrusters.size(); ++i) {
    const double dx = std::cos(thrusters[i].angle_rad);
    const double dy = std::sin(thrusters[i].angle_rad);
    const double gain = thrusters[i].force_per_duty;
    matrix[0][i] = gain * dx;
    matrix[1][i] = gain * dy;
    matrix[2][i] = gain * (thrusters[i].x * dy - thrusters[i].y * dx);
  }
  return matrix;
}

}  // namespace

std::vector<double> allocateWrench(
  const std::vector<ThrusterGeometry> & thrusters,
  const std::vector<double> & wrench,
  double regularization_lambda)
{
  if (thrusters.size() < 3U || wrench.size() != 3U) {
    throw std::invalid_argument("allocation requires at least three thrusters and a 3D wrench");
  }

  std::vector<double> commands = solveRegularizedLeastSquares(
    allocationMatrix(thrusters), wrench, std::max(1e-9, regularization_lambda));
  double max_abs = 0.0;
  for (double command : commands) {
    max_abs = std::max(max_abs, std::abs(command));
  }
  if (max_abs > 1.0) {
    for (double & command : commands) {
      command /= max_abs;
    }
  }
  for (std::size_t i = 0; i < commands.size(); ++i) {
    if (thrusters[i].reverse) {
      commands[i] *= -1.0;
    }
    commands[i] = std::clamp(commands[i], -1.0, 1.0);
  }
  return commands;
}

std::vector<double> commandToWrench(
  const std::vector<ThrusterGeometry> & thrusters,
  const std::vector<double> & commands)
{
  if (thrusters.size() != commands.size()) {
    throw std::invalid_argument("thruster and command counts must match");
  }
  const auto matrix = allocationMatrix(thrusters);
  std::vector<double> wrench(3, 0.0);
  for (std::size_t row = 0; row < wrench.size(); ++row) {
    for (std::size_t i = 0; i < commands.size(); ++i) {
      const double physical_command = thrusters[i].reverse ? -commands[i] : commands[i];
      wrench[row] += matrix[row][i] * physical_command;
    }
  }
  return wrench;
}

}  // namespace thruster_controller
}  // namespace njord
