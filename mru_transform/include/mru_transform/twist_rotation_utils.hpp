#ifndef MRU_TRANSFORM_TWIST_ROTATION_UTILS_HPP
#define MRU_TRANSFORM_TWIST_ROTATION_UTILS_HPP

#include <array>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Vector3.h>

namespace mru_transform
{

// Rotate a 3x3 covariance sub-block embedded at offset (block_offset,
// block_offset) within a 6x6 covariance matrix stored row-major as
// std::array<double, 36>.  Computes Sigma_out = R * Sigma_in * R^T on
// the indicated block, writing back into `covariance_out` at the same
// offset.  Other blocks of covariance_out are left untouched.
//
// block_offset = 0: upper-left 3x3 (linear velocity block)
// block_offset = 3: lower-right 3x3 (angular velocity block)
//
// Scope note: this helper handles diagonal blocks only.  The off-diagonal
// 3x3 blocks at (0, 3) and (3, 0) — cross-covariance between linear and
// angular velocity — are NOT rotated.  For current known sources
// (mavros gps_vel, posmv, asv_sim) those cross-blocks are always zero
// because linear and angular velocity come from independent sensors, so
// in practice nothing is lost.  If a future source provides non-zero
// cross-covariance (e.g., a fused INS reporting a full 6x6), this
// helper's output will silently zero those blocks.  See
// rolker/mru_transform#18 review discussion.
inline void rotate_covariance_block_3x3(
  const std::array<double, 36> &covariance_in,
  const tf2::Matrix3x3 &R,
  int block_offset,
  std::array<double, 36> &covariance_out)
{
  tf2::Matrix3x3 Sigma_in;
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      Sigma_in[i][j] =
        covariance_in[(block_offset + i) * 6 + (block_offset + j)];
    }
  }
  const tf2::Matrix3x3 Sigma_out = R * Sigma_in * R.transpose();
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      covariance_out[(block_offset + i) * 6 + (block_offset + j)] =
        Sigma_out[i][j];
    }
  }
}

} // namespace mru_transform

#endif
