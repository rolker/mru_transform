// Tests for REP-105 twist covariance rotation in mru_transform.
//
// The production rotation in MRUTransform::updateVelocity /
// updateOrientation goes:
//   Sigma_body = R * Sigma_world * R^T
// where R is the rotation extracted from a TF lookup
// (base_frame ← header.frame_id).  These tests exercise the covariance
// rotation helper for correctness on synthetic inputs with known expected
// outputs and for the structural invariants any orthogonal similarity
// transform must preserve (trace, symmetry).
//
// Vector rotation itself (v_body = R * v_world) is exercised directly by
// tf2::Matrix3x3::operator*(tf2::Vector3) — no mru_transform code of its
// own — so it's not repeated here.

#include <array>
#include <cmath>

#include <gtest/gtest.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

#include "mru_transform/twist_rotation_utils.hpp"

namespace
{

constexpr double kEpsilon = 1e-9;

tf2::Matrix3x3 yawRotation(double yaw_rad)
{
  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, yaw_rad);
  return tf2::Matrix3x3(q);
}

// Build a 6x6 covariance array with the 3x3 block at (offset, offset)
// populated from a flat 9-array row-major.  Other entries zero.
std::array<double, 36> makeBlockCovariance(
  const std::array<double, 9> &block, int offset)
{
  std::array<double, 36> cov{};
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      cov[(offset + i) * 6 + (offset + j)] = block[i * 3 + j];
    }
  }
  return cov;
}

std::array<double, 9> extractBlock(
  const std::array<double, 36> &cov, int offset)
{
  std::array<double, 9> block{};
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      block[i * 3 + j] = cov[(offset + i) * 6 + (offset + j)];
    }
  }
  return block;
}

}  // namespace

// -------------------------------------------------------------------------
// rotate_covariance_block_3x3 — helper from twist_rotation_utils.hpp
// -------------------------------------------------------------------------

TEST(CovarianceRotation, IdentityIsNoOp)
{
  const std::array<double, 9> block{
    0.04, 0.00, 0.00,
    0.00, 0.02, 0.00,
    0.00, 0.00, 0.01};
  auto cov_in = makeBlockCovariance(block, 0);
  std::array<double, 36> cov_out{};
  tf2::Matrix3x3 I; I.setIdentity();
  mru_transform::rotate_covariance_block_3x3(cov_in, I, 0, cov_out);
  auto out_block = extractBlock(cov_out, 0);
  for (size_t i = 0; i < 9; ++i) {
    EXPECT_NEAR(out_block[i], block[i], kEpsilon) << "entry " << i;
  }
}

TEST(CovarianceRotation, Yaw90SwapsDiagonalXY)
{
  // Diagonal covariance: sigma_xx=4, sigma_yy=1, sigma_zz=9 (units m^2/s^2)
  const std::array<double, 9> block{
    4.0, 0.0, 0.0,
    0.0, 1.0, 0.0,
    0.0, 0.0, 9.0};
  auto cov_in = makeBlockCovariance(block, 0);
  std::array<double, 36> cov_out{};
  auto R = yawRotation(M_PI / 2);
  mru_transform::rotate_covariance_block_3x3(cov_in, R, 0, cov_out);
  auto out_block = extractBlock(cov_out, 0);
  // 90-degree yaw rotates the covariance ellipse by 90deg: xx <-> yy.
  EXPECT_NEAR(out_block[0 * 3 + 0], 1.0, 1e-12);  // new xx was old yy
  EXPECT_NEAR(out_block[1 * 3 + 1], 4.0, 1e-12);  // new yy was old xx
  EXPECT_NEAR(out_block[2 * 3 + 2], 9.0, 1e-12);  // zz untouched
  // Off-diagonals must remain zero.
  EXPECT_NEAR(out_block[0 * 3 + 1], 0.0, 1e-12);
  EXPECT_NEAR(out_block[1 * 3 + 0], 0.0, 1e-12);
}

TEST(CovarianceRotation, PreservesTrace)
{
  // Trace is invariant under similarity transforms A^T Sigma A = A Sigma A^T
  // when A is orthogonal (which rotation matrices are).
  const std::array<double, 9> block{
    0.50, 0.15, 0.02,
    0.15, 0.40, 0.03,
    0.02, 0.03, 0.30};
  auto cov_in = makeBlockCovariance(block, 0);
  std::array<double, 36> cov_out{};
  auto R = yawRotation(0.317);
  mru_transform::rotate_covariance_block_3x3(cov_in, R, 0, cov_out);
  double trace_in = block[0] + block[4] + block[8];
  auto out_block = extractBlock(cov_out, 0);
  double trace_out = out_block[0] + out_block[4] + out_block[8];
  EXPECT_NEAR(trace_in, trace_out, 1e-12);
}

TEST(CovarianceRotation, PreservesSymmetry)
{
  // A symmetric input rotated by an orthogonal matrix must remain symmetric.
  const std::array<double, 9> block{
    0.25, 0.10, -0.05,
    0.10, 0.60,  0.07,
    -0.05, 0.07, 0.15};
  auto cov_in = makeBlockCovariance(block, 0);
  std::array<double, 36> cov_out{};
  tf2::Quaternion q;
  q.setRPY(0.2, -0.4, 0.6);
  tf2::Matrix3x3 R(q);
  mru_transform::rotate_covariance_block_3x3(cov_in, R, 0, cov_out);
  auto out_block = extractBlock(cov_out, 0);
  EXPECT_NEAR(out_block[0 * 3 + 1], out_block[1 * 3 + 0], 1e-12);
  EXPECT_NEAR(out_block[0 * 3 + 2], out_block[2 * 3 + 0], 1e-12);
  EXPECT_NEAR(out_block[1 * 3 + 2], out_block[2 * 3 + 1], 1e-12);
}

TEST(CovarianceRotation, AngularBlockOffset3)
{
  // Same math, but the block lives at offset 3 (lower-right 3x3 of the 6x6
  // twist.covariance) — verifies the offset logic is correct.
  const std::array<double, 9> block{
    4.0, 0.0, 0.0,
    0.0, 1.0, 0.0,
    0.0, 0.0, 9.0};
  auto cov_in = makeBlockCovariance(block, 3);
  std::array<double, 36> cov_out{};
  auto R = yawRotation(M_PI / 2);
  mru_transform::rotate_covariance_block_3x3(cov_in, R, 3, cov_out);
  // Linear block (0..2, 0..2) must be untouched.
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      EXPECT_NEAR(cov_out[i * 6 + j], 0.0, kEpsilon)
        << "linear block leaked at (" << i << "," << j << ")";
    }
  }
  // Angular block: xx<->yy swap as in the yaw-90 case.
  auto out_block = extractBlock(cov_out, 3);
  EXPECT_NEAR(out_block[0 * 3 + 0], 1.0, 1e-12);
  EXPECT_NEAR(out_block[1 * 3 + 1], 4.0, 1e-12);
  EXPECT_NEAR(out_block[2 * 3 + 2], 9.0, 1e-12);
}

TEST(CovarianceRotation, HandComputedRotationMatchesClosedForm)
{
  // Concrete scenario: 45-degree yaw on a purely forward-variance input
  //   Sigma_in = diag(1, 0, 0)
  //   R = Rz(pi/4) = [[c, -s, 0], [s, c, 0], [0, 0, 1]] with c=s=sqrt(2)/2
  //   Sigma_out = R Sigma_in R^T = 0.5 * [[1, 1, 0], [1, 1, 0], [0, 0, 0]]
  const std::array<double, 9> block{
    1.0, 0.0, 0.0,
    0.0, 0.0, 0.0,
    0.0, 0.0, 0.0};
  auto cov_in = makeBlockCovariance(block, 0);
  std::array<double, 36> cov_out{};
  auto R = yawRotation(M_PI / 4);
  mru_transform::rotate_covariance_block_3x3(cov_in, R, 0, cov_out);
  auto out_block = extractBlock(cov_out, 0);
  EXPECT_NEAR(out_block[0 * 3 + 0], 0.5, 1e-12);
  EXPECT_NEAR(out_block[0 * 3 + 1], 0.5, 1e-12);
  EXPECT_NEAR(out_block[1 * 3 + 0], 0.5, 1e-12);
  EXPECT_NEAR(out_block[1 * 3 + 1], 0.5, 1e-12);
  EXPECT_NEAR(out_block[2 * 3 + 2], 0.0, 1e-12);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
