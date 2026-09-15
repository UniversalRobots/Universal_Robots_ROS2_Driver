// Copyright 2026 Universal Robots A/S
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the {copyright_holder} nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <set>
#include <string>

#include "ur_robot_driver/mode_compatibility.hpp"

using ModeCompatibilityMatrix = ur_robot_driver::ModeCompatibilityMatrix;

bool lookupModeCompatibility(const ModeCompatibilityMatrix& matrix, const std::string& mode,
                             const std::string& other_mode)
{
  const auto mode_it = matrix.find(mode);
  if (mode_it == matrix.end()) {
    return false;
  }

  const auto other_it = mode_it->second.find(other_mode);
  if (other_it == mode_it->second.end()) {
    return false;
  }

  return other_it->second;
}

std::set<std::string> collectControlModes(const ModeCompatibilityMatrix& matrix)
{
  std::set<std::string> modes;
  for (const auto& [mode, compatibilities] : matrix) {
    modes.insert(mode);
    for (const auto& [other_mode, compatible] : compatibilities) {
      (void)compatible;
      modes.insert(other_mode);
    }
  }
  return modes;
}

TEST(ModeCompatibilitySymmetryTest, TablesAreSymmetric)
{
  const ModeCompatibilityMatrix matrix = ur_robot_driver::createModeCompatibilityMatrix();
  const std::set<std::string> modes = collectControlModes(matrix);

  for (const auto& mode_a : modes) {
    for (const auto& mode_b : modes) {
      if (mode_a == mode_b) {
        continue;
      }

      const bool a_compatible_with_b = lookupModeCompatibility(matrix, mode_a, mode_b);
      const bool b_compatible_with_a = lookupModeCompatibility(matrix, mode_b, mode_a);

      EXPECT_EQ(a_compatible_with_b, b_compatible_with_a)
          << "Mode compatibility mismatch between '" << mode_a << "' and '" << mode_b << "': " << mode_a << " -> "
          << mode_b << " is " << (a_compatible_with_b ? "compatible" : "incompatible") << ", but " << mode_b << " -> "
          << mode_a << " is " << (b_compatible_with_a ? "compatible" : "incompatible");
    }
  }
}

int main(int argc, char* argv[])
{
  ::testing::InitGoogleMock(&argc, argv);
  return RUN_ALL_TESTS();
}
