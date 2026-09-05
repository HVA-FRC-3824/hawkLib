// Copyright (c) 2026-2027 FRC 3824 HVA RoHawktics
// http://github.com/HVA-FRC-3824
//
// Use of this source code is governed by an MIT-style license that can be found in the LICENSE file at
// the root directory of this project. Some code may be governed by other licenses which can be found in the "/External Licenses" directory.

package frc.shared.hardware.vision.poseVision;

public record PoseCameraIOReplay(String name) implements PoseCameraIO {

  // There are other ways of doing this. See how FRC 5000 Hammerheads did it.
  // This is imo the simplest method in terms of usage, but is not 100% idomatic.
  public void updateInputs(PoseCameraInputs inputs) {

    inputs.name = name;
  }
}
