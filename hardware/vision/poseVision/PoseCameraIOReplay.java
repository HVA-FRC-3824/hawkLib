// Copyright (c) 2026-2027 FRC 3824 HVA RoHawktics
// http://github.com/HVA-FRC-3824
//
// Use of this source code is governed by an MIT-style license that can be found in the LICENSE file at
// the root directory of this project.

package frc.shared.hardware.vision.poseVision;

public record PoseCameraIOReplay(String name) implements PoseCameraIO {

  public void updateInputs(PoseCameraInputs inputs) {

    inputs.name = name;
  }
}
