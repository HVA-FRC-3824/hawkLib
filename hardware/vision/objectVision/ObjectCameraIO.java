// Copyright (c) 2026-2027 FRC 3824 HVA RoHawktics
// http://github.com/HVA-FRC-3824
//
// Use of this source code is governed by an MIT-style license that can be found in the LICENSE file at
// the root directory of this project.

package frc.shared.hardware.vision.objectVision;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.units.measure.Angle;
import org.littletonrobotics.junction.AutoLog;

public interface ObjectCameraIO {

  @AutoLog
  public static class ObjectCameraInputs {

    public String name = "";
    public Angle rotToBestObject = Degrees.of(0);
    public boolean hasRotToBestObject = false;
    public ObjectTargetData[] objects = new ObjectTargetData[0];
  }

  public default void updateInputs(ObjectCameraInputs inputs) {}
}
