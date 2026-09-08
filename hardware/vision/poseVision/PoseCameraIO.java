// Copyright (c) 2026-2027 FRC 3824 HVA RoHawktics
// http://github.com/HVA-FRC-3824
//
// Use of this source code is governed by an MIT-style license that can be found in the LICENSE file at
// the root directory of this project. Some code may be governed by other licenses which can be found in the "/External Licenses" directory.

package frc.shared.hardware.vision.poseVision;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import frc.shared.Quadruple;
import org.littletonrobotics.junction.AutoLog;

public interface PoseCameraIO {

  @AutoLog
  public static class PoseCameraInputs {

    public Transform3d offset = new Transform3d();
    public String name = "";
    public VisionData[] measurements = new VisionData[0];
    public Pose2d[] lastSeenTags = new Pose2d[0];
  }

  public static record VisionData(
      Pose3d visionMeasurement,
      double timestampSeconds,
      Quadruple<Double, Double, Double, Double> stdDevs,
      int[] target) {

    public Matrix<N3, N1> get2dStdDevs() {

      return VecBuilder.fill(stdDevs.getFirst(), stdDevs.getSecond(), stdDevs.getFourth());
    }
  }

  public default void updateInputs(PoseCameraInputs inputs) {}
}
