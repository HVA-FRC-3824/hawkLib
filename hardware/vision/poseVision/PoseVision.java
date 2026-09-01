// Copyright (c) 2026-2027 FRC 3824 HVA RoHawktics
// http://github.com/HVA-FRC-3824
//
// Use of this source code is governed by an MIT-style license that can be found in the LICENSE file at
// the root directory of this project.

package frc.shared.hardware.vision.poseVision;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.numbers.N4;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.shared.hardware.vision.poseVision.PoseCameraIO.PoseCameraInputs;
import java.util.Arrays;
import java.util.List;
import java.util.function.Consumer;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class PoseVision extends SubsystemBase {

  private Consumer<VisionData> m_poseEstimatorConsumer;

  public List<Pair<PoseCameraIO, PoseCameraInputs>> m_cameras;

  public PoseVision(Consumer<VisionData> poseEstimatorConsumer, PoseCameraIO... cameras) {

    m_poseEstimatorConsumer = poseEstimatorConsumer;

    // Totally not the most efficient way to do this
    m_cameras =
        Arrays.asList(cameras).stream()
            .map(camera -> new Pair<PoseCameraIO, PoseCameraInputs>(camera, new PoseCameraInputs()))
            .toList();
  }

  @Override
  public void periodic() {

    for (Pair<PoseCameraIO, PoseCameraInputs> camera : m_cameras) {

      camera.getFirst().updateInputs(camera.getSecond());
      Logger.processInputs(camera.getSecond().name, (LoggableInputs) camera.getSecond());

      for (VisionData data : camera.getSecond().measurements) {
        m_poseEstimatorConsumer.accept(data);
      }
    }
  }

  public record VisionData(
      Pose3d visionMeasurement, double timestampSeconds, Matrix<N4, N1> stdDevs, int[] target) {

    public Matrix<N3, N1> get2dStdDevs() {

      return VecBuilder.fill(stdDevs().get(0, 0), stdDevs().get(1, 0), stdDevs().get(3, 0));
    }
  }
}
