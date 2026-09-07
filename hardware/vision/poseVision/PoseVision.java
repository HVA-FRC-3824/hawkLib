// Copyright (c) 2026-2027 FRC 3824 HVA RoHawktics
// http://github.com/HVA-FRC-3824
//
// Use of this source code is governed by an MIT-style license that can be found in the LICENSE file at
// the root directory of this project. Some code may be governed by other licenses which can be found in the "/External Licenses" directory.

package frc.shared.hardware.vision.poseVision;

import edu.wpi.first.math.Pair;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.shared.hardware.vision.poseVision.PoseCameraIO.VisionData;
import java.util.Arrays;
import java.util.List;
import java.util.function.Consumer;
import org.littletonrobotics.junction.Logger;

public class PoseVision extends SubsystemBase {

  private Consumer<VisionData> m_poseEstimatorConsumer;

  public List<Pair<PoseCameraIO, PoseCameraInputsAutoLogged>> m_cameras;

  public PoseVision(Consumer<VisionData> poseEstimatorConsumer, PoseCameraIO... cameras) {

    m_poseEstimatorConsumer = poseEstimatorConsumer;

    m_cameras =
        Arrays.asList(cameras).stream()
            .map(
                camera ->
                    new Pair<PoseCameraIO, PoseCameraInputsAutoLogged>(
                        camera, new PoseCameraInputsAutoLogged()))
            .toList();
  }

  @Override
  public void periodic() {

    m_cameras.stream()
        .flatMap(
            camera -> {
              camera.getFirst().updateInputs(camera.getSecond());
              Logger.processInputs(camera.getSecond().name, camera.getSecond());

              return Arrays.asList(camera.getSecond().measurements).stream();
            })
        .forEach(data -> m_poseEstimatorConsumer.accept(data));
  }
}
