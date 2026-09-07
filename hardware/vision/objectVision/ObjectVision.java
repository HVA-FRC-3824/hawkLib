// Copyright (c) 2026-2027 FRC 3824 HVA RoHawktics
// http://github.com/HVA-FRC-3824
//
// Use of this source code is governed by an MIT-style license that can be found in the LICENSE file at
// the root directory of this project. Some code may be governed by other licenses which can be found in the "/External Licenses" directory.

package frc.shared.hardware.vision.objectVision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.o2026.RobotState;
import java.util.Comparator;
import java.util.List;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;

public class ObjectVision extends SubsystemBase {

  private ObjectCameraIO m_io;
  private ObjectCameraInputsAutoLogged m_inputs = new ObjectCameraInputsAutoLogged();

  private List<Translation2d> m_fieldRelativeODTargets = List.of();

  public ObjectVision(ObjectCameraIO io) {

    m_io = io;
  }

  @Override
  public void periodic() {

    m_io.updateInputs(m_inputs);
    Logger.processInputs(m_inputs.name, m_inputs);

    m_fieldRelativeODTargets =
        List.of(m_inputs.objects).stream()
            .map(
                object ->
                    object
                        .translation()
                        .toTranslation2d()
                        .rotateBy(RobotState.getPoseEst().toPose2d().getRotation())
                        .plus(RobotState.getPoseEst().toPose2d().getTranslation()))
            .toList();

    Pose2d[] poseArray =
        m_fieldRelativeODTargets.stream()
            .map(t -> new Pose2d(t, new Rotation2d()))
            .toArray(Pose2d[]::new);
    Logger.recordOutput("odObject", poseArray);
  }

  public Optional<Rotation2d> directionToObject() {

    return getClosestObject()
        .map(
            translation -> {
              Rotation2d angle =
                  translation.minus(RobotState.getPoseEst().toPose2d().getTranslation()).getAngle();
              Logger.recordOutput("objectYaw", angle.getDegrees());
              return angle;
            });
  }

  public Optional<Translation2d> getClosestObject() {

    var poses = m_fieldRelativeODTargets;

    if (poses.isEmpty()) return Optional.empty();

    return poses.stream()
        .min(
            Comparator.comparingDouble(
                object -> object.getDistance(RobotState.getPoseEst().toPose2d().getTranslation())));
  }

  public boolean hasObjects() {

    return m_inputs.objects.length > 0;
  }
}
