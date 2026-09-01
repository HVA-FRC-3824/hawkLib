// Copyright (c) 2026-2027 FRC 3824 HVA RoHawktics
// http://github.com/HVA-FRC-3824
//
// Use of this source code is governed by an MIT-style license that can be found in the LICENSE file at
// the root directory of this project.

package frc.shared.hardware.vision.objectVision;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Angle;
import frc.o2026.RobotState;
import frc.shared.hardware.vision.VisionConfig;
import java.util.Comparator;
import java.util.List;
import java.util.Optional;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.gamepieces.GamePieceOnFieldSimulation;

public class ObjectCameraIOSim implements ObjectCameraIO {

  private static final Rotation2d CAMERA_HORIZONTAL_FOV = Rotation2d.fromDegrees(75),
      CAMERA_VERTICAL_FOV = Rotation2d.fromDegrees(45);

  private final VisionConfig m_config;

  private final SimulatedArena m_arena;

  public ObjectCameraIOSim(VisionConfig config, SimulatedArena arena) {

    m_config = config;

    m_arena = arena;
  }

  @Override
  public void updateInputs(ObjectCameraInputs inputs) {

    inputs.name = m_config.name();

    Pose3d robotPose = RobotState.getSimRealPose();
    inputs.objects =
        m_arena.gamePiecesOnField().stream()
            .map(GamePieceOnFieldSimulation::getPose3d)
            .map(Pose3d::getTranslation)
            .map(
                translation -> {
                  // translation is field-relative. Convert to robot-relative:
                  Translation2d robotRelative2d =
                      translation
                          .toTranslation2d()
                          .minus(robotPose.getTranslation().toTranslation2d())
                          .rotateBy(robotPose.getRotation().toRotation2d().unaryMinus());
                  double relativeZ = translation.getZ() - robotPose.getZ();
                  return new Translation3d(
                      robotRelative2d.getX(), robotRelative2d.getY(), relativeZ);
                })
            .filter(
                robotToTarget -> {
                  // Convert robot-relative target position to camera-relative:
                  Translation3d cameraToTarget =
                      robotToTarget
                          .minus(m_config.offset().getTranslation())
                          .rotateBy(m_config.offset().getRotation().unaryMinus());

                  // Target is in front of camera (X > 0)
                  if (cameraToTarget.getX() <= 0) return false;

                  // Check horizontal FOV
                  double horizontalAngle = Math.atan2(cameraToTarget.getY(), cameraToTarget.getX());
                  if (Math.abs(horizontalAngle) > CAMERA_HORIZONTAL_FOV.getRadians() / 2.0)
                    return false;

                  // Check vertical FOV
                  double verticalAngle = Math.atan2(cameraToTarget.getZ(), cameraToTarget.getX());
                  if (Math.abs(verticalAngle) > CAMERA_VERTICAL_FOV.getRadians() / 2.0)
                    return false;

                  return true;
                })
            .map(robotToTarget -> new ObjectTargetData(0, 1.0, robotToTarget))
            .toArray(ObjectTargetData[]::new);

    var optRot = getRotToBestObject(inputs.objects);
    inputs.rotToBestObject = optRot.orElse(Degrees.of(0));
    inputs.hasRotToBestObject = optRot.isPresent();
  }

  public Optional<Angle> getRotToBestObject(ObjectTargetData[] objects) {

    Optional<ObjectTargetData> closestObject =
        List.of(objects).stream()
            .min(
                Comparator.comparingDouble(data -> data.translation().toTranslation2d().getNorm()));

    if (closestObject.isEmpty()) return Optional.empty();

    // Convert the robot-relative target position to camera-relative, and extract its yaw angle
    Translation3d robotToTarget = closestObject.get().translation();
    Translation3d cameraToTarget =
        robotToTarget
            .minus(m_config.offset().getTranslation())
            .rotateBy(m_config.offset().getRotation().unaryMinus());

    return Optional.of(cameraToTarget.toTranslation2d().getAngle().getMeasure());
  }
}
