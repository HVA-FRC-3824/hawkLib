// Copyright (c) 2026-2027 FRC 3824 HVA RoHawktics
// http://github.com/HVA-FRC-3824
//
// Use of this source code is governed by an MIT-style license that can be found in the LICENSE file at
// the root directory of this project. Some code may be governed by other licenses which can be found in the "/External Licenses" directory.

package frc.shared.hardware.vision.poseVision;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import frc.o2026.RobotState;
import frc.shared.Quadruple;
import frc.shared.hardware.vision.VisionConfig;
import frc.shared.hardware.vision.VisionUtils;
import frc.shared.hardware.vision.limelight.LimelightHelpers;
import frc.shared.hardware.vision.limelight.LimelightHelpers.PoseEstimate;
import java.util.ArrayList;
import java.util.List;
import org.littletonrobotics.junction.Logger;

public class PoseCameraIOLimelight implements PoseCameraIO {

  private final VisionConfig m_config;

  private Pose2d[] m_lastSeenTags = new Pose2d[0];

  public PoseCameraIOLimelight(VisionConfig config) {

    m_config = config;

    // Change the camera pose relative to robot center (x forward, y left, z up, degrees)
    LimelightHelpers.setCameraPose_RobotSpace(
        m_config.name(),
        m_config.offset().getX(), // Forward offset (meters)
        m_config.offset().getY(), // Side offset (meters)
        m_config.offset().getZ(), // Height offset (meters)
        m_config.offset().getRotation().getMeasureX().in(Degrees), // Roll (degrees)
        m_config.offset().getRotation().getMeasureY().in(Degrees), // Pitch (degrees)
        m_config.offset().getRotation().getMeasureZ().in(Degrees) // Yaw (degrees)
        );
  }

  private ArrayList<VisionData> getMeasurements() {

    var rot = RobotState.getPoseEst().getRotation();
    LimelightHelpers.SetRobotOrientation(
        m_config.name(),
        rot.getMeasureZ().in(Degrees),
        RobotState.getAngularVelocity().in(DegreesPerSecond),
        rot.getMeasureY().in(Degrees), // pitch
        0,
        rot.getMeasureX().in(Degrees), // roll
        0);

    PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(m_config.name());

    // Filter out frivolous readings
    if (mt2 == null) return new ArrayList<>();
    if (mt2.pose.equals(new Pose2d())) return new ArrayList<>();

    var tags = List.of(mt2.rawFiducials).stream().mapToInt((tag) -> tag.id).boxed().toList();

    m_lastSeenTags =
        tags.stream().map(VisionUtils::getTagPose).map(Pose3d::toPose2d).toArray(Pose2d[]::new);

    Logger.recordOutput("Vision/" + m_config.name() + "/lastMeasurement", mt2.timestampSeconds);

    Logger.recordOutput("Vision/" + m_config.name() + "/est", mt2.pose);

    var tagArr = tags.stream().mapToInt(x -> x).toArray();
    var measurements = new ArrayList<VisionData>(1);

    var stdDevs =
        VisionUtils.getEstimationStdDevs(mt2.pose, tagArr)
            .orElse(
                VecBuilder.fill(
                    Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE));

    measurements.add(
        new VisionData(
            new Pose3d(mt2.pose),
            mt2.timestampSeconds,
            new Quadruple<>(
                stdDevs.get(0, 0), stdDevs.get(1, 0), stdDevs.get(2, 0), stdDevs.get(3, 0)),
            tagArr));

    return measurements;
  }

  @Override
  public void updateInputs(PoseCameraInputs inputs) {

    inputs.offset = m_config.offset();
    inputs.name = m_config.name();
    inputs.lastSeenTags = m_lastSeenTags;
    inputs.measurements = getMeasurements().toArray(VisionData[]::new);
  }
}
