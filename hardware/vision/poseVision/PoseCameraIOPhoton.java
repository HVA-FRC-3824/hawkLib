// Copyright (c) 2026-2027 FRC 3824 HVA RoHawktics
// http://github.com/HVA-FRC-3824
//
// Use of this source code is governed by an MIT-style license that can be found in the LICENSE file at
// the root directory of this project. Some code may be governed by other licenses which can be found in the "/External Licenses" directory.

package frc.shared.hardware.vision.poseVision;

import static edu.wpi.first.units.Units.DegreesPerSecond;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N4;
import edu.wpi.first.wpilibj.Timer;
import frc.o2026.Configs;
import frc.o2026.Constants;
import frc.o2026.RobotState;
import frc.shared.hardware.vision.VisionConfig;
import frc.shared.hardware.vision.VisionUtils;
import frc.shared.hardware.vision.poseVision.PoseVision.VisionData;
import java.util.ArrayList;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class PoseCameraIOPhoton implements PoseCameraIO {

  protected PhotonCamera m_camera;
  private PhotonPoseEstimator estimator;

  private final VisionConfig m_config;

  private Pose2d[] m_lastSeenTags = new Pose2d[0];

  public PoseCameraIOPhoton(VisionConfig config) {

    m_config = config;

    m_camera = new PhotonCamera(m_config.name());
    estimator = new PhotonPoseEstimator(Constants.Vision.TagLayout, m_config.offset());
  }

  private ArrayList<VisionData> getMeasurements() {

    return new ArrayList<VisionData>(
        m_camera.getAllUnreadResults().stream()
            .filter(PhotonPipelineResult::hasTargets)
            .filter(result -> result.getBestTarget().poseAmbiguity < 0.3)
            .map(
                (result) -> {

                  // Use multiple tags to create a very accurate pose estimate
                  var est = estimator.estimateCoprocMultiTagPose(result);
                  if (est.isPresent()) {
                    return est;
                  }

                  // Use gyro data in combination with tag data to get an estimate as
                  // accurate as your gyro
                  estimator.addHeadingData(
                      Timer.getTimestamp(), RobotState.getPoseEst().getRotation().toRotation2d());
                  // I've found that this data can be inaccurate when rotating at high velocity
                  if (RobotState.getAngularVelocity().lte(DegreesPerSecond.of(5.0))) {
                    est = estimator.estimatePnpDistanceTrigSolvePose(result);
                    if (est.isPresent()) return est;
                  }

                  // Take multiple estimations from multiple tags and average their results
                  if (result.getTargets().size() > 1) {
                    est = estimator.estimateAverageBestTargetsPose(result);
                    if (est.isPresent()) return est;
                  }

                  // No complicated sensor fusion between multiple tags or gyro
                  // simply the best guess given a single tag
                  est = estimator.estimateLowestAmbiguityPose(result);
                  return est;
                })
            .filter(Optional::isPresent)
            .map(Optional::get)
            .map(
                est -> {
                  var targets = est.targetsUsed.stream().mapToInt((target) -> target.fiducialId);

                  Logger.recordOutput(
                      "Vision/" + m_camera.getName() + "/lastMeasurement", est.timestampSeconds);

                  Logger.recordOutput("Vision/" + m_camera.getName() + "/est", est.estimatedPose);

                  var targetArray = targets.toArray();

                  // calculate the trust based on the distance of the tag(s) used
                  var stdDevs =
                      VisionUtils.getEstimationStdDevs(est.estimatedPose.toPose2d(), targetArray)
                      .orElse(VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE));

                  if (est.strategy != PoseStrategy.PNP_DISTANCE_TRIG_SOLVE) {

                    stdDevs.set(3, 0, Double.MAX_VALUE);
                  }

                  m_lastSeenTags =
                      est.targetsUsed.stream()
                          .map(PhotonTrackedTarget::getFiducialId)
                          .map(VisionUtils::getTagPose)
                          .map(Pose3d::toPose2d)
                          .toArray(Pose2d[]::new);

                  return new VisionData(
                      est.estimatedPose, est.timestampSeconds, stdDevs, targetArray);
                })
            .toList());
  }

  @Override
  public void updateInputs(PoseCameraInputs inputs) {

    inputs.offset = m_config.offset();
    inputs.name = m_config.name();
    inputs.lastSeenTags = m_lastSeenTags;
    inputs.measurements = getMeasurements().toArray(VisionData[]::new);
  }
}
