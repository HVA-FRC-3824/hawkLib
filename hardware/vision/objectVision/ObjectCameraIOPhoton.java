// Copyright (c) 2026-2027 FRC 3824 HVA RoHawktics
// http://github.com/HVA-FRC-3824
//
// Use of this source code is governed by an MIT-style license that can be found in the LICENSE file at
// the root directory of this project. Some code may be governed by other licenses which can be found in the "/External Licenses" directory.

package frc.shared.hardware.vision.objectVision;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Distance;
import frc.shared.hardware.vision.VisionConfig;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;

public class ObjectCameraIOPhoton implements ObjectCameraIO {

  private final PhotonCamera m_camera;

  private final VisionConfig m_config;

  private final Distance m_targetHeight;

  public ObjectCameraIOPhoton(VisionConfig config, Distance targetHeight) {

    m_config = config;

    m_targetHeight = targetHeight;

    m_camera = new PhotonCamera(m_config.name());
  }

  @Override
  public void updateInputs(ObjectCameraInputs inputs) {

    var results = m_camera.getAllUnreadResults();

    inputs.name = m_config.name();
    inputs.objects =
        results.stream()
            .flatMap(result -> result.getTargets().stream())
            .map(
                target -> {
                  Distance distance =
                      Meters.of(
                          PhotonUtils.calculateDistanceToTargetMeters(
                              m_config.offset().getTranslation().getZ(),
                              m_targetHeight.in(Meters) / 2.0,
                              -m_config.offset().getRotation().getMeasureY().in(Radians),
                              Degrees.of(target.getPitch()).in(Radians)));

                  Rotation2d yaw = Rotation2d.fromDegrees(target.getYaw());

                  // Camera-relative (X forward, Y left). This is a pure flat-ground approximation —
                  Translation3d cameraToTarget =
                      new Translation3d(
                          distance.times(yaw.getCos()),
                          distance.times(yaw.getSin()),
                          m_targetHeight
                              .div(2.0)
                              .minus(m_config.offset().getTranslation().getMeasureZ()));

                  Translation3d robotToTarget =
                      cameraToTarget
                          .rotateBy(m_config.offset().getRotation())
                          .plus(m_config.offset().getTranslation());

                  return new ObjectTargetData(
                      target.objDetectId, target.objDetectConf, robotToTarget);
                })
            .toArray(ObjectTargetData[]::new);

    if (results.isEmpty()) {
      inputs.hasRotToBestObject = false;
      return;
    }

    PhotonPipelineResult rot = results.get(0);
    if (rot == null) {
      inputs.hasRotToBestObject = false;
      return;
    }

    if (!rot.hasTargets()) {
      inputs.hasRotToBestObject = false;
      return;
    }

    inputs.rotToBestObject = Degrees.of(rot.getBestTarget().getYaw());
    inputs.hasRotToBestObject = true;
  }
}
