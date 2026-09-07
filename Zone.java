// Copyright (c) 2026-2027 FRC 3824 HVA RoHawktics
// http://github.com/HVA-FRC-3824
//
// Use of this source code is governed by an MIT-style license that can be found in the LICENSE file at
// the root directory of this project. Some code may be governed by other licenses which can be found in the "/External Licenses" directory.

package frc.shared;

import static edu.wpi.first.units.Units.Meters;

import com.pathplanner.lib.util.FlippingUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.o2026.Constants;
import java.util.function.Supplier;

public class Zone {

  // From the blue side always
  public static Trigger fromCorners(
      Supplier<Pose2d> poseSupplier,
      Translation2d upperRightCorner,
      Translation2d lowerLeftCorner) {

    return new Trigger(
        () -> {
          var pose = FlippingUtil.flipFieldPose(poseSupplier.get());

          return upperRightCorner.getMeasureX().lt(pose.getMeasureX())
              && lowerLeftCorner.getMeasureX().gt(pose.getMeasureX())
              && upperRightCorner.getMeasureY().lt(pose.getMeasureY())
              && lowerLeftCorner.getMeasureY().gt(pose.getMeasureY());
        });
  }

  public static Trigger fromAllianceWall(Supplier<Pose2d> poseSupplier, Distance dist) {

    return fromCorners(
        poseSupplier,
        new Translation2d(Meters.of(Constants.Field.FieldWidthMeters), dist),
        new Translation2d(Meters.of(0.0), Meters.of(0.0)));
  }
}
