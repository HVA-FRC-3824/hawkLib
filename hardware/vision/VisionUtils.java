// Copyright (c) 2026-2027 FRC 3824 HVA RoHawktics
// http://github.com/HVA-FRC-3824
//
// Use of this source code is governed by an MIT-style license that can be found in the LICENSE file at
// the root directory of this project.

package frc.shared.hardware.vision;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N4;
import edu.wpi.first.units.measure.Distance;
import frc.o2026.Configs;
import frc.o2026.Constants;
import java.util.Arrays;
import java.util.Optional;

public class VisionUtils {
  public static Pose3d getTagPose(int fiduciary) {

    return Constants.Vision.TagLayout.getTagPose(fiduciary).orElse(new Pose3d());
  }

  public static Optional<Matrix<N4, N1>> getEstimationStdDevs(Pose2d estimatedPose, int[] targets) {

    var estStdDevs = Configs.Vision.kSingleTagStdDevs;

    // No tags visible. Default to single-tag std devs
    if (targets.length == 0 || estimatedPose == null) {
      return Optional.empty();
    }

    // Precalculation - see how many tags we found, and calculate an average-distance metric

    Distance avgDist =
        Meters.of(
            Arrays.stream(targets)
                    .filter(
                        tgt -> 1 <= tgt && tgt <= Constants.Vision.TagLayout.getTags().size() - 1)
                    .mapToDouble(
                        (tgt) ->
                            getTagPose(tgt)
                                .toPose2d()
                                .getTranslation()
                                .getDistance(estimatedPose.getTranslation()))
                    .sum()
                / targets.length);

    
    if (targets.length == 1 && avgDist.gt(Meters.of((Constants.Field.FieldLengthMeters * 3) / 5))) {
      return Optional.empty();
    } else {
      // Increase std devs based on (average) distance
      // max distance 15 meters
      return Optional.of(estStdDevs.times(1 + (Math.pow(avgDist.in(Meters), 2) / 30)));
    }
  }
}
