// Copyright (c) 2026-2027 FRC 3824 HVA RoHawktics
// http://github.com/HVA-FRC-3824
//
// Use of this source code is governed by an MIT-style license that can be found in the LICENSE file at
// the root directory of this project.

package frc.shared.hardware.vision.poseVision;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N4;
import edu.wpi.first.units.measure.Distance;
import frc.o2026.Configs;
import frc.o2026.Constants;
import frc.shared.hardware.vision.poseVision.PoseVision.VisionData;

import static edu.wpi.first.units.Units.Meters;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Set;
import java.util.function.Consumer;
import java.util.stream.DoubleStream;

import org.photonvision.simulation.PhotonCameraSim;

public interface PoseCameraIO {

  public ArrayList<VisionData> getMeasurements();

  public default PhotonCameraSim getSimCamera() {
    return null;
  }

  public Transform3d getOffset();

  public List<Pose2d> getLastSeenTags();

  public default void addGyroResetter(Consumer<Rotation3d> gyroResetter) {}

  public default void periodic() {}

  public static Pose3d getTagPose(int fiduciary) {

    return Constants.Vision.TagLayout.getTagPose(fiduciary).orElse(new Pose3d());
  }

  public static Matrix<N4, N1> getEstimationStdDevs(Pose2d estimatedPose, int[] targets) {

    // Pose present. Start running Heuristic
    var estStdDevs = Configs.Vision.kSingleTagStdDevs;

    if (targets.length == 0) {
      // No tags visible. Default to single-tag std devs
      return Configs.Vision.kSingleTagStdDevs;
    }

    // Precalculation - see how many tags we found, and calculate an average-distance metric

    Distance avgDist = Meters.of(
      Arrays.stream(targets)
        .filter(tgt -> 1 <= tgt && tgt <= Constants.Vision.TagLayout.getTags().size() - 1)
        .mapToDouble((tgt) -> 
          PoseCameraIO.getTagPose(tgt).toPose2d().getTranslation().getDistance(estimatedPose.getTranslation())
        ).sum()
        / targets.length);
 
    // Increase std devs based on (average) distance
    // max distance 15 meters
    if (targets.length == 1 && avgDist.gt(Meters.of(15))) {
      return VecBuilder.fill(
          Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
    } else {
      return estStdDevs.times(1 + (Math.pow(avgDist.in(Meters), 2) / 30));
    }
  }
}
