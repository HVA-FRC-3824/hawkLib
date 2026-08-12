// Copyright (c) 2026-2027 FRC 3824 HVA RoHawktics
// http://github.com/HVA-FRC-3824
//
// Use of this source code is governed by an MIT-style license that can be found in the LICENSE file at
// the root directory of this project.

package frc.shared.reefscape;

import static edu.wpi.first.units.Units.Centimeters;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import frc.shared.Quadruple;
import frc.shared.hardware.vision.poseVision.PoseCameraIO;
import java.util.HashMap;
import java.util.List;
import java.util.Optional;
import java.util.stream.IntStream;
import java.util.stream.Stream;

public class ReefscapeScoring {

  private static final Distance CoralScoreYOffset = Inches.of(-6.467946);
  private static final Distance CoralDiameter = Inches.of(4.0);

  // Format: 06-11/17-22 1/2/3/4 L/R
  // Ex: 082R, 194R
  private static HashMap<String, Pair<Pose3d, Boolean>> scoringLocations;

  private static ReefscapeScoring m_inst = null;

  private Optional<Pose3d> m_heldCoral = Optional.empty();

  public static ReefscapeScoring getInstance() {

    if (m_inst == null) {
      m_inst = new ReefscapeScoring();
    }

    return m_inst;
  }

  private ReefscapeScoring() {

    scoringLocations = getReefs();
  }

  public HashMap<String, Pair<Pose3d, Boolean>> getReefs() {

    HashMap<String, Pair<Pose3d, Boolean>> map = new HashMap<>();
    List.of(IntStream.range(6, 12), IntStream.range(17, 23)).stream()
        .map(IntStream::boxed)
        .forEach(
            (reefTag) -> {
              reefTag
                  .map(
                      tag ->
                          new Pair<Pose2d, Integer>(PoseCameraIO.getTagPose(tag).toPose2d(), tag))
                  .forEach(
                      (tagPose) -> {
                        List.of(
                                new Pair<Distance, String>(CoralScoreYOffset, "L"),
                                new Pair<Distance, String>(CoralScoreYOffset.times(-1), "R"))
                            .stream()
                            .forEach(
                                yOffset -> {
                                  List.of(
                                          // Height, Depth, Pitch
                                          new Quadruple<Distance, Distance, Angle, Integer>(
                                              Centimeters.of(71),
                                              Centimeters.of(-41),
                                              Degrees.of(35),
                                              2),
                                          new Quadruple<Distance, Distance, Angle, Integer>(
                                              Centimeters.of(111),
                                              Centimeters.of(-35),
                                              Degrees.of(35),
                                              3),
                                          new Quadruple<Distance, Distance, Angle, Integer>(
                                              Centimeters.of(173),
                                              Centimeters.of(-27),
                                              Degrees.of(90),
                                              4),
                                          new Quadruple<Distance, Distance, Angle, Integer>(
                                              Centimeters.of(50),
                                              Centimeters.of(-35),
                                              Degrees.of(-30),
                                              1))
                                      .stream()
                                      .map(
                                          heightAndDepthAndPitch -> {
                                            var transformedPose =
                                                tagPose
                                                    .getFirst()
                                                    .plus(
                                                        new Transform2d(
                                                            heightAndDepthAndPitch
                                                                .getSecond()
                                                                .plus(CoralDiameter.times(2.0)),
                                                            yOffset.getFirst(),
                                                            Rotation2d.kZero));

                                            return new Pair<Pose3d, String>(
                                                new Pose3d(
                                                    new Translation3d(
                                                        transformedPose.getMeasureX(),
                                                        transformedPose.getMeasureY(),
                                                        heightAndDepthAndPitch.getFirst()),
                                                    new Rotation3d(
                                                        Degrees.of(0),
                                                        heightAndDepthAndPitch
                                                            .getThird()
                                                            .unaryMinus(),
                                                        tagPose
                                                            .getFirst()
                                                            .getRotation()
                                                            .getMeasure())),
                                                heightAndDepthAndPitch.getFourth()
                                                    + yOffset.getSecond());
                                          })
                                      .forEach(
                                          loc -> {
                                            map.put(
                                                tagPose.getSecond().toString() + loc.getSecond(),
                                                new Pair<Pose3d, Boolean>(loc.getFirst(), false));
                                          });
                                });
                      });
            });
    return map;
  }

  public void score(String key) {

    scoringLocations.put(
        key, new Pair<Pose3d, Boolean>(scoringLocations.get(key).getFirst(), true));
  }

  public List<Pose3d> getCoral() {

    var scatteredCoral =
        scoringLocations.values().stream()
            .filter(Pair::getSecond) // filter for locs with coral
            .map(Pair::getFirst) // get their poses
            .toList(); // collect

    if (m_heldCoral.isPresent()) {
      scatteredCoral =
          Stream.concat(scatteredCoral.stream(), List.of(m_heldCoral.get()).stream()).toList();
    }

    return scatteredCoral;
  }

  public void setHeldCoral(Optional<Pose3d> heldCoral) {

    m_heldCoral = heldCoral;
  }
}
