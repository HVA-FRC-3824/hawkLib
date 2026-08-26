package frc.shared.hardware.vision.objectVision;

import edu.wpi.first.math.geometry.Translation3d;

// Transform is the transform from the robot center, using robot relative coordinates
  public record ObjectTargetData(
      int objectId, double confidence, Translation3d translation) {}