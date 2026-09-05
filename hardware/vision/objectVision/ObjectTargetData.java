// Copyright (c) 2026-2027 FRC 3824 HVA RoHawktics
// http://github.com/HVA-FRC-3824
//
// Use of this source code is governed by an MIT-style license that can be found in the LICENSE file at
// the root directory of this project. Some code may be governed by other licenses which can be found in the "/External Licenses" directory.

package frc.shared.hardware.vision.objectVision;

import edu.wpi.first.math.geometry.Translation3d;

// Transform is the transform from the robot center, using robot relative coordinates
public record ObjectTargetData(int objectId, double confidence, Translation3d translation) {}
