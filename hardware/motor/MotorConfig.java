// Copyright (c) 2026-2027 FRC 3824 HVA RoHawktics
// http://github.com/HVA-FRC-3824
//
// Use of this source code is governed by an MIT-style license that can be found in the LICENSE file at
// the root directory of this project. Some code may be governed by other licenses which can be found in the "/External Licenses" directory.

package frc.shared.hardware.motor;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;

import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import lombok.Getter;
import lombok.With;

@With
public class MotorConfig {

  @Getter Current supplyCurrent = Amps.of(70.0);
  @Getter Current statorCurrent = Amps.of(120.0);
  @Getter boolean inverted = false;
  @Getter boolean brakeMode = true;
  @Getter boolean continuousWrap = false;
  @Getter double P = 0.0;
  @Getter double I = 0.0;
  @Getter double D = 0.0;
  @Getter double S = 0.0;
  @Getter double V = 0.0;
  @Getter double A = 0.0;
  @Getter double G = 0.0;
  @Getter AngularVelocity velocityLimit = RotationsPerSecond.of(600.0);
  @Getter AngularAcceleration accelerationLimit = RotationsPerSecondPerSecond.of(6000.0);
  @Getter double sensorToMechanismRatio = 1.0;

  public MotorConfig() {}

  // Needed for lombok @With
  public MotorConfig(
      Current supplyCurrent,
      Current statorCurrent,
      boolean inverted,
      boolean brakeMode,
      boolean continuousWrap,
      double P,
      double I,
      double D,
      double S,
      double V,
      double A,
      double G,
      AngularVelocity velocityLimit,
      AngularAcceleration accelerationLimit,
      double sensorToMechanismRatio) {

    this.supplyCurrent = supplyCurrent;
    this.statorCurrent = statorCurrent;
    this.inverted = inverted;
    this.brakeMode = brakeMode;
    this.continuousWrap = continuousWrap;
    this.P = P;
    this.I = I;
    this.D = D;
    this.S = S;
    this.V = V;
    this.A = A;
    this.G = G;
    this.velocityLimit = velocityLimit;
    this.accelerationLimit = accelerationLimit;
    this.sensorToMechanismRatio = sensorToMechanismRatio;
  }
}
