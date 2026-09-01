// Copyright (c) 2026-2027 FRC 3824 HVA RoHawktics
// http://github.com/HVA-FRC-3824
//
// Use of this source code is governed by an MIT-style license that can be found in the LICENSE file at
// the root directory of this project.

package frc.shared.hardware.motor;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

public interface MotorIO {

  public default void periodic() {}

  public default void config(MotorConfig config) {}

  public default int getId() {
    return 0;
  }

  public default void follow(int id, boolean inverted) {}

  public default void brake() {}

  public default void setPosition(Angle angle) {}

  public default void setVelocity(AngularVelocity angleVel) {}

  public default void setVoltage(Voltage volts) {}

  public default void resetEncoder(Angle angle) {}

  @AutoLog
  public static class MotorInputs {

    public AngularVelocity velocity = RotationsPerSecond.of(0.0);
    public Angle position = Degrees.of(0.0);
    public Angle rawPosition = Degrees.of(0.0);
    public Voltage appliedVolts = Volts.of(0.0);
    public Current statorCurrent = Amps.of(0.0);
    public Temperature temperature = Celsius.of(0.0);
    public double lastReference = 0.0;
  }

  public default void updateInputs(MotorInputs inputs) {}
}
