// Copyright (c) 2026-2027 FRC 3824 HVA RoHawktics
// http://github.com/HVA-FRC-3824
//
// Use of this source code is governed by an MIT-style license that can be found in the LICENSE file at
// the root directory of this project. Some code may be governed by other licenses which can be found in the "/External Licenses" directory.

package frc.shared.hardware;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.Pair;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import java.util.HashMap;
import java.util.List;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class SimBattery {

  private static HashMap<String, Supplier<Current>> m_currents =
      new HashMap<String, Supplier<Current>>();

  private static Voltage m_supplyVoltage = Volts.of(12.0);

  public static void registerDevice(String name, Supplier<Current> supplier) {

    m_currents.put(name, supplier);
  }

  public static void calculateSupplyVoltage() {

    // Getting the Thristiest Boi like this is not the best way to do it,
    // but we wont be using this outside of testing.
    List<Pair<String, Current>> ampPairs =
        m_currents.entrySet().stream()
            .map(entry -> new Pair<String, Current>(entry.getKey(), entry.getValue().get()))
            .toList();

    var ampStream = ampPairs.stream().mapToDouble(amp -> amp.getSecond().in(Amps)).distinct();

    ampStream
        .max()
        .ifPresent(
            max -> {
              ampPairs.stream()
                  .filter(amp -> max == amp.getSecond().in(Amps))
                  .forEach(amp -> Logger.recordOutput("Thirstiest Boi", amp.getFirst()));
            });

    m_supplyVoltage = Volts.of(BatterySim.calculateDefaultBatteryLoadedVoltage());
  }

  public static Voltage getSupplyVoltage() {

    return m_supplyVoltage;
  }
}
