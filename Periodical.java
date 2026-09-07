// Copyright (c) 2026-2027 FRC 3824 HVA RoHawktics
// http://github.com/HVA-FRC-3824
//
// Use of this source code is governed by an MIT-style license that can be found in the LICENSE file at
// the root directory of this project. Some code may be governed by other licenses which can be found in the "/External Licenses" directory.

package frc.shared;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.HashSet;
import java.util.Set;
import org.littletonrobotics.junction.Logger;

// Effectively piggyback off of the existing periodical framework
// in the commands system
public class Periodical extends SubsystemBase {

  public Set<Runnable> m_runnables = new HashSet<>();

  private static Periodical m_instance;

  private Periodical() {}

  public static void addPeriodic(int runEveryN, Runnable runnable) {

    if (m_instance == null) {
      m_instance = new Periodical();
    }

    m_instance.m_runnables.add(() -> Logger.runEveryN(runEveryN, runnable));
  }

  @Override
  public void periodic() {
    m_runnables.stream().forEach(Runnable::run);
  }
}
