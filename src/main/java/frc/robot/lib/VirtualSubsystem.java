package frc.robot.lib;

// From FRC 6328 http://github.com/Mechanical-Advantage

import java.util.ArrayList;
import java.util.List;

public abstract class VirtualSubsystem {
  private static List<VirtualSubsystem> subsystems = new ArrayList<>();

  public VirtualSubsystem() {
    subsystems.add(this);
  }

  public static void periodicAll() {
    for (VirtualSubsystem subsystem : subsystems) {
      subsystem.periodic();
    }
  }

  public static void simulationPeriodicAll() {
    for (VirtualSubsystem subsystem : subsystems) {
      subsystem.simulationPeriodic();
    }
  }

  public abstract void periodic();
  public abstract void simulationPeriodic();
}