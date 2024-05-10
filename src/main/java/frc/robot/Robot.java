// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import com.ctre.phoenix6.signals.NeutralModeValue;
import com.pathplanner.lib.pathfinding.Pathfinding;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.lib.Util;
import frc.robot.lib.VirtualSubsystem;
import frc.robot.subsystems.misc.Lights;
import frc.robot.subsystems.misc.ShotAngleInterpolator;
import frc.robot.subsystems.swerve.LocalADStarAK;
import frc.robot.subsystems.swerve.generated.TunerConstants;

public class Robot extends LoggedRobot  {
  private Command mAutonomousCommand;
  private RobotContainer mRobotContainer;
  public PowerDistribution mPDH;

  @Override
  public void robotInit() {

    this.handleADKStartup();
    ShotAngleInterpolator.getInstance().addInitialInterpolationData();
    mRobotContainer = new RobotContainer();

  }

  @Override
  public void robotPeriodic() {
    VirtualSubsystem.periodicAll();
    CommandScheduler.getInstance().run(); 
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {
    Lights.getInstance().tickWave(Util.isOnBlueAlliance() ? Color.kTeal : new Color(0.55, 0.02, 0.0075), Color.kBlack, 15, 1);
  }

  @Override
  public void disabledExit() {
    Lights.getInstance().off();
  }

  @Override
  public void autonomousInit() {

    mAutonomousCommand = mRobotContainer.getAutonomousCommand();
    if (mAutonomousCommand != null) {
      mAutonomousCommand.schedule();
    }

    TunerConstants.Swerve.configNeutralMode(NeutralModeValue.Brake);

  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {

    if (mAutonomousCommand != null) {
      mAutonomousCommand.cancel();
    }

    TunerConstants.Swerve.configNeutralMode(NeutralModeValue.Coast);
    
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}

  @Override
  public void simulationPeriodic() {}

  private void handleADKStartup () {

    Pathfinding.setPathfinder(new LocalADStarAK());

    Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
    Logger.recordMetadata("Build Date", BuildConstants.BUILD_DATE);
    Logger.recordMetadata("Git Branch", BuildConstants.GIT_BRANCH);
    Logger.recordMetadata("Name", BuildConstants.MAVEN_NAME);

    if (isReal()) {
      Logger.addDataReceiver(new WPILOGWriter()); // Log to a USB stick ("/U/logs")
      Logger.addDataReceiver(new NT4Publisher()); // Publish data to NetworkTables
      mPDH = new PowerDistribution(1, ModuleType.kRev); // Enables power distribution logging
    } else {
      setUseTiming(false); // Run as fast as possible
      String logPath = LogFileUtil.findReplayLog(); // Pull the replay log from AdvantageScope (or prompt the user)
      Logger.setReplaySource(new WPILOGReader(logPath)); // Read replay log
      Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim"))); // Save outputs to a new log
    }

    // Logger.disableDeterministicTimestamps() // See "Deterministic Timestamps" in the "Understanding Data Flow" page
    Logger.start(); // Start logging! No more data receivers, replay sources, or metadata values may be added.

  }
}
