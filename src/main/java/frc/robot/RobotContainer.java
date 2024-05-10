// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.swerve.DefaultDrive;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.swerve.generated.TunerConstants;
import frc.robot.commands.WaitForState;
import frc.robot.commands.lights.AnimateLights;
import frc.robot.lib.RumbleThread;
import frc.robot.lib.Util;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Sweepers;
import frc.robot.subsystems.misc.Lights;
import frc.robot.subsystems.statemachines.Superstructure;
import frc.robot.subsystems.statemachines.Superstructure.SuperState;


public class RobotContainer {
  
  /* Setting up bindings for necessary control of the swerve drive platform */
  private CommandXboxController driver = new CommandXboxController(0);
  private CommandXboxController codriver = new CommandXboxController(1);

  private Swerve mSwerve = TunerConstants.Swerve; 
  private Lights mLights = Lights.getInstance();
  private static final Superstructure mStructure = Superstructure.getInstance();

  public RobotContainer() {

    //! All these functions are needed, please do not touch unless explicitly asked to do so !//

    // Get driver station to stop
    DriverStation.silenceJoystickConnectionWarning(true);

    // Set a custom brownout voltage (from Zach)
    // RobotController.setBrownoutVoltage(6.1);
    // Comment out when we are testing, so we don't mess up the batteries

    // Set up the rumble thread
    RumbleThread.getInstance().bindControllers(driver, codriver);

    // Config swerve
    mSwerve.bindDriverController(driver);
    mSwerve.setDefaultCommand(new DefaultDrive(driver));

    // Setup autos
    configNamedCommands();
    mSwerve.buildAutoChooser();

    mLights.setDefaultCommand(new AnimateLights()); // Run our light animations

    configureDriverBindings(); // Self explanatory
    configureCodriverBindings(); // Self explanatory

    //? Uncomment the following if you want to test stuff ?//
    // configureDevBindings();
    
  }

  private void configureDriverBindings() {

    driver.start().onTrue(Commands.runOnce(mSwerve::requestOdometryReset)); // Reset odometry to a pre-defined pose if all else brakes
    driver.back().onTrue(Commands.runOnce(() -> mSwerve.requestVisionOverride(!mSwerve.getVisionState()))); // Ignore photon updates

    driver.a().whileTrue(Commands.run(mSwerve::requestLock, mSwerve)).onTrue(Commands.runOnce(mStructure::requestClimbAdvance)); // Enter lock pose and request to advance our climb sequence
    driver.y().onTrue(Commands.runOnce(mSwerve::requestGyroReset)); // Reset Gyro
    driver.x().onTrue(mSwerve.getStageAlignCommand()); // Align to closest stage pose
    driver.b().onTrue(Commands.runOnce(mStructure::toggleCleanup)); // Toggle cleanup mode

    // Run the stockpile aim at and move to its setpoint
    driver.povRight().onTrue(Commands.runOnce(mStructure::requestStockpile)).whileTrue(Commands.run(() -> mSwerve.setForceOrientSetpoint(mSwerve.getVelocityCompensatedYaw(Util.getAllianceStockpilePose(), 0))));
    driver.povLeft(); //! Unbound
    driver.povUp(); //! Unbound
    driver.povDown(); //! Unbound

    driver.rightBumper().onTrue(Commands.runOnce(mStructure::requestInterpolating)); // Run auto shoot
    driver.leftBumper(); // Currently bounded to slow mode, keeping this here to make sure no one double binds
    
    driver.rightTrigger(0.1).onTrue(Commands.runOnce(mStructure::requestShoot)); // Shoot 
    driver.leftTrigger(0.1).whileTrue(Commands.run(mStructure::requestIntake)); // Intake

  }

  private void configureCodriverBindings() {

    codriver.a().onTrue(Commands.runOnce(mStructure::requestStowReset)); // Stow and reset everything
    codriver.y().whileTrue(Commands.runEnd(mStructure::requestEjectAndReset, mStructure::requestStow)); // Expel all notes by running everything out and then reset the state machine
    codriver.x().whileTrue(Commands.runEnd(() -> mStructure.requestManualIntake(true), mStructure::requestManualIntakeOff)); // Run manual intake
    codriver.b().whileTrue(Commands.runEnd(() -> mStructure.requestManualIntake(false), mStructure::requestManualIntakeOff)); // Run manual intake

    codriver.povUp().onTrue(Commands.runOnce(mStructure::requestTrap)); // Move elevator to trap
    codriver.povDown().onTrue(Commands.runOnce(mStructure::requestStow)); // Stow
    codriver.povLeft().onTrue(Commands.runOnce(mStructure::requestSubwoofer)); // Move arm to subwoofer
    codriver.povRight(); //! Unbound

    codriver.rightBumper().onTrue(Commands.runOnce(mStructure::requestAmp)); // Move superstructure to amp
    codriver.leftBumper().onTrue(Commands.runOnce(mStructure::requestClimbAndTrap)); // Start climb / trap sequence

    codriver.leftTrigger(0.1); //! Unbound
    codriver.rightTrigger(0.1).onTrue(Commands.runOnce(mStructure::requestPodiumDunk)); // Request elevated shot on podium
    codriver.rightTrigger(0.1).whileTrue(Commands.run(() -> mSwerve.setForceOrientSetpoint(mSwerve.getAbsoluteYawToPose(Util.getSpeakerPose())))); // Aim at speaker

    codriver.start().onTrue(Commands.runOnce(mStructure::requestIdle));
    // codriver.back().whileTrue(new SweepersThrough(12));

  }

  public void configureDevBindings() {
    codriver.a().whileTrue(Commands.runEnd(() -> Sweepers.getInstance().sweepIn(), () -> Sweepers.getInstance().sweepOff()));
    codriver.x().whileTrue(Commands.runEnd(() -> Intake.getInstance().setVelocity(50), () -> Intake.getInstance().setPower(0)));
    codriver.y().whileTrue(Commands.runEnd(() -> Indexer.getInstance().setVelocity(50), () -> Indexer.getInstance().setPower(0)));
    codriver.b().whileTrue(Commands.runEnd(() -> Shooter.getInstance().setVelocity(2000), () -> Shooter.getInstance().setPower(0)));

    
    codriver.povUp().onTrue(Commands.runOnce(() -> mStructure.requestElePos(0.25)));
    codriver.povDown().onTrue(Commands.runOnce(() -> mStructure.requestElePos(0)));

    codriver.povLeft().onTrue(Commands.runOnce(() -> mStructure.requestArmPos(10)));
    codriver.povRight().onTrue(Commands.runOnce(() -> mStructure.requestArmPos(-10)));

    codriver.start().onTrue(Commands.runOnce(mStructure::requestIdle));

  }


  public Command getAutonomousCommand() {
    return mSwerve.getAutoChooser().getSelected();
  }

  private void configNamedCommands() {
    
    NamedCommands.registerCommand("RequestStow", new InstantCommand(mStructure::requestStowReset));

    NamedCommands.registerCommand("RequestInterpolate", new WaitForState(mStructure::requestInterpolating, SuperState.INTERPOLATING));
    NamedCommands.registerCommand("RequestShoot", new WaitForState(mStructure::requestShoot, SuperState.SHOOTING));
    NamedCommands.registerCommand("RequestIntake", new WaitForState(mStructure::requestIntake, SuperState.INTAKING));
    NamedCommands.registerCommand("RequestIntakeHigh", new WaitForState(()-> mStructure.requestIntake(-2), SuperState.INTAKING));
    NamedCommands.registerCommand("RequestRaisedIntake", new WaitForState(() -> mStructure.requestIntake(0), SuperState.INTAKING));

    NamedCommands.registerCommand("TrackSpeaker", Commands.runOnce(() -> mSwerve.setPPLibOverrideRotation(true)));
    NamedCommands.registerCommand("TrackPath", Commands.runOnce(() -> mSwerve.setPPLibOverrideRotation(false)));
    
  }

}
