package frc.robot.lib;

import java.util.function.Supplier;

import org.photonvision.EstimatedRobotPose;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants;
import frc.robot.subsystems.swerve.generated.TunerConstants;

public class Util {

  public static final double kEpsilon = 1e-12;

  public static double deadBand(double val, double deadband){
    return (Math.abs(val) > Math.abs(deadband)) ? val : 0.0;
  }

  public static boolean epsilonEquals(double a, double b, double epsilon) {
    return Math.abs(a - b) < epsilon;
  }

  public static boolean epsilonEquals(double a, double b) {
    return epsilonEquals(a, b, kEpsilon);
  }

  public static boolean epsilonEquals(int a, int b, int epsilon) {
      return (a - epsilon <= b) && (a + epsilon >= b);
  }

  public static boolean epsilonEquals(EstimatedRobotPose a, Rotation2d b, double epsilon) {
    return (a.estimatedPose.getRotation().toRotation2d().getDegrees() - epsilon <= b.getDegrees()) && (a.estimatedPose.getRotation().toRotation2d().getDegrees() + epsilon >= b.getDegrees());
  }

  public static boolean epsilonEquals(Pose2d a, Pose2d b, double epsilonX, double epsilonY) {
    return epsilonEquals(a.getX(), b.getX(), epsilonX) && epsilonEquals(a.getY(), b.getY(), epsilonY);
  }

  public static boolean epsilonEquals(Pose2d a, Pose2d b, double epsilon) {
    return epsilonEquals(a.getX(), b.getX(), epsilon) && epsilonEquals(a.getY(), b.getY(), epsilon);
  }

  public static boolean isOnBlueAlliance() {
    return DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue;
  }

  public static Pose2d getSpeakerPose() {
    return new Pose2d(isOnBlueAlliance() ? 0 : 16.6, 5.5, Rotation2d.fromDegrees(isOnBlueAlliance() ? 0 : 180)); // All in meters
  }

  public static Pose2d getDebugAutoAlignPose() {
    // return new Pose2d(2.38, 5.588, Rotation2d.fromDegrees(0));
    return new Pose2d(14.293, 5.588, Rotation2d.fromDegrees(0));
  }

  public static Pose2d getDebugAmpPose() {
    // return new Pose2d(2.38, 5.588, Rotation2d.fromDegrees(0));
    return new Pose2d(13.669, 7.525, Rotation2d.fromDegrees(90));
  }
  
  public static double clamp(double value, double min, double max) {
    return Math.max(min, Math.min(max, value));
  }   

  public static Pose2d getAmpPose() {
    return isOnBlueAlliance() ? new Pose2d(5, 5, Rotation2d.fromDegrees(90)) : new Pose2d(14.038, 7.18, Rotation2d.fromDegrees(90));
  }

  public static Pose2d getClosestStagePose(Supplier<Pose2d> referenceSup) {
    Pose2d reference = referenceSup.get();
    return reference.nearest(isOnBlueAlliance() ? Constants.SwerveConstants.kBlueStagePositions: Constants.SwerveConstants.kRedStagePositions);
  }

  public static Pose2d getAllianceStockpilePose() {
    return isOnBlueAlliance() ? new Pose2d(0.61, 7.47, new Rotation2d()) : new Pose2d(15.74, 7.47, new Rotation2d());
  }

  public static double getTimeToTarget(double dist) {

    // for alpha
    // double a = 0.0623564;
    // double b = 0.391244;
    
    double a = 0.0302358;
    double b = 0.466889;

    return (a * dist) + b;

  }

  public static Pose2d getFuturePosition(double dt) {

    // csys of chassis speed is flipped compared to robot pose on red alliance
    if(!Util.isOnBlueAlliance()) {
        dt *= -1;
    }

    ChassisSpeeds speeds = TunerConstants.Swerve.getFieldRelativeSpeeds();
    Pose2d robot = TunerConstants.Swerve.getPose();

    double dx = robot.getX() + (speeds.vxMetersPerSecond * dt);
    double dy = robot.getY() + (speeds.vyMetersPerSecond * dt);

    return new Pose2d(dx, dy, new Rotation2d());

  }
  
}