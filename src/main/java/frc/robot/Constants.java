package frc.robot;

import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Filesystem;
import frc.robot.lib.CanDeviceId;
import java.io.File;
import java.util.ArrayList;
import java.util.List;
import java.util.stream.Stream;

public class Constants {
    public static class SwerveConstants {

        public static final double kControllerDefaultDeadband = 0.1;
        public static final double kControllerExitDeadband = 0.1;

        public static PathConstraints kDefaultPathConstraints = new PathConstraints(
            5.0, 5.0,
            Units.degreesToRadians(720), Units.degreesToRadians(720)
        );

        public static final PIDConstants kSnapPID = new PIDConstants(5, 5, 0);
        public static final double kSnapTolerancePos = Units.degreesToRadians(8);
        public static final double kSnapToleranceVel = 10;

        // From Phoenix tuner
        public static final double kMountPosePitch = 1.5680853128433228;
        public static final double kMountPoseRoll = -2.3384628295898438;
        public static final double kMountPoseYaw = -91.6754150390625;

        public static final double kOverrideStickXPercent = 0.2;
        public static final double kOverrideStickYPercent = 0.2;
        public static final double kOverrideStickRotPercent = 0.3;

        public static final PIDConstants kAutoAlignXPID = new PIDConstants(1.1, 0, 0);
        public static final PIDConstants kAutoAlignYPID = new PIDConstants(1.1, 0, 0);
        public static final double kAutoAlignRotationalTolerance = 5; // deg
        public static final double kAutoAlignPositionalTolerance = 0.05;
        public static final double kAutoAlignMaxEndingVelocity = 0.1;


        // I am deeply sorry for doing this
        public static final ArrayList<Pose2d> kBlueStagePositions = new ArrayList<>();
        static {
            kBlueStagePositions.add(new Pose2d(4.35, 4.95, Rotation2d.fromDegrees(300)));
            kBlueStagePositions.add(new Pose2d(4.35, 3.25, Rotation2d.fromDegrees(60)));
            kBlueStagePositions.add(new Pose2d(5.85, 4.10, Rotation2d.fromDegrees(180)));
        }

        // I am deeply sorry for doing this
        public static final ArrayList<Pose2d> kRedStagePositions = new ArrayList<>();
        static {
            kRedStagePositions.add(new Pose2d(12.20, 4.95, Rotation2d.fromDegrees(-300)));
            kRedStagePositions.add(new Pose2d(12.20, 3.25, Rotation2d.fromDegrees(-60)));
            kRedStagePositions.add(new Pose2d(10.7, 4.10, Rotation2d.fromDegrees(-180)));
        }

    }

    public static class AutoConstants {

        public static final PIDConstants kTranslationPID = new PIDConstants(0.5, 0, 0);
        public static final PIDConstants kRotationPID = new PIDConstants(6, 0, 0);
        public static final ReplanningConfig kReplanningConfig = new ReplanningConfig(true, true);
        public static final double kMaxModuleSpeed = 4.5; // m / s
        public static final double kDriveBaseRadius = 1; // m

        public static final HolonomicPathFollowerConfig kHolonomicPathFollowerConfig = new HolonomicPathFollowerConfig(
            Constants.AutoConstants.kTranslationPID, 
            Constants.AutoConstants.kRotationPID, 
            Constants.AutoConstants.kMaxModuleSpeed,
            Constants.AutoConstants.kDriveBaseRadius,
            Constants.AutoConstants.kReplanningConfig
        );

        public static final List<File> deployDirectoryFiles = Stream.of(new File(Filesystem.getDeployDirectory() + "/pathplanner/paths").listFiles()).filter(File::isFile).toList();
    }
    
    public class VisionConstants {
        
        public static final double kVisionRotationRejectionEpsilon = 5; // deg
        public static final double kVisionPositionRejectionEpsilon = 1; // m

        public static final Rotation3d kCameraRotation = new Rotation3d(0, Units.degreesToRadians(-40 + 15), Units.degreesToRadians(180));
        public static final Transform3d kCameraTransform = new Transform3d(Units.inchesToMeters(-12.592 - 4), Units.inchesToMeters(-6.714), Units.inchesToMeters(22.203), kCameraRotation);

        public static final boolean kScaleStandardDevBasedOnDistanceAndAmbiguity = true;
        public static final double kDistanceScaleFactor = 2;
        public static final double kAmbiguityScaleFactor = 2;

        public static final double kAmbiguityRejectionLimit = 0.5;
        public static final double kDistanceRejectionLimit = 6.1;

        public static final boolean kRejectOnVelocity = false;
        public static final double kMaxXVelocity = 2.5; // m / s
        public static final double kMaxYVelocity = 2.5; // m / s
        public static final double kMaxRotationalVelocity = 0.17; // rad / s - (~ 10 deg / s)

    }

    public class ShootingConstants {

        public static final double kMinDistanceFromTarget = 2; // m
        public static final double kMaxSpeedWhenShooting = 2; // m / s

    }

    public class IntakeConstants {

        public static final CanDeviceId kLeftSweeperId = new CanDeviceId(2);
        public static final CanDeviceId kRightSweeperId = new CanDeviceId(4);
        public static final CanDeviceId kIntakeId = new CanDeviceId(5);

        public static final PIDConstants kIntakePID = new PIDConstants(0.5, 0, 0); 
        public static final double kIntakeFF = 0.12;
        public static final double kS = 0.8;

        public static final double kMaxOutput = 1; 
        public static final double kMinOutput = -1;
        

    }

    public static class ShooterConstants {
        
        public static final CanDeviceId kBottomRollerId = new CanDeviceId(41);
        public static final CanDeviceId kTopRollerId = new CanDeviceId(42);
        public static final CanDeviceId kIndexerId = new CanDeviceId(43);

        public static final PIDConstants kShooterPID = new PIDConstants(0.6, 0, 0); 
        // max rpm of the Falcon500
        public static final double kMaxRPM = 6380.0;
        // max rps of the Falcon500
        public static final double kMaxRPS = kMaxRPM / 60.0;
        // feedforward calculated in units of Volts / RPS 
        // if we take the max voltage of the motor and divide by the max rotational velocity,
        // we get the correct (theoretical) kV. 
        // this could maybe be tuned a little bit, but fine for now
        public static final double kShooterFF =  12.0 / kMaxRPS;

        public static final double kShooterKs = 0.3;


        public static final double kMaxOutput = 1; 
        public static final double kMinOutput = -1;
        
        public static final PIDConstants kIndexerPID = new PIDConstants(0.5, 0, 0); 
        public static final double kIndexerFF = 0.12;
        public static final double kIndexerKs = 0.4;

        public static final double kVelocityShootToleranceRPM = 120;

        public static final double kAverageNoteVelocity = 5.71788;
        public static final double kShotCompensationDelta = 0.02;
        public static final double kMaxShotDistance = 4; // this is the last defined point for the interpolator

    }

    public static class ArmConstants {

        public static final CanDeviceId kEncoderId = new CanDeviceId(47);
        public static final CanDeviceId kPivotId = new CanDeviceId(45);
        public static final CanDeviceId kPivotIdFollower = new CanDeviceId(46);

        public static final double kArmRatio = 40.0;

        public static final double kArmSafeToMove = 35;

        public static final double kMotionMagicCruiseVel = 800; 
        public static final double kMotionMagicAcceleration = 2000;

        public static final double kMotionMagicJerk = kMotionMagicAcceleration * 3; 

        public static final double kPivotP = 2000;
        public static final double kPivotI = 0;
        public static final double kPivotD = 0;
        public static final double kG = 0.7; // 0.9 - 0.5
        public static final double kS = 0.2; //measured 0.165;
        public static final double kV = 8.435;
        
        public static final double kTopSafe = 85;

        // all pivot angles are relative to the metal pivot

        // public static final double kOffsetCGToArm = -10; // angle between CG and metal arm (from pivot)
        // public static final double kOffsetArmToShooter = -39.4 - 1.26; // angle between shooter plate and metal structure

        public static final double kOffsetCGToArm = -10.6; // -8 * 53.333 / 40 to account for incorrect gearing
        public static final double kOffsetArmToShooter = -43.9; // angle between shooter plate and metal structure

        public static final double kArmPositionalToleranceDegrees = 2;
        public static final double kArmVelocityTolerance = 1;

        // public static final double kHardStopLimit = 8;
    }

    public static class ElevatorConstants {

        public static final CanDeviceId kLeaderId = new CanDeviceId(6, "DrivebaseCanivore");
        public static final CanDeviceId kFollowerId = new CanDeviceId(7, "DrivebaseCanivore");

        public static final boolean kOpposeMaster = true;

        public static final double kElevatorSafeToMove = 0.02;

        //for conversions
        public static final double kGearRatio = (37 / 15) * 5; // ~12.33333
        public static final double kWheelCircumference = Units.inchesToMeters(3.5); //0.0889 meters

        public static final double kEleTopPosition = 0.34;

        public static final double kMotionMagicKA = 0;
        public static final double kMotionMagicKV = 12; // 12
        public static final double kMotionMagicKP = 280;
        public static final double kMotionMagicKI = 0;
        public static final double kMotionMagicKD = 0;
        public static final double kMotionMagicKG = 0.2;
        public static final double kMotionMagicKS = 0.1;
        
        public static final double kTrapP = 3;
        public static final double kTrapI = 0;
        public static final double kTrapD = 0;
        public static final double kTrapG = -0.7;
        public static final double kTrapS = 0.135;
        public static final double kTrapV = 0.12;

        public static final double kMotionMagicCruiseVel = 3;
        public static final double kMotionMagicAcceleration = 3;
        public static final double kMotionMagicJerk = 0;
      
        // for hard stop
        // public static final double kHardStopLimit = 10;
        public static final double kElevatorPositionalTolerance = .0025;

    }

    public static class LEDConstants {

        public static final int kLedPort = 9;
        public static final int kLedLength = 30;

    }

    public static class HandoffConstants {

        public static final int kIntakeBeamBreakPort = 9;
        public static final int kShooterBeamBreakPort = 7;
        public static final int kIndexerBeamBreakPort = 8;
        public static final int kFloorBeamBreakPort = 0;

    }

}
