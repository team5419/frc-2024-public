package frc.robot.subsystems.swerve;

import java.io.File;
import java.util.Optional;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonTrackedTarget;

import com.ctre.phoenix6.configs.MountPoseConfigs;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.utility.PhoenixPIDController;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.ProxyCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.commands.swerve.SequentialAutoAlign;
import frc.robot.lib.Util;
import frc.robot.subsystems.swerve.generated.TunerConstants;

/**
 * Class that extends the Phoenix SwerveDrivetrain class and implements subsystem
 */
public class Swerve extends SwerveDrivetrain implements Subsystem {

    public enum SwerveState {
        OPEN_LOOP,
        PATH_FOLLOWING,
        FORCE_ORIENT,
        LOCK,
        IDLE
    }

    private SwerveState mCurrentState;
    private SwerveRequest mDesiredRequest;

    private boolean mHasExitedForceOrient;
    private boolean mHasBoundControllers;
    private boolean mIsInterpolatingShots;

    private SwerveRequest.ApplyChassisSpeeds mReqChassisSpeeds = new SwerveRequest.ApplyChassisSpeeds();
    private SwerveRequest.FieldCentric mReqDrive = new SwerveRequest.FieldCentric().withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    private SwerveRequest.SwerveDriveBrake mReqBrake = new SwerveRequest.SwerveDriveBrake();
    private SwerveRequest.FieldCentricFacingAngle mReqForceOrient = new SwerveRequest.FieldCentricFacingAngle();
    private SwerveRequest.Idle mReqIdle = new SwerveRequest.Idle();

    private final Rotation2d mBlueAlliancePerspectiveRotation = Rotation2d.fromDegrees(0); /* Blue alliance sees forward as 0 degrees (toward red alliance wall) */
    private final Rotation2d mRedAlliancePerspectiveRotation = Rotation2d.fromDegrees(180); /* Red alliance sees forward as 180 degrees (toward blue alliance wall) */
    private boolean mHasAppliedOperatorPerspective = false; /* Keep track if we've ever applied the operator perspective before or not */
  
    private PhotonCamera mCamera;
    private AprilTagFieldLayout mLayout;
    private PhotonPoseEstimator mEstimator;
    private boolean mIsPhotonEstimating, mPhotonSetupSuccess;

    private SwerveTelemetry mSwerveTelemetry = new SwerveTelemetry(TunerConstants.kMaxSpeed);

    private Supplier<Double> mRotSup, mXSup, mYSup;
    private CommandXboxController mDriverController;

    private SendableChooser<Command> mAutoChooser;
    boolean isOverridingPathFollowingRotation;

    public Swerve(SwerveDrivetrainConstants driveTrainConstants, SwerveModuleConstants... modules) {

        super(driveTrainConstants, modules); // OPTIONAL: could pass in a custom odometry frequency as the second argument and the modules as the third argument
        
        configAutos();
        System.out.println("[SWERVE] [INIT] AutoBuilder Configured");

        registerTelemetry(mSwerveTelemetry::telemeterize);
        System.out.println("[SWERVE] [INIT] Telemetry registered");

        mCurrentState = SwerveState.IDLE;
        mDesiredRequest = mReqIdle;

        MountPoseConfigs mMountCfg = new MountPoseConfigs()
            .withMountPosePitch(Constants.SwerveConstants.kMountPosePitch)
            .withMountPoseRoll(Constants.SwerveConstants.kMountPoseRoll)
            .withMountPoseYaw(Constants.SwerveConstants.kMountPoseYaw);

        getPigeon2().getConfigurator().apply(mMountCfg);

        PhoenixPIDController headingController = mReqForceOrient.HeadingController;
        headingController.setPID(Constants.SwerveConstants.kSnapPID.kP, Constants.SwerveConstants.kSnapPID.kI, Constants.SwerveConstants.kSnapPID.kD);
        headingController.setTolerance(Constants.SwerveConstants.kSnapTolerancePos, Constants.SwerveConstants.kSnapToleranceVel);
        headingController.enableContinuousInput(0, Math.PI * 2);
        mHasExitedForceOrient = false;

        requestEnableCoastMode(true);

        mHasBoundControllers = false;
        mIsInterpolatingShots = false;
        mIsPhotonEstimating = false;
      
        try { // Try to set up vision
      
            mLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2024Crescendo.m_resourceFile);
            mCamera = new PhotonCamera("OV9281_0");
            mEstimator = new PhotonPoseEstimator(mLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, mCamera, Constants.VisionConstants.kCameraTransform);
            mEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
            mIsPhotonEstimating = true;
            mPhotonSetupSuccess = true;

            System.out.println("[SWERVE] [INFO] [APRILTAG] PhotonVision setup complete, fusing measurements with odometry");

        } catch (Exception e) {

            DriverStation.reportError("[SWERVE] [ERR] [APRILTAG] [INIT]" + e.getMessage(), e.getStackTrace());
            mIsPhotonEstimating = false;
            mPhotonSetupSuccess = false;

        }

    }

    /**
     * Main function that is ran during teleop. It is responsible for setting the swerve's setpoint based on generic input. It also handles force orient, and when to break out of it
     * @param y - the y input supplier
     * @param x - the x input supplier
     * @param rotation - the rotation supplier
     * @param slowMode - supplier to tell if we should scale down our inputs (ie slow them down)
     */
    public void feedTeleopValues (Supplier<Double> y, Supplier<Double> x, Supplier<Double> rotation, Supplier<Boolean> slowMode) {
        if(mCurrentState != SwerveState.OPEN_LOOP && mCurrentState != SwerveState.FORCE_ORIENT && !DriverStation.isAutonomous()) {
            mCurrentState = SwerveState.OPEN_LOOP;
        }

        if(mCurrentState == SwerveState.FORCE_ORIENT) {

            // If the controllers are giving an input over some value, the exit out of force orient
            if (Math.abs(rotation.get()) > Constants.SwerveConstants.kOverrideStickRotPercent) { 
                mCurrentState = SwerveState.OPEN_LOOP;
                return;
            } 

            passFaceAngleValues(x.get(), y.get(), slowMode.get()); 
            mDesiredRequest = mReqForceOrient;

        } else if (mCurrentState == SwerveState.OPEN_LOOP) {

            passDriveRequestValues(x.get(), y.get(), rotation.get(), slowMode.get());
            mDesiredRequest = mReqDrive;

        } else {

            mDesiredRequest = mReqBrake;

        }
    }

    // Helper for our auto align
    public void feedTeleopValues(double x, double y) {
        feedTeleopValues(() -> x, () -> y, () -> 0.0, () -> false);
    }

    /**
     * Set the goal for force orient, and enable tracking of the goal
     * @param setpoint - the Rotation2d to track, relative to the robot's center
     */
    public void setForceOrientSetpoint(Rotation2d setpoint) {
        if(mCurrentState != SwerveState.FORCE_ORIENT) {
            mCurrentState = SwerveState.FORCE_ORIENT;
        }

        mReqForceOrient.withTargetDirection(setpoint);
        mReqForceOrient.HeadingController.reset();
        mHasExitedForceOrient = false;
    }

    /**
     * Request the swerve to enter a "lock" pose, or where the wheels form a "X"
     */
    public void requestLock() {
        if(mCurrentState != SwerveState.LOCK && mCurrentState == SwerveState.OPEN_LOOP) {
            mCurrentState = SwerveState.LOCK;
        }

        mDesiredRequest = mReqBrake;
    }

    /**
     * Auto path-find to pose using pathplanner
     * @param target - target pose
     * @param meterRotationDelay - delay to start rotating
     * @return a command to schedule
     */
    public Command goToPointCommand(Pose2d target, double meterRotationDelay, boolean overrideExitOverride) {

        // Will handle state machine stuff in the driveRobotRelative func

        if(!mHasBoundControllers) {
            return new PrintCommand("[SWERVE] [WARN] [GOTOPOINT] Exit controllers not bound, exiting goToPoint() execution");
        }

        if(!Util.isOnBlueAlliance()) {
            target = target.plus(new Transform2d(0, 0, Rotation2d.fromDegrees(180)));
        }

        return AutoBuilder.pathfindToPose(
            target, Constants.SwerveConstants.kDefaultPathConstraints, 0, meterRotationDelay
        ).onlyWhile(() -> driverIsNotOverriding() || overrideExitOverride);
    }

    /**
     * Check if the driver controller is past the "overriding" point
     * @return true if the driver wants to exit early
     */
    public boolean driverIsNotOverriding() {
         if(!mHasBoundControllers) {
            System.out.println("[SWERVE] [WARN] [GETOVERRIDE] Exit controllers not bound, returning false on GetDriverOverride");
            return false;
        }

        return (Math.abs(mRotSup.get()) < Constants.SwerveConstants.kOverrideStickRotPercent 
            && Math.abs(mXSup.get()) < Constants.SwerveConstants.kOverrideStickXPercent
            && Math.abs(mYSup.get()) < Constants.SwerveConstants.kOverrideStickYPercent);
    }

    /**
     * Bind the driver controller for the driverIsNotOverriding function
     * @param controller - the driver controller
     */
    public void bindDriverController(CommandXboxController controller) {

        if (mHasBoundControllers) return;

        mRotSup = controller::getRightX;
        mXSup = controller::getLeftX;
        mYSup = controller::getRightY;

        mHasBoundControllers = true;

        mDriverController = controller;
        
    }

    /**
     * Get the heading controller
     * @return the heading controller
     */
    public PhoenixPIDController getHeadingController() {
        return mReqForceOrient.HeadingController;
    }

    /**
     * If the force orient PID controller is at it's goal, using the default tolerances 
     * @return true if the controller is at the setpoint
     */
    public boolean forceOrientAtSetpoint() {
        return getHeadingController().atSetpoint();
    }

    @Override
    public void periodic() {

        handleOperatorPerspectiveFlip();

        if (mIsPhotonEstimating) {
            handleVision();
        }

        handleDriveState();

        Logger.recordOutput("Swerve/Pose", getPose());
        Logger.recordOutput("Swerve/Closest Speaker Pose", Util.getClosestStagePose(this::getPose));
        Logger.recordOutput("Swerve/Current State", mCurrentState);
        Logger.recordOutput("Swerve/Heading PID Error", mReqForceOrient.HeadingController.getPositionError());
        Logger.recordOutput("Swerve/Distance From Speaker", Util.getSpeakerPose().getTranslation().getDistance(getPose().getTranslation()));


        // For tuning
        double distance = PhotonUtils.getDistanceToPose(getPose(), Util.getSpeakerPose());
        double timeToTarget = Util.getTimeToTarget(distance);
        Logger.recordOutput("Interpolation/Time to target", timeToTarget);

    }

    /**
     * Where the state machine happens.
     * At the moment the only important part is that we will exit out of the FORCE_ORIENT state when we are at the setpoint, and that we apply the control request at the bottom
     */
    private void handleDriveState() {
        switch (mCurrentState) {
            case IDLE:
                break;

            case OPEN_LOOP:
                break;

            case LOCK:
                break;

            case FORCE_ORIENT:
                if(mReqForceOrient.HeadingController.atSetpoint() && !mHasExitedForceOrient) {
                    mCurrentState = SwerveState.OPEN_LOOP;
                    mHasExitedForceOrient = true;
                }
                break;

            case PATH_FOLLOWING:
                break;
        
            default:
                System.out.println("[SWERVE] [WARN] In unaccounted state, please add to switch statement");
                mDesiredRequest = mReqBrake;
                break;
        }

        setControl(mDesiredRequest);
    }

    /**
     * Function that handles our vision.
     * 
     * We decided to use PhotonVision, with the PhotonPoseEstimator class, witch proved to be more of a pain than its worth.
     * 
     * We choose to look at vision as more of a reinforcement of the odometry, and less of a main factor of it, hence we do so much rejection
     */
    private void handleVision () {
        Optional<EstimatedRobotPose> estimation = mEstimator.update();
        if(estimation.isPresent()) {

            EstimatedRobotPose estimate = estimation.get();

            double devX = 0.9; // m
            double devY = 0.9; // m
            double devRot = 3; // rad

            // If velocity too high then reject
            if(Constants.VisionConstants.kRejectOnVelocity) {
                ChassisSpeeds speeds = getFieldRelativeSpeeds();
                if( Math.abs(speeds.vxMetersPerSecond) > Constants.VisionConstants.kMaxXVelocity 
                 || Math.abs(speeds.vyMetersPerSecond) > Constants.VisionConstants.kMaxYVelocity 
                 || Math.abs(speeds.omegaRadiansPerSecond) > Constants.VisionConstants.kMaxRotationalVelocity) {
                    Logger.recordOutput("Vision/Rejection Reason", "Rejecting on swerve speed");
                    return;
                }
            }

            if (Constants.VisionConstants.kScaleStandardDevBasedOnDistanceAndAmbiguity) {

                double totalDistance = 0;
                double totalAmbiguity = 0;
                double tagsUsed = estimate.targetsUsed.size();

                for(PhotonTrackedTarget target : estimate.targetsUsed) {

                    totalAmbiguity += target.getPoseAmbiguity();
                    totalDistance += target.getBestCameraToTarget().getTranslation().getNorm();
                    // System.out.println("Tag " + target.getFiducialId() + " is " + target.getBestCameraToTarget().getTranslation().getNorm() + " away");

                }

                double averageAmbiguity = totalAmbiguity / tagsUsed;
                double averageDistance = totalDistance / tagsUsed;

                // If average ambiguity is too high then reject 
                if(averageAmbiguity >= Constants.VisionConstants.kAmbiguityRejectionLimit) {
                    Logger.recordOutput("Vision/Rejection Reason", "Rejecting on ambiguity, calculated ambiguity is: " + averageAmbiguity);
                    return;
                };

                // If average distance if too hight then reject
                if(averageDistance >= Constants.VisionConstants.kDistanceRejectionLimit) {
                    Logger.recordOutput("Vision/Rejection Reason", "Rejecting on distance, calculated distance: " + averageDistance);
                    return;
                };

                // devX *= (averageDistance * Constants.VisionConstants.kDistanceScaleFactor) * (averageAmbiguity * Constants.VisionConstants.kDistanceScaleFactor);
                // devY *= (averageDistance * Constants.VisionConstants.kDistanceScaleFactor) * (averageAmbiguity * Constants.VisionConstants.kDistanceScaleFactor);
                // devRot *= (averageDistance * Constants.VisionConstants.kDistanceScaleFactor) * (averageAmbiguity * Constants.VisionConstants.kDistanceScaleFactor);

                // Simple band-aid as we did not have time to test out the scaling above
                if(averageDistance > 4) {
                    devX *= 2;
                    devY *= 2;
                    devRot *= 2;
                }
            }

            Logger.recordOutput("Vision/Estimated Pose", estimate.estimatedPose);
            Logger.recordOutput("Vision/Estimated Pose 2d", estimate.estimatedPose.toPose2d());
            Logger.recordOutput("Vision/Rejection Reason", "No rejection");

            addVisionMeasurement(
                estimate.estimatedPose.toPose2d(), 
                estimate.timestampSeconds, 
                VecBuilder.fill(devX, devY, devRot)
            );

        } else {
            Logger.recordOutput("Vision/Rejection Reason", "No measurement present");
        }
    }

    /**
     * Get the yaw to a pose, as a field relative absolute. Results puts the back of the robot towards the pose
     * @param pose - The pose to point at
     * @return the calculated yaw angle
     * @apiNote This is a expensive function, so be careful where you call it
     */
    public Rotation2d getAbsoluteYawToPose(Pose2d target) {
        Pose2d robot = getPose();
        double fieldAngle = Math.atan2(target.getY() - robot.getY(), target.getX() - robot.getX());
        return Rotation2d.fromRadians(fieldAngle).plus(Rotation2d.fromDegrees(Util.isOnBlueAlliance() ? 180 : 0));
    }

    /**
     * Get the yaw of the robot required to shoot a note at a pose, with velocity compensation
     * @param target - the target to aim at
     * @param timeToTarget - the time it takes to reach the target
     * @return - the calculated yaw
     * @apiNote This is a expensive function, so be careful where you call it
     */
    public Rotation2d getVelocityCompensatedYaw(Pose2d target, double timeToTarget) {

        ChassisSpeeds speeds = getFieldRelativeSpeeds();
        Pose2d robot = getPose();
        double delta = Constants.ShooterConstants.kShotCompensationDelta;
        // csys of chassis speed is flipped compared to robot pose on red alliance
        if(!Util.isOnBlueAlliance()) {
            timeToTarget *= -1;
            delta *= -1;
        }

        double fieldAngle = Math.atan2(
            target.getY() - robot.getY() - speeds.vyMetersPerSecond * (delta + timeToTarget),
            target.getX() - robot.getX() - speeds.vxMetersPerSecond * (delta + timeToTarget)
        );
        
        return Rotation2d.fromRadians(fieldAngle).plus(Rotation2d.fromDegrees(Util.isOnBlueAlliance() ? 180 : 0));

    }

    /**
     * Get the driver controller
     * @return the bound driver controller
     */
    public CommandXboxController getDriverController() {
        return mDriverController;
    }

    /**
     * Get the command that brings us to the closest stage
     * @return the command that brings us to the closest stage
     */
    public Command getStageAlignCommand() {
        return new ProxyCommand(() -> new SequentialAutoAlign(0.2, Util.getClosestStagePose(this::getPose)).onlyWhile(this::driverIsNotOverriding));
    }

    /**
     * Enable the path planner rotation override feature, which tracks the speaker
     * @param override - do we want to override?
     */
    public void setPPLibOverrideRotation(boolean override) {
        isOverridingPathFollowingRotation = override;
    }

    /**
     * Get the override rotation for tracking the speaker
     * @return - the Rotation2d
     */
    private Optional<Rotation2d> getOverrideRotation() {

        Logger.recordOutput("Auto/Is Overriding", isOverridingPathFollowingRotation);

        if(isOverridingPathFollowingRotation){
            
            double timeToTarget = 0; //? Why is this 0 if we are trying to shoot on the move??

            if(!Util.isOnBlueAlliance()) {
                timeToTarget *= -1;
            }

            Rotation2d originalRotation = getVelocityCompensatedYaw(Util.getSpeakerPose(), timeToTarget);
            
            if(DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red){
                originalRotation = originalRotation.plus(Rotation2d.fromDegrees(180));
            }
            
            return Optional.of(originalRotation);
        }
        
        return Optional.empty();
    }

    /**
     * Enable / disable coast mode on the swerve
     * @param enableCoast - should we enable?
     */
    public void requestEnableCoastMode(boolean enableCoast) {
        configNeutralMode(enableCoast ? NeutralModeValue.Coast : NeutralModeValue.Brake);
    }

    /**
     * @return If we should flip the path for path planner
     */
    private boolean shouldFlipPath() {
        Optional<Alliance> alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) {
            return alliance.get() == DriverStation.Alliance.Red;
        }
        return false;
    }
  
    /**
     * Handle flipping our perspective for the drivetrain
     */
    private void handleOperatorPerspectiveFlip() {
        /** 
         * Periodically try to apply the operator perspective
         * If we haven't applied the operator perspective before, then we should apply it regardless of DS state
         * This allows us to correct the perspective in case the robot code restarts mid-match
         * Otherwise, only check and apply the operator perspective if the DS is disabled
         * This ensures driving behavior doesn't change until an explicit disable event occurs during testing 
        **/
        if (!mHasAppliedOperatorPerspective || DriverStation.isDisabled()) {
            DriverStation.getAlliance().ifPresent((allianceColor) -> {
                this.setOperatorPerspectiveForward( allianceColor == Alliance.Red ? mRedAlliancePerspectiveRotation : mBlueAlliancePerspectiveRotation );
                mHasAppliedOperatorPerspective = true;
            });
        }
    }

    /**
     * Set up path planner
     */
    public void configAutos() {
        AutoBuilder.configureHolonomic(
            this::getPose,
            this::seedFieldRelative, // ctre defined
            this::getFieldRelativeSpeeds,
            this::driveRobotRelativeSpeeds,
            Constants.AutoConstants.kHolonomicPathFollowerConfig,
            this::shouldFlipPath,
            this
        );

        PPHolonomicDriveController.setRotationTargetOverride(this::getOverrideRotation);
    }

    /**
     * Reset odometry
     */
    public void requestOdometryReset() {
        tareEverything();
        seedFieldRelative(new Pose2d(Util.isOnBlueAlliance() ? 2.7 : 13.7, 4.10, Rotation2d.fromDegrees(Util.isOnBlueAlliance() ? 0 : 180)));
    }
  
    /**
     * Reset odometry to a certain position
     * @param pose - the new position the odometry will read
     */
    public void requestOdometryReset(Pose2d pose){
        tareEverything();
        seedFieldRelative(pose);
    }

    /**
     * Reset the gyro; set the forward direction
     */
    public void requestGyroReset() {
        seedFieldRelative(new Pose2d(getPose().getTranslation(), Util.isOnBlueAlliance() ? Rotation2d.fromDegrees(0) : Rotation2d.fromDegrees(180)));
    }

    /**
     * Enable / disable vision
     * @param enableVision - should we enable vision?
     */
    public void requestVisionOverride(boolean enableVision) {
        if(!mPhotonSetupSuccess && enableVision) {
            System.out.println("[SWERVE] [WARN] [APRILTAG] Requesting Apriltag override, but field setup not successful, ignoring request");
            return;
        }

        mIsPhotonEstimating = enableVision;

        Logger.recordOutput("Vision/Is Photon Estimating", mIsPhotonEstimating);
    }

    /**
     * Are we visioning?
     * @return if vision is enabled
     */
    public boolean getVisionState() {
        return mIsPhotonEstimating;
    }

    /**
     * Apply chassisSpeeds to the drive base
     * @param speeds - the chassisSpeeds to apply
     */
    public void driveRobotRelativeSpeeds(ChassisSpeeds speeds) {
        if(mCurrentState != SwerveState.PATH_FOLLOWING) {
            mCurrentState = SwerveState.PATH_FOLLOWING;
        }
       
        mDesiredRequest = mReqChassisSpeeds.withSpeeds(speeds);
    }

    /**
     * Get the odometry's pose
     * @return - the swerve's Pose2d
     */
    public Pose2d getPose() {
        return m_odometry.getEstimatedPosition();
    }

    /**
     * Get the field relative speeds of the drivebase
     */
    public ChassisSpeeds getFieldRelativeSpeeds() {
        return m_kinematics.toChassisSpeeds(getState().ModuleStates);
    }

    /**
     * Get the robot's velocity
     */
    public double getVelocity() {
        ChassisSpeeds speeds = getFieldRelativeSpeeds();
        return Math.hypot(Math.abs(speeds.vxMetersPerSecond), Math.abs(speeds.vyMetersPerSecond));
    }

    /**
     * Get the built auto chooser
     * @apiNote BuildAutoChooser has to be called before this
     */
    public SendableChooser<Command> getAutoChooser(){
        return mAutoChooser;
    }

    public void buildAutoChooser() {
        mAutoChooser = new SendableChooser<Command>();
        System.out.println("[SWERVE] [INIT] Building auto chooser");
        System.out.println(new File(Filesystem.getDeployDirectory().getPath()+ "/pathplanner/autos/"));
        for(File f : new File(Filesystem.getDeployDirectory().getPath() + "/pathplanner/autos/").listFiles()){
            if(f.isDirectory()) continue;
            mAutoChooser.addOption(f.getName(), AutoBuilder.buildAuto(f.getName().substring(0, f.getName().length()-5)));
        }
        ShuffleboardTab tab = Shuffleboard.getTab("Auto Chooser");
        mAutoChooser.setDefaultOption("6P-Front-Seq", AutoBuilder.buildAuto("6P-Front-Seq"));
        tab.add(mAutoChooser);
        System.out.println("[SWERVE] [INIT] Auto chooser built");
    }

    /**
     * Set the swerve to interpolate shots
     */
    public void setIsInterpolatingShots(boolean isInterpolating) {
        mIsInterpolatingShots = isInterpolating;
    }

    /**
     * Get if we are interpolating shots
     * @return
     */
    public boolean isInterpolatingShots() {
        return mIsInterpolatingShots;
    }

    /**
     * Helper to compute and apply requests, for default driving
     */
    private void passDriveRequestValues(double x, double y, double rotation, boolean slowMode) {
        double calcX = Util.deadBand(-x, Constants.SwerveConstants.kControllerDefaultDeadband) * TunerConstants.kMaxSpeed;
        double calcY = Util.deadBand(-y, Constants.SwerveConstants.kControllerDefaultDeadband) * TunerConstants.kMaxSpeed;
        double calcRot = Util.deadBand(-rotation, Constants.SwerveConstants.kControllerDefaultDeadband) * TunerConstants.kMaxAngularRate;

        if (slowMode) {
            calcX *= 0.25;
            calcY *= 0.25;
            calcRot *= 0.25;
        }

        mReqDrive.withVelocityX(calcX).withVelocityY(calcY).withRotationalRate(calcRot);
    }

    /**
     * Helper to compute and apply requests, for force orient mode
     * @param x
     * @param y
     * @param slowMode
     */
    private void passFaceAngleValues(double x, double y, boolean slowMode) {
        double calcX = Util.deadBand(-x, Constants.SwerveConstants.kControllerDefaultDeadband) * TunerConstants.kMaxSpeed;
        double calcY = Util.deadBand(-y, Constants.SwerveConstants.kControllerDefaultDeadband) * TunerConstants.kMaxSpeed;

        if (slowMode) {
            calcX *= 0.25;
            calcY *= 0.25;
        }

        mReqForceOrient.withVelocityX(calcX).withVelocityY(calcY);
    }

    @Override
    public void simulationPeriodic() {
        /* Assume  */
        updateSimState(0.02, 12);
    }
}