package frc.robot.commands.swerve;

import java.util.function.Supplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.lib.Util;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.swerve.generated.TunerConstants;

/**
 * @author Grayson
 **/
public class PIDAutoAlign extends Command {

    private final Swerve mSwerve = TunerConstants.Swerve;
    private final Supplier<Pose2d> mTargetPose;

    private final PIDController mXController, mYController; // left right

    private boolean mIsDynamic = false;

    public PIDAutoAlign(Supplier<Pose2d> targetPose, boolean isDynamic) {
        mTargetPose = targetPose;
        mIsDynamic = isDynamic;

        mXController = new PIDController(Constants.SwerveConstants.kAutoAlignXPID.kP, Constants.SwerveConstants.kAutoAlignXPID.kI, Constants.SwerveConstants.kAutoAlignXPID.kD); // forward back (m)
        mYController = new PIDController(Constants.SwerveConstants.kAutoAlignYPID.kP, Constants.SwerveConstants.kAutoAlignYPID.kI, Constants.SwerveConstants.kAutoAlignYPID.kD); // left right (m)

        mXController.setTolerance(Constants.SwerveConstants.kAutoAlignPositionalTolerance); 
        mYController.setTolerance(Constants.SwerveConstants.kAutoAlignPositionalTolerance);

        addRequirements(mSwerve);
        // this.onlyWhile(mSwerve::driverIsNotOverriding);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        mXController.reset(); 
        mYController.reset();

        mXController.setSetpoint(mTargetPose.get().getY()); 
        mYController.setSetpoint(mTargetPose.get().getX()); 
        mSwerve.setForceOrientSetpoint(mTargetPose.get().getRotation());

        mXController.setIZone(1);
        mYController.setIZone(1);

    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {

        if(mIsDynamic) {
            
            mXController.setSetpoint(mTargetPose.get().getY());
            mYController.setSetpoint(mTargetPose.get().getX());
            mSwerve.setForceOrientSetpoint(mTargetPose.get().getRotation());
            
        }

        double dx = mXController.calculate(mSwerve.getPose().getY()); 
        double dy = mYController.calculate(mSwerve.getPose().getX());

        // Logger.recordOutput("dy", dy);
        // Logger.recordOutput("dx", dx);
        // Logger.recordOutput("Error y", mYController.getPositionError());
        // Logger.recordOutput("Error x", mXController.getPositionError());

        mSwerve.feedTeleopValues(dx, dy);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        mSwerve.requestLock();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        // return false;
        return mXController.atSetpoint() && mYController.atSetpoint() && // In the right translation
            Util.epsilonEquals(mSwerve.getPose().getRotation().getDegrees(), mTargetPose.get().getRotation().getDegrees(), Constants.SwerveConstants.kAutoAlignRotationalTolerance) // in the right rotation
            && Math.abs(mSwerve.getVelocity()) < Constants.SwerveConstants.kAutoAlignMaxEndingVelocity; // going slow enough, so we don't skid
    }
}