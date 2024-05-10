package frc.robot.commands.swerve;

import org.photonvision.PhotonUtils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.swerve.generated.TunerConstants;

/**
 * @author Grayson A
 */
public class SequentialAutoAlign extends SequentialCommandGroup {

    public SequentialAutoAlign(double switchDistanceMeters, Pose2d targetPose) {

        // Get our swerve
        Swerve mSwerve = TunerConstants.Swerve;

        // Require swerve
        addRequirements(mSwerve);

        // Only do this while the driver isn't trying to do anything
        // onlyWhile(mSwerve::driverIsNotOverriding);

        // Run our stuff one after another
        addCommands(

            // Run them at the same time, until the deadline ends, where then we cancel all
            new ParallelDeadlineGroup(

                // Our condition to end
                Commands.waitUntil(() -> PhotonUtils.getDistanceToPose(mSwerve.getPose(), targetPose) < switchDistanceMeters), 

                // What were doing in the meantime
                mSwerve.goToPointCommand(targetPose, 0, false) 
            ),

            // After we are close enough, then use raw PID
            new PIDAutoAlign(() -> targetPose, false)
        );
    }
}
