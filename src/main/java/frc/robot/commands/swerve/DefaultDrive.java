package frc.robot.commands.swerve;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.swerve.generated.TunerConstants;


/**
 * @author Grayson A
 */
public class DefaultDrive extends Command {

    private final Swerve swerve = TunerConstants.Swerve;
    private final CommandXboxController driver;

    public DefaultDrive(CommandXboxController driver) {
        this.driver = driver;
        addRequirements(swerve);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        swerve.feedTeleopValues(driver::getLeftX, driver::getLeftY, driver::getRightX, () -> driver.leftBumper().getAsBoolean());
    }
}
