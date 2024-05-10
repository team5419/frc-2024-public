package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.statemachines.IntakeSequenceStateMachine;
import frc.robot.subsystems.statemachines.Superstructure;
import frc.robot.subsystems.statemachines.IntakeSequenceStateMachine.HandoffState;
import frc.robot.subsystems.statemachines.Superstructure.SuperState;

/**
 * @author Grayson A
 */
public class WaitForState extends Command {

    private final Superstructure mSS = Superstructure.getInstance();
    private Runnable request;
    private SuperState goalState;

    public WaitForState(Runnable request, SuperState goalState) {
        this.request = request;
        this.goalState = goalState;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        request.run();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return mSS.getState() == goalState || IntakeSequenceStateMachine.getInstance().getState() == HandoffState.INTAKING;
    }

}