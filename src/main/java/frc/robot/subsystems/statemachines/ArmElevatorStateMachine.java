package frc.robot.subsystems.statemachines;

import frc.robot.Constants;
import frc.robot.SuperstructureGoal;
import frc.robot.lib.DelayedBoolean;
import frc.robot.lib.Util;
import frc.robot.lib.VirtualSubsystem;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.Timer;

public class ArmElevatorStateMachine extends VirtualSubsystem {

    //States for the arm and elevator
    public enum ArmElevatorState {
        MOVING,
        IDLE,
        AT_POS,
        HOMING_ELE,
        HOMING_ARM,
    }

    private static ArmElevatorStateMachine sInstance;
    private ArmElevatorState mState = ArmElevatorState.IDLE;

    private DelayedBoolean mHomingDelay;

    private Arm mArm;
    private Elevator mElevator;

    private double armGoal = SuperstructureGoal.STOW.armRotation;
    private double elevatorGoal = SuperstructureGoal.STOW.elevatorHeight;

    private boolean requestIdle;
    private boolean requestNewPosition;
    private boolean requestStowAndReset;
    private static double armStow = SuperstructureGoal.STOW.armRotation;

    public ArmElevatorStateMachine() {
        mArm = Arm.getInstance();
        mElevator = Elevator.getInstance();
        // Range for the stow angle, as sometimes mechanical damage can affect it
        // Uses a pigeon and constant applied voltage to find the stow angle
        armStow = Util.clamp(mArm.getPosition(), -16, -10);
    }
    
    public static ArmElevatorStateMachine getInstance() {
        if (sInstance == null)
            sInstance = new ArmElevatorStateMachine();
        return sInstance;
    }

    public ArmElevatorState getState() {
        return mState;
    }

    public Elevator getElevator() {
        return mElevator;
    }

    public Arm getArm() {
        return mArm;
    }

    public double getArmStowPosition() {
        return armStow;
    }

    public void zeroAllRequests() {
        requestNewPosition = false;
        requestIdle = false;
        requestStowAndReset = false;
    }

    public void requestIdle() {
        armGoal = SuperstructureGoal.STOW.armRotation;
        elevatorGoal = SuperstructureGoal.STOW.elevatorHeight;
        zeroAllRequests();
        requestIdle = true;
    }

    public void requestStowAndReset() {
        zeroAllRequests();
        requestStowAndReset = true;
    }

    public void requestPosition(double elevatorPos, double armPos) {
        zeroAllRequests();
        // NOTE: while the following is a robust way to prevent collisions, it also prevents us from "cheating" when doing the hook handoff for the trap climb
        // // do not request new position if invalid
        // if(armGoal > Constants.ArmConstants.kArmSafeToMove && elevatorGoal < Constants.ElevatorConstants.kElevatorSafeToMove){
        //     requestNewPosition = false;
        // } else {
        //     requestNewPosition = true;
        //     armGoal = armPos;
        //     elevatorGoal = elevatorPos;
        // }

        requestNewPosition = true;
        armGoal = armPos;
        elevatorGoal = elevatorPos;
    }

    public boolean isAtPositionWithTolerance(double armPos, double armTol, double elePos, double eleTol) {
        return mArm.isAtPositionWithTolerance(armPos, armTol) && mElevator.isAtPositionWithTolerance(elePos, eleTol);
    }

    public boolean isStowed() {
        return isAtPositionWithTolerance(armStow, 
        Constants.ArmConstants.kArmPositionalToleranceDegrees, 
        SuperstructureGoal.STOW.elevatorHeight, 
        Constants.ElevatorConstants.kElevatorPositionalTolerance);
    }

    /**
     * Sets current arm position to the zero position
     */
    public void resetPositionUsingPigeon() {
        mArm.resetPositionUsingPigeon();
    }

    public void periodic() {
        Logger.recordOutput("Arm Elevator State/State", mState);
        Logger.recordOutput("Arm Elevator State/Elevator Goal", elevatorGoal);
        Logger.recordOutput("Arm Elevator State/Arm Goal", armGoal);
        Logger.recordOutput("Arm Elevator State/Is requesting idle", requestIdle);
        Logger.recordOutput("Arm Elevator State/Is requesting stow and reset", requestStowAndReset);
        Logger.recordOutput("Arm Elevator State/Is requesting new position", requestNewPosition);
        Logger.recordOutput("Arm Elevator State/Arm at stow", armStow);
        Logger.recordOutput("Arm Elevator State/Stowed Boolean", isStowed());

        ArmElevatorState nState = mState;

        if (mState == ArmElevatorState.IDLE) {
            // Actions
            mArm.setPower(0);
            mElevator.setPower(0);

            // Transitions
            if (requestNewPosition) {
                nState = ArmElevatorState.MOVING;
                zeroAllRequests();
            }
        } else if (mState == ArmElevatorState.MOVING) {
            // Actions
            // If the arm or elevator is unsafe to move without hitting a hardstop, move the opposite subsystem first
            if (mArm.getPosition() < Constants.ArmConstants.kArmSafeToMove
                    && mElevator.getPosition() < Constants.ElevatorConstants.kElevatorSafeToMove
                    && armGoal > Constants.ArmConstants.kArmSafeToMove) {

                mElevator.setGoal(elevatorGoal);
                mArm.setGoal(Constants.ArmConstants.kArmSafeToMove);
                            
            } else if (elevatorGoal < Constants.ElevatorConstants.kElevatorSafeToMove
                    && mElevator.getPosition() > Constants.ElevatorConstants.kElevatorSafeToMove
                    && mArm.getPosition() > Constants.ArmConstants.kArmSafeToMove
                    && armGoal < Constants.ArmConstants.kArmSafeToMove) {

                mElevator.setGoal(Constants.ElevatorConstants.kElevatorSafeToMove);
                mArm.setGoal(armGoal);
                
            } else {
                mElevator.setGoal(elevatorGoal);
                mArm.setGoal(armGoal);
            }

            // Transitions
            if (mArm.atPosition() && mElevator.atPosition()) {
                nState = ArmElevatorState.AT_POS;
                zeroAllRequests();
            } else if (requestIdle) {
                nState = ArmElevatorState.IDLE;
                // zeroAllRequests();
            } else if (requestStowAndReset) {
                nState = ArmElevatorState.HOMING_ELE;
            }
        } else if (mState == ArmElevatorState.AT_POS) {
            // Transitions
            if (requestNewPosition) {
                nState = ArmElevatorState.MOVING;
                zeroAllRequests();
            } else if (requestIdle) {
                nState = ArmElevatorState.IDLE;
                zeroAllRequests();
            } else if (isStowed()){
                nState = ArmElevatorState.IDLE;
            } else if (requestStowAndReset) {
                nState = ArmElevatorState.HOMING_ELE;
            }
        } else if (mState == ArmElevatorState.HOMING_ELE) {
            // Actions
            if(mHomingDelay == null) mHomingDelay = new DelayedBoolean(Timer.getFPGATimestamp(), 0.08);

            mElevator.setPower(-0.3);

            // Transitions
            // If the elevator has been moving down into a hardstop for some amount of time, go to HOMING_ARM
            if (mHomingDelay.update(Timer.getFPGATimestamp(), Math.abs(mElevator.getVelocity()) < 0.02)) {
                nState = ArmElevatorState.HOMING_ARM;
                mElevator.resetPosition() ;
                mElevator.setGoal(SuperstructureGoal.STOW.elevatorHeight);
                mArm.resetPositionUsingPigeon();
                mHomingDelay = null;
            }

            if(requestIdle) {
                nState = ArmElevatorState.IDLE;
                mHomingDelay = null;
            }

            zeroAllRequests();
        } else if (mState == ArmElevatorState.HOMING_ARM) {
            // Actions
            if (mHomingDelay == null) mHomingDelay = new DelayedBoolean(Timer.getFPGATimestamp(), 0.08);
            mArm.setPower(-0.05);
            
            // Transitions
            // If the arm is below a certain position, and not moving (hitting a hardstop), go to IDLE
            if (mHomingDelay.update(Timer.getFPGATimestamp(), Math.abs(mArm.getVelocity()) < 5)) {
                // NOTE: put clamp here if necessary
                armStow = mArm.getPosition();
                mArm.stop();

                nState = ArmElevatorState.IDLE;
                mHomingDelay = null;
            }

            if (requestIdle) {
                nState = ArmElevatorState.IDLE;
                mHomingDelay = null;
            }

        }

        mState = nState;
    }

    @Override
    public void simulationPeriodic() {
    }
}