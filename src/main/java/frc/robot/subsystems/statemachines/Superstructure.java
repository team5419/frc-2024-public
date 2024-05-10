package frc.robot.subsystems.statemachines;

import org.littletonrobotics.junction.Logger;
import org.photonvision.PhotonUtils;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import org.littletonrobotics.junction.Logger;
import org.photonvision.PhotonUtils;

import frc.robot.Constants;
import frc.robot.SuperstructureGoal;

import frc.robot.lib.DelayedBoolean;
import frc.robot.lib.RumbleThread;
import frc.robot.lib.RumbleThread.ControllersToRumble;
import frc.robot.lib.Util;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.misc.ShotAngleInterpolator;
import frc.robot.subsystems.misc.ShotAngleInterpolator.InterpolatedSuperstructureState;
import frc.robot.subsystems.statemachines.ArmElevatorStateMachine.ArmElevatorState;
import frc.robot.subsystems.statemachines.IntakeSequenceStateMachine.HandoffState;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.swerve.generated.TunerConstants;

public class Superstructure extends SubsystemBase {

    public enum SuperState {
        IDLE,
        MOVING_TO_POS,
        INTAKING,
        SHOOTING,
        EJECT_AND_RESET,
        REACHING_FOR_CHAIN,
        CLIMBING,
        REACHING_FOR_TRAP,
        TRAP_SHOOTING,
        TRAP_RETRACTING,
        STOW_RESET,
        INTERPOLATING,
        CLEANUP
    }
    public enum IncomingRequest{
        REQ_NONE,
        REQ_IDLE,
        REQ_NEWPOS,
        REQ_INTAKE,
        REQ_SHOOT,
        REQ_AUTOAIM,
        REQ_CLEANUP,
        REQ_EJECT_RESET,
        REQ_STOW_RESET,
        REQ_CLIMB,
        REQ_CLIMB_ADVANCE,
        REQ_TRAP_RETRACT
    }

    private static Superstructure sInstance = null;
    private SuperState currentState = SuperState.IDLE;
    private SuperState nextState = SuperState.IDLE;
    private IncomingRequest nextRequest = IncomingRequest.REQ_NONE;

    private ArmElevatorStateMachine mAAndEState;
    private IntakeSequenceStateMachine mHandoffState;
    private Shooter mShooter;

    private boolean requestManualIntake;
    private boolean hasRumbledForFloor;
    private boolean wasInterpolating;
    private boolean intakeFWD;

    private double mArmGoal;
    private double mEleGoal;
    private double mShootSpeed;
    private double distanceFromPos;
    // variables to hold the requested positions before the state properly transitions
    private double newDesiredArmPos;
    private double newDesiredElePos;
    private double newDesiredShootSpeed;

    private DelayedBoolean mShotFinishedDelay, mClimbSafeDelay;

    private Swerve mSwerve = TunerConstants.Swerve;
    
    private InterpolatedSuperstructureState state;

    public Superstructure() {
        mAAndEState = ArmElevatorStateMachine.getInstance();
        mHandoffState = IntakeSequenceStateMachine.getInstance();
        mShooter = Shooter.getInstance();
        zeroAllRequests();
    }

    public static Superstructure getInstance() {
        if (sInstance == null)
            sInstance = new Superstructure();
        return sInstance;
    }

    //////////////////////////////// MANAGE REQUESTS ////////////////////////////////
    /////////////////////////////////////////////////////////////////////////////////

    public void zeroAllRequests() {
        nextRequest = IncomingRequest.REQ_NONE;
    }

    public void requestIdle() {
        nextRequest = IncomingRequest.REQ_IDLE;
    }

    public void requestNewPos(double eleGoal, double armGoal, double shootSpeed) {
        newDesiredElePos = Util.clamp(eleGoal, 0, Constants.ElevatorConstants.kEleTopPosition);
        newDesiredArmPos = Util.clamp(armGoal, mAAndEState.getArmStowPosition(), Constants.ArmConstants.kTopSafe);
        newDesiredShootSpeed = shootSpeed;
        nextRequest = IncomingRequest.REQ_NEWPOS;
    }

    public void requestNewPos(SuperstructureGoal goal) {
        requestNewPos(goal.elevatorHeight, goal.armRotation, goal.shooterSpeed);
    }

    public void requestStowReset() {
        nextRequest = IncomingRequest.REQ_STOW_RESET;
    }

    public void requestElePos(double goal) {
        requestNewPos(goal, mArmGoal, mShootSpeed);
    }

    public void requestArmPos(double goal) {
        requestNewPos(mEleGoal, goal, mShootSpeed);
    }

    public void requestSubwoofer() {
        requestNewPos(SuperstructureGoal.SUBWOOFER.elevatorHeight, mAAndEState.getArmStowPosition(), SuperstructureGoal.SUBWOOFER.shooterSpeed);
    }

    public void requestStockpile() {
        requestNewPos(SuperstructureGoal.STOCKPILE.elevatorHeight, mAAndEState.getArmStowPosition(), SuperstructureGoal.STOCKPILE.shooterSpeed);
    }

    public void requestAmp() {
        requestNewPos(SuperstructureGoal.AMP);
    }

    public void requestPodiumDunk() {
        requestNewPos(SuperstructureGoal.PODIUM_DUNK);
    }

    public void requestTrap() {
        requestNewPos(SuperstructureGoal.TRAP);
    }

    public void requestStow() {
        requestNewPos(SuperstructureGoal.STOW.elevatorHeight, mAAndEState.getArmStowPosition(), SuperstructureGoal.STOW.shooterSpeed);
    }

    // We sometimes set the arm position a little bit above the stow position to save time moving the arm during auto
    public void requestIntake(double armPos) {
        newDesiredArmPos = armPos;
        newDesiredElePos = SuperstructureGoal.STOW.elevatorHeight;
        newDesiredShootSpeed = SuperstructureGoal.SUBWOOFER.shooterSpeed;
        nextRequest = IncomingRequest.REQ_INTAKE;
    }

    public void requestIntake() {
        requestIntake(mAAndEState.getArmStowPosition());
    }

    public void requestShoot() {
        nextRequest = IncomingRequest.REQ_SHOOT;
    }

    // Auto-Aim
    public void requestInterpolating() {
        nextRequest = IncomingRequest.REQ_AUTOAIM;
    }

    public void toggleCleanup() {
        nextRequest = IncomingRequest.REQ_CLEANUP;
    }

    public void requestEjectAndReset() {
        nextRequest = IncomingRequest.REQ_EJECT_RESET;
    }

    public void requestManualIntake(boolean intakingFWD) {
        requestManualIntake = true;
        intakeFWD = intakingFWD;
    }

    public void requestManualIntakeOff() {
        requestManualIntake = false;
    }

    // Climbs onto chain
    public void requestClimbAndTrap() {
        nextRequest = IncomingRequest.REQ_CLIMB;
    }

    public void requestClimbAdvance() {
        nextRequest = IncomingRequest.REQ_CLIMB_ADVANCE;
    }

    // Retracts after trapping
    public void requestTrapRetract() {
        nextRequest = IncomingRequest.REQ_TRAP_RETRACT;
    }

    //////////////////////////////// UTIL FUNCTIONS /////////////////////////////////
    /////////////////////////////////////////////////////////////////////////////////

    public SuperState getState() {
        return currentState;
    }

    public double getElePos() {
        return mAAndEState.getElevator().getPosition();
    }

    public double getArmPos() {
        return mAAndEState.getArm().getPosition();
    }

    // private void setRumble(double amount) {
    //     mSwerve.getDriverController().getHID().setRumble(RumbleType.kBothRumble, amount);
    // };

    ///////////////////////////////// MATH FUNCTIONS ////////////////////////////////
    /////////////////////////////////////////////////////////////////////////////////

    // after trap shooting, find the arm angle that allows the arm to fold back 
    // while the elevator comes down without breaking extension limits
    private double findAngleFromElevator(double elePos) {
        double h = 18.75; // in -- ground to pivot height
        double zC = 46.5; // in -- max height to avoid breaking extension limits
        double l = 23.5; // in -- length from pivot to end of indexer

        return Util.clamp(Units.radiansToDegrees(Math.asin((zC - h - Units.metersToInches(elePos)) / l)) + 3, 0, 80);
    }

    // calculate the velocity-compensated shooter speed & angle and desired robot pose
    private void runInterpolation() {

        distanceFromPos = PhotonUtils.getDistanceToPose(mSwerve.getPose(), Util.getSpeakerPose());
        double timeToTarget = Util.getTimeToTarget(distanceFromPos);
        double dt = 0.4;

        Logger.recordOutput("Swerve/Time To Target", timeToTarget);

        // Handle swerve turn ----------------------------------------------------------------------------------------------
        
        if(!DriverStation.isAutonomous()) {
            mSwerve.setForceOrientSetpoint(mSwerve.getVelocityCompensatedYaw(Util.getSpeakerPose(), timeToTarget));
        }

        // Handle shot interpolation ----------------------------------------------------------------------------------------------
        state = ShotAngleInterpolator.getInstance().getVelocityCompensatedSuperstructure(dt);

        mShootSpeed = state.interpolatedShooterSpeed();
        mArmGoal = state.interpolatedArmAngle();
        mEleGoal = state.interpolatedElevatorHeight();

        Logger.recordOutput("ShotAngleInterpolator/Arm Setpoint Interpolated", state.interpolatedArmAngle());
        Logger.recordOutput("ShotAngleInterpolator/Elevator Setpoint Interpolated", state.interpolatedElevatorHeight());
        Logger.recordOutput("ShotAngleInterpolator/Shooter Setpoint Interpolated", state.interpolatedShooterSpeed());
    }

    public boolean interpolationReady() {

        double tolerance = Util.clamp(6 - (PhotonUtils.getDistanceToPose(mSwerve.getPose(), Util.getSpeakerPose())), 0.4, 2);
        boolean armReady = mAAndEState.getArm().isAtPositionWithTolerance(mArmGoal, tolerance);
        boolean shooterReady = mShooter.isAtOrAboveVelocityWithTolerance(mShootSpeed, Constants.ShooterConstants.kVelocityShootToleranceRPM);
        boolean yawReady = mSwerve.forceOrientAtSetpoint();

        Logger.recordOutput("Interpolation/Calculated Tolerance", tolerance);
        Logger.recordOutput("Interpolation/ArmReady", armReady);
        Logger.recordOutput("Interpolation/ShooterReady", shooterReady);

        return armReady && shooterReady && yawReady;

    }

    /////////////////////////////// HANDLE TRANSITIONS //////////////////////////////
    /////////////////////////////////////////////////////////////////////////////////

    // manual intake is a sub-state that can be called from most superstates in case a beam break fails
    public void allowManualIntake() {
        if (requestManualIntake) {
            mHandoffState.requestManualIntake(intakeFWD);
        } else if (mHandoffState.getState().equals(HandoffState.MANUAL_INTAKE)) {
            mHandoffState.requestIdle();
        }
    }

    // generally always allow the co-driver to exit any state in case something funky happens
    public SuperState allowFaultTransitions() {
        if(nextRequest == IncomingRequest.REQ_IDLE) {
            return SuperState.IDLE;
        } else if (nextRequest == IncomingRequest.REQ_STOW_RESET) {
            return enterStowResetState();                              
        } else if (nextRequest == IncomingRequest.REQ_EJECT_RESET) {
            return SuperState.EJECT_AND_RESET;
        } else {
            return currentState;
        }
    }

    // when transitioning into a new position state, update the goal to match the input desired state
    private void updateGoalPos() {
        mEleGoal = newDesiredElePos;
        mArmGoal = newDesiredArmPos;
        mShootSpeed = newDesiredShootSpeed;

        mAAndEState.requestPosition(mEleGoal, mArmGoal);
        mShooter.setVelocity(mShootSpeed);
    }

    private void updateGoalPos(SuperstructureGoal goal) {
        mEleGoal = goal.elevatorHeight;
        mArmGoal = goal.armRotation;
        mShootSpeed = goal.shooterSpeed;

        mAAndEState.requestPosition(mEleGoal, mArmGoal);
        mShooter.setVelocity(mShootSpeed);
    }

    private SuperState enterNewPosState() {
        // zeroAllRequests();
        updateGoalPos();
        nextRequest = IncomingRequest.REQ_NONE;
        return SuperState.MOVING_TO_POS;
    }
    
    private SuperState enterStowResetState() {
        mAAndEState.resetPositionUsingPigeon();
        updateGoalPos(SuperstructureGoal.RESET);
        mHandoffState.requestIdle();
        
        // zeroAllRequests();
        nextRequest = IncomingRequest.REQ_NONE;
        return SuperState.STOW_RESET;
    }

    private SuperState enterCleanupState() {
        // if the handoff state machine has not yet been initialized, start it
        if (mHandoffState.getState() == HandoffState.IDLE) {
            mHandoffState.requestIntake();
        }

        // increase arm max accel for tracking
        mAAndEState.getArm().setAggressiveMotionMagicConfigs();
        
        // zeroAllRequests();
        nextRequest = IncomingRequest.REQ_NONE;
        return SuperState.CLEANUP;
    }

    ////////////////////////////////// STATE MACHINE ////////////////////////////////
    /////////////////////////////////////////////////////////////////////////////////

    private SuperState handleIdle() {

        // Actions
        mAAndEState.requestIdle();
        mHandoffState.requestIdle();
        mShooter.setVelocity(0);

        // Transitions
        allowManualIntake();

        if (nextRequest == IncomingRequest.REQ_NEWPOS) {
            return enterNewPosState();
        } else if (nextRequest == IncomingRequest.REQ_CLIMB) {
            return SuperState.REACHING_FOR_CHAIN;
        } else if (nextRequest == IncomingRequest.REQ_INTAKE) {
            updateGoalPos();
            return SuperState.INTAKING;
        } else if (nextRequest == IncomingRequest.REQ_AUTOAIM && mHandoffState.isSafeToMoveArmEle()) {
            mAAndEState.resetPositionUsingPigeon();
            mAAndEState.getArm().setAggressiveMotionMagicConfigs(); // increase arm max accel for tracking
            return SuperState.INTERPOLATING;
        } else if (nextRequest == IncomingRequest.REQ_CLEANUP) {
            mAAndEState.resetPositionUsingPigeon();
            return enterCleanupState();
        }

        return allowFaultTransitions();

    }

    private SuperState handleMoving() {

        // Actions
        mShooter.setVelocity(mShootSpeed);

        // Transitions
        allowManualIntake();

        if (nextRequest == IncomingRequest.REQ_SHOOT && mHandoffState.isSafeToMoveArmEle()) {
            return SuperState.SHOOTING;
        } else if (nextRequest == IncomingRequest.REQ_CLIMB) {
            return SuperState.REACHING_FOR_CHAIN;
        } else if (nextRequest == IncomingRequest.REQ_INTAKE) {
            updateGoalPos();
            return SuperState.INTAKING;
        } else if (nextRequest == IncomingRequest.REQ_NEWPOS) {
            return enterNewPosState();
        } else if (nextRequest == IncomingRequest.REQ_AUTOAIM && mHandoffState.isSafeToMoveArmEle()) {
            // mAAndEState.resetPositionUsingPigeon();
            mAAndEState.getArm().setAggressiveMotionMagicConfigs(); // increase arm max accel for tracking
            return SuperState.INTERPOLATING;
        } else if (nextRequest == IncomingRequest.REQ_CLEANUP) {
            return enterCleanupState();
        }

        return allowFaultTransitions();
    }

    private SuperState handleIntaking() {

        // Actions
        mShooter.setVelocity(SuperstructureGoal.SUBWOOFER.shooterSpeed);

        // since this state always requests intake, it will still return to the automated intake state machine when manual intake is released
        mHandoffState.requestIntake();
        allowManualIntake();

        if(mHandoffState.noteInFloor() && !hasRumbledForFloor){
            RumbleThread.getInstance().setRumble(RumbleType.kBothRumble, 0.2, 0.75, ControllersToRumble.DRIVER);
            hasRumbledForFloor = true;
        }

        // Transitions
        if (nextRequest == IncomingRequest.REQ_IDLE) {
            return SuperState.IDLE;
        } else if (nextRequest == IncomingRequest.REQ_SHOOT && mHandoffState.isSafeToMoveArmEle()) {
            mAAndEState.resetPositionUsingPigeon();
            return SuperState.SHOOTING;
        } else if (nextRequest == IncomingRequest.REQ_NEWPOS && mHandoffState.isSafeToMoveArmEle()) {
            mAAndEState.resetPositionUsingPigeon();
            return enterNewPosState();
        } else if (nextRequest == IncomingRequest.REQ_AUTOAIM && mHandoffState.isSafeToMoveArmEle()) {
            mAAndEState.resetPositionUsingPigeon();
            mAAndEState.getArm().setAggressiveMotionMagicConfigs(); // increase arm max accel for tracking
            return SuperState.INTERPOLATING;
        } else if (nextRequest == IncomingRequest.REQ_CLIMB) {
            mAAndEState.resetPositionUsingPigeon();
            return SuperState.REACHING_FOR_CHAIN;
        } else if (nextRequest == IncomingRequest.REQ_EJECT_RESET) { 
            return SuperState.EJECT_AND_RESET;
        } else if (nextRequest == IncomingRequest.REQ_CLEANUP) {
            mAAndEState.resetPositionUsingPigeon();
            return enterCleanupState();
        }

        return SuperState.INTAKING;
    }

    private SuperState handleInterpolating() {

        // Actions
        runInterpolation();
        mAAndEState.requestPosition(mEleGoal, mArmGoal);
        mShooter.setVelocity(mShootSpeed);

        allowManualIntake();

        if (interpolationReady()) {
            RumbleThread.getInstance().setRumble(RumbleType.kBothRumble, 1, 0.75, ControllersToRumble.DRIVER);
            wasInterpolating = true;
            mHandoffState.requestShoot(); // not strictly necessary but gets the request out one loop earlier
            return SuperState.SHOOTING;
        }

        if (nextRequest == IncomingRequest.REQ_NEWPOS) {
            mAAndEState.getArm().setDefaultMotionMagicConfigs(); // return motion magic to normal
            return enterNewPosState();
        } else if (nextRequest == IncomingRequest.REQ_STOW_RESET || nextRequest == IncomingRequest.REQ_IDLE || nextRequest == IncomingRequest.REQ_EJECT_RESET) {
            mAAndEState.getArm().setDefaultMotionMagicConfigs(); // return motion magic to normal
            return allowFaultTransitions();
        }

        return SuperState.INTERPOLATING;
    }

    private SuperState handleShooting() {
        // continue to track the target until the note is out of the robot
        if (wasInterpolating) {
            runInterpolation();
            mAAndEState.requestPosition(mEleGoal, mArmGoal);
            mShooter.setVelocity(mShootSpeed);
        }

        if (mShotFinishedDelay == null) mShotFinishedDelay = new DelayedBoolean(Timer.getFPGATimestamp(), 0.15);

        if ((mAAndEState.getState() == ArmElevatorState.AT_POS || mAAndEState.getState() == ArmElevatorState.IDLE) && mShooter.isAtOrAboveVelocityWithTolerance(mShootSpeed, Constants.ShooterConstants.kVelocityShootToleranceRPM)) {
            mHandoffState.requestShoot();
        }

        // Transitions
        if (mShotFinishedDelay.update(Timer.getFPGATimestamp(), !mHandoffState.noteInShooter())) {

            wasInterpolating = false;
            mShotFinishedDelay = null;
            mHandoffState.requestIdle();
            mAAndEState.getArm().setDefaultMotionMagicConfigs(); // return arm motion magic settings back to normal

            if (nextRequest == IncomingRequest.REQ_INTAKE) {
                
                mHandoffState.requestIntake();
                zeroAllRequests();
                updateGoalPos();
                return SuperState.INTAKING;

            } else if (nextRequest == IncomingRequest.REQ_CLEANUP) {
                // the next line immediately changes the handoff state to INTAKING because otherwise it will miss the transition
                mHandoffState.requestIntake();
                return enterCleanupState();

            } else if (mAAndEState.isStowed()) {
                return SuperState.IDLE;
            } else {
                return enterStowResetState();
            }

        } else if (nextRequest == IncomingRequest.REQ_IDLE) {
            wasInterpolating = false;
            return SuperState.IDLE;
        }

        return SuperState.SHOOTING;
    }

    private SuperState handleCleanup() {

        //! NOTE: this state will not work if any beam breaks fail
        // Goal: Cycle continuously between intaking & shooting while aim is locked on speaker

        // Allow for toggle of the cleanup state
        if(nextRequest == IncomingRequest.REQ_CLEANUP) {
            return enterStowResetState();
        }

        // Actions
        runInterpolation();
        mShooter.setVelocity(mShootSpeed); // always track flywheel speed

        // track the interpolated arm setpoint as long as it does not go out of handoff range 
        if (mHandoffState.getState() == HandoffState.ALIGNING_IN_INDEXER || mHandoffState.getState() == HandoffState.NOTE_READY) { 

            // interpolate the arm position without clamping the arm angle
            mAAndEState.requestPosition(mEleGoal, mArmGoal);

            // shoot the note when ready & aim is correct
            if ((distanceFromPos < 4) && interpolationReady()) {
                RumbleThread.getInstance().setRumble(RumbleType.kBothRumble, 0.5, 0.75, ControllersToRumble.DRIVER);
                mHandoffState.requestShoot();
                if (nextRequest == IncomingRequest.REQ_SHOOT) { // can use requestShoot to end the sequence -> stow & reset -> idle
                    wasInterpolating = true;
                    return SuperState.SHOOTING;
                }
            }

        } else if (mHandoffState.getState() == HandoffState.INTAKING || mHandoffState.getState() == HandoffState.PASSING_TO_INDEXER || mHandoffState.getState() == HandoffState.NOTE_ENTERED_INDEXER) {
            // interpolate the arm position but clamp the arm angle
            mArmGoal = Util.clamp(mArmGoal, mAAndEState.getArmStowPosition(), 0);
            mAAndEState.requestPosition(mEleGoal, mArmGoal);

        // return back to intaking when the shot is done
        } else if (mHandoffState.getState() == HandoffState.SHOOTING) {
            
            if (mShotFinishedDelay == null) mShotFinishedDelay = new DelayedBoolean(Timer.getFPGATimestamp(), 0.15);

            if (mShotFinishedDelay.update(Timer.getFPGATimestamp(), !mHandoffState.noteInShooter())) {
                mHandoffState.requestIntake();
                mShotFinishedDelay = null;
            }

        }

        // Transitions
        if (nextRequest == IncomingRequest.REQ_IDLE || nextRequest == IncomingRequest.REQ_STOW_RESET || nextRequest == IncomingRequest.REQ_EJECT_RESET) {
            // return arm motion magic to normal
            mAAndEState.getArm().setDefaultMotionMagicConfigs();
            return allowFaultTransitions(); 
        }
        

        if (nextRequest == IncomingRequest.REQ_INTAKE) {
            updateGoalPos();
            mAAndEState.getArm().setDefaultMotionMagicConfigs();
            return SuperState.INTAKING;
        } else if (nextRequest == IncomingRequest.REQ_NEWPOS) { 
            // robot may need to amp out of or between speaker cleanups
            mAAndEState.getArm().setDefaultMotionMagicConfigs();
            return enterNewPosState();
        } else if (nextRequest == IncomingRequest.REQ_AUTOAIM) { 
            // could replace this transition with requestShoot to end the sequence 
            return SuperState.INTERPOLATING;
        }

        return SuperState.CLEANUP;
    }

    public SuperState handleStowReset() {

        // transition into stow_reset will have already given the arm/ele a goal of the RESET position
           
        // Transitions
        if(nextRequest == IncomingRequest.REQ_IDLE) {
            currentState = SuperState.IDLE;
        } else if(nextRequest == IncomingRequest.REQ_INTAKE && (mAAndEState.getState() == ArmElevatorState.MOVING)) {
            // allow quick transition to INTAKING if starting to stow but before homing
            zeroAllRequests();
            mAAndEState.requestPosition(mEleGoal, mArmGoal);
            return SuperState.INTAKING;
        } else if (mAAndEState.isAtPositionWithTolerance(SuperstructureGoal.RESET.armRotation, Constants.ArmConstants.kArmPositionalToleranceDegrees,
                SuperstructureGoal.RESET.elevatorHeight, Constants.ElevatorConstants.kElevatorPositionalTolerance)) {
            mAAndEState.requestStowAndReset();
        } else if (mAAndEState.getState() == ArmElevatorState.IDLE) {
            mArmGoal = mAAndEState.getArmStowPosition();
            mEleGoal = SuperstructureGoal.STOW.elevatorHeight;
            return SuperState.IDLE;
        }

        return SuperState.STOW_RESET;
    }

    private SuperState handleEjectReset() {

        // Actions
        mAAndEState.requestPosition(SuperstructureGoal.STOW.elevatorHeight, mAAndEState.getArmStowPosition());
        mHandoffState.requestEjectAndReset();
        mShooter.setVelocity(700);

        // Transitions
        if (nextRequest == IncomingRequest.REQ_STOW_RESET) {
            return mAAndEState.isStowed() ? SuperState.IDLE : enterStowResetState();
        } else if (nextRequest == IncomingRequest.REQ_NEWPOS) {
            mHandoffState.requestIdle();
            return SuperState.MOVING_TO_POS;
        } else if (nextRequest == IncomingRequest.REQ_CLEANUP) {
            return enterCleanupState();
        }

        return SuperState.EJECT_AND_RESET;
    }

    private SuperState handleReachingForChain() {

        // Actions
        updateGoalPos(SuperstructureGoal.TRAP);
        mShooter.setPower(0);

        // Transitions
        allowManualIntake();

        if (nextRequest == IncomingRequest.REQ_CLIMB_ADVANCE && mAAndEState.getState() == ArmElevatorState.AT_POS) {
            nextRequest = IncomingRequest.REQ_NONE;
            return SuperState.CLIMBING;
        } else if (nextRequest == IncomingRequest.REQ_IDLE) {
            return SuperState.IDLE;
        } else if (nextRequest == IncomingRequest.REQ_NEWPOS) {
            return enterNewPosState();
        } else if (nextRequest == IncomingRequest.REQ_STOW_RESET) {
            return enterStowResetState();           
        }

        return SuperState.REACHING_FOR_CHAIN;
    }

    private SuperState handleClimbing() {

        if (mClimbSafeDelay == null) mClimbSafeDelay = new DelayedBoolean(Timer.getFPGATimestamp(), 0.25);

        // Actions
        mEleGoal = 0.015;
        mAAndEState.requestPosition(mEleGoal, mArmGoal);

        // Transitions
        allowManualIntake();

        if (mAAndEState.getState() == ArmElevatorState.AT_POS || mClimbSafeDelay.update(Timer.getFPGATimestamp(), mAAndEState.getElevator().getPosition() < 0.045)) {
            mClimbSafeDelay = null;
            return SuperState.REACHING_FOR_TRAP;
        } else if (nextRequest == IncomingRequest.REQ_IDLE) {
            mClimbSafeDelay = null;
            return SuperState.IDLE;
        }
    
        return SuperState.CLIMBING;
    }

    private SuperState handleReachingForTrap() {
         // Actions
        updateGoalPos(SuperstructureGoal.TRAP);
        mShooter.setPower(0);

        // Transitions
        if (mAAndEState.getState() == ArmElevatorState.AT_POS && nextRequest == IncomingRequest.REQ_CLIMB_ADVANCE) {
            nextRequest = IncomingRequest.REQ_NONE;
            return SuperState.TRAP_SHOOTING;
        } else if (nextRequest == IncomingRequest.REQ_IDLE) {
            return SuperState.IDLE;
        }

        return SuperState.REACHING_FOR_TRAP;
    }

    private SuperState handleTrapShooting() {

        // Actions
        mHandoffState.requestTrap();

        allowManualIntake();

        // Transitions
        if (nextRequest == IncomingRequest.REQ_IDLE) {
            return SuperState.IDLE;
        } else if (nextRequest == IncomingRequest.REQ_TRAP_RETRACT || nextRequest == IncomingRequest.REQ_CLIMB_ADVANCE) {
            return SuperState.TRAP_RETRACTING;
        }

        return SuperState.TRAP_SHOOTING;
    }

    public SuperState handleTrapRetracting() {

        // Actions
        mEleGoal = Util.clamp(mEleGoal - .002, .15, SuperstructureGoal.TRAP.elevatorHeight);
        mArmGoal = findAngleFromElevator(mAAndEState.getElevator().getPosition());

        mAAndEState.requestPosition(mEleGoal, mArmGoal);

        // Transitions
        if (nextRequest == IncomingRequest.REQ_IDLE) {
            return SuperState.IDLE;
        }

        return SuperState.TRAP_RETRACTING;
    }

    public void periodic() {

        Logger.recordOutput("Superstructure/Current State", currentState.toString());
        Logger.recordOutput("Superstructure/Incoming Request", nextRequest.toString());
        Logger.recordOutput("Superstructure/Shooter goal", mShootSpeed);
        Logger.recordOutput("Superstructure/At position", mAAndEState.isAtPositionWithTolerance(mArmGoal, Constants.ArmConstants.kArmPositionalToleranceDegrees, mEleGoal, Constants.ElevatorConstants.kElevatorPositionalTolerance));

        nextState = currentState;

        switch (currentState) {

            case IDLE: nextState = handleIdle(); break;
            case MOVING_TO_POS: nextState = handleMoving(); break;
            case INTAKING: nextState = handleIntaking(); break;
            case INTERPOLATING: nextState = handleInterpolating(); break;
            case SHOOTING: nextState = handleShooting(); break;
            case CLEANUP: nextState = handleCleanup(); break;
            case STOW_RESET: nextState = handleStowReset(); break;
            case EJECT_AND_RESET: nextState = handleEjectReset(); break;
            case REACHING_FOR_CHAIN: nextState = handleReachingForChain(); break;
            case CLIMBING: nextState = handleClimbing(); break;
            case REACHING_FOR_TRAP: nextState = handleReachingForTrap(); break;
            case TRAP_SHOOTING: nextState = handleTrapShooting(); break;
            case TRAP_RETRACTING: nextState = handleTrapRetracting() ; break;
        
            default:
                System.err.println("[SUPERSTRUCTURE] [ERR] In unaccounted state, please add to switch statement");
                System.exit(1);
                break;
        }
        
        currentState = nextState;
        
    }
}
