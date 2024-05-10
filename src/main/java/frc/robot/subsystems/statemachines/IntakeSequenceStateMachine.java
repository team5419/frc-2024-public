package frc.robot.subsystems.statemachines;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.DigitalGlitchFilter;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;

import frc.robot.Constants;
import frc.robot.lib.DelayedBoolean;
import frc.robot.lib.RumbleThread;
import frc.robot.lib.VirtualSubsystem;
import frc.robot.lib.RumbleThread.ControllersToRumble;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Sweepers;


//! Note: the max speed of the intake and indexer is 80 rps

/**
 * A state machine to manage our note intake and handoff, as well as reducing noise to stop bad beam brake readings
 */

public class IntakeSequenceStateMachine extends VirtualSubsystem {
    public enum HandoffState {
        IDLE,
        INTAKING,
        WAITING,
        PASSING_TO_INDEXER,
        NOTE_ENTERED_INDEXER,
        ALIGNING_IN_INDEXER,
        NOTE_READY,
        SHOOTING,
        TRAP_SHOT,
        EJECT_AND_RESET,
        MANUAL_INTAKE,
        SWEEP_THRU,
    }

    private static IntakeSequenceStateMachine sInstance = null;
    private HandoffState mCurrentState = HandoffState.IDLE;
    private HandoffState mNextState;

    private DigitalInput mIntakeBeamBreak, mIndexerBeamBreak, mShooterBeamBreak, mFloorBeamBreak;
    // Eliminates unwanted glitches/pulses on digital input lines FMI - https://www.infineon.com/dgdl/Infineon-Component_Glitch_Filter_V2.0-Software%20Module%20Datasheets-v02_00-EN.pdf?fileId=8ac78c8c7d0d8da4017d0e886222183c
    private DigitalGlitchFilter mGlitchFilter;
    private Sweepers mSweepers;
    private Intake mIntake;
    private Indexer mIndexer;

    private DelayedBoolean mShotDelay;

    private boolean requestIdle;
    private boolean requestIntake;
    private boolean requestShoot;
    private boolean requestTrap;
    private boolean requestEjectAndReset;
    private boolean requestManualIntake;
    private boolean requestSweepThru;

    private boolean sweepRight;
    private boolean manualIntakeFWD;

    public IntakeSequenceStateMachine() {
    
        mSweepers = Sweepers.getInstance();
        mIntake = Intake.getInstance();
        mIndexer = Indexer.getInstance();

        mIntakeBeamBreak = new DigitalInput(Constants.HandoffConstants.kIntakeBeamBreakPort);
        mIndexerBeamBreak = new DigitalInput(Constants.HandoffConstants.kIndexerBeamBreakPort);
        mShooterBeamBreak = new DigitalInput(Constants.HandoffConstants.kShooterBeamBreakPort);
        mFloorBeamBreak = new DigitalInput(Constants.HandoffConstants.kFloorBeamBreakPort);

        mShotDelay = new DelayedBoolean(Timer.getFPGATimestamp(), 0.25);

        mGlitchFilter = new DigitalGlitchFilter();
        mGlitchFilter.add(mIndexerBeamBreak);
        mGlitchFilter.add(mIntakeBeamBreak);
        mGlitchFilter.add(mShooterBeamBreak);
        mGlitchFilter.add(mFloorBeamBreak);

        mGlitchFilter.setPeriodNanoSeconds(2_000);

    }

    public static IntakeSequenceStateMachine getInstance() {
        if (sInstance == null) sInstance = new IntakeSequenceStateMachine();
        return sInstance;
    }

    public boolean isSafeToMoveArmEle() {
        // do not move arm/ele if the note is in both the intake and indexer
        return mCurrentState == HandoffState.NOTE_READY || mCurrentState == HandoffState.ALIGNING_IN_INDEXER || mCurrentState == HandoffState.IDLE;
    }

    public HandoffState getState() {
        return mCurrentState;
    }

    public void zeroAllRequests() {
        requestIdle = false;
        requestIntake = false;
        requestShoot = false;
        requestTrap = false;
        requestEjectAndReset = false;
        requestSweepThru = false;
        requestManualIntake = false;
    }

    public void requestIdle() {
        zeroAllRequests();
        requestIdle = true;
    }

    public void requestIntake() {
       zeroAllRequests();
       requestIntake = true;
    }

    public void requestShoot() {
        zeroAllRequests();
        requestShoot = true;
    }

    public void requestTrap() {
        zeroAllRequests();
        requestTrap = true;
    }

    public void requestEjectAndReset() {
        zeroAllRequests();
        requestEjectAndReset = true;
    }

    public void requestManualIntake(boolean intakeFWD) {
        manualIntakeFWD = intakeFWD;
        zeroAllRequests();
        requestManualIntake = true;
    }

    public void requestSweepThru(boolean sweepingRight) {
        sweepRight = sweepingRight;
        zeroAllRequests();
        requestSweepThru = true;
    }

    public void updateState() {
        mNextState = mCurrentState;

        if (mCurrentState == HandoffState.IDLE) {
            // Actions
            mSweepers.sweepOff();
            mIntake.setPower(0);
            mIndexer.setPower(0);
            

            // Transitions
            if (requestIntake) {
                mNextState = HandoffState.INTAKING;
            } else if (requestSweepThru) {
                mNextState = HandoffState.SWEEP_THRU;
            } else if (requestShoot) {
                mNextState = HandoffState.SHOOTING;
            } else if (requestTrap) {
                mNextState = HandoffState.TRAP_SHOT;
            } else {
                allowFaultTolTransitions();
            }

        } else if (mCurrentState == HandoffState.INTAKING) {
            // Actions
            mSweepers.sweepIn();
            mIntake.setVelocity(80);
            mIndexer.setVelocity(80);

            // Transitions
            if(noteInShooter()){
                mNextState = HandoffState.NOTE_READY;
            } else if(noteInIndexer() ){
                mNextState = HandoffState.NOTE_ENTERED_INDEXER;
            } else if (requestIdle) {
                mNextState = HandoffState.IDLE;
            } else if(!noteInIndexer() && !noteInShooter() && noteInIntake()){
                mNextState = HandoffState.PASSING_TO_INDEXER;
            }


        } else if (mCurrentState == HandoffState.PASSING_TO_INDEXER) {
            // Actions
            mSweepers.sweepOut();
            mIntake.setVelocity(70);
            mIndexer.setVelocity(70);
            // use controller rumble to alert driver of successful intake
            RumbleThread.getInstance().setRumble(RumbleType.kBothRumble, 1, 0.5, ControllersToRumble.DRIVER);

            // Transitions
            if (!noteInShooter() && noteInIndexer()) {
                mNextState = HandoffState.NOTE_ENTERED_INDEXER;
            } else if (noteInShooter() && noteInIndexer()) {
                mNextState = HandoffState.NOTE_READY;
            } else if(!noteInIntake()) { 
                mNextState = HandoffState.INTAKING;
            } else {
                allowFaultTolTransitions();
            }

        } else if (mCurrentState == HandoffState.NOTE_ENTERED_INDEXER) {
            // Actions
            mSweepers.sweepOut();
            mIntake.setVelocity(40);
            mIndexer.setVelocity(40);

            // Transitions
            if (!noteInIntake()) {
                mNextState = HandoffState.ALIGNING_IN_INDEXER;
            } else {
                allowFaultTolTransitions();
            }

        } else if (mCurrentState == HandoffState.ALIGNING_IN_INDEXER) {
            // Actions
            mSweepers.sweepOut();
            mIntake.setVelocity(0);
            mIndexer.setVelocity(10);

            // Transitions
            if (noteInShooter()) {
                mNextState = HandoffState.NOTE_READY; 
            } else if(!noteInIndexer() && noteInIntake()){
                mNextState = HandoffState.PASSING_TO_INDEXER;
            } else if(!noteInIndexer() && !noteInIntake()){
                mNextState = HandoffState.INTAKING;
            } else {
                allowFaultTolTransitions();
            }

        } else if (mCurrentState == HandoffState.NOTE_READY) {
            // Actions
            mSweepers.sweepOff();
            mIntake.setPower(0);
            mIndexer.setPower(0);

            // Transitions
            if (requestShoot) {
                mNextState = HandoffState.SHOOTING;
                requestIdle = false;
            } else if (requestTrap) {
                mNextState = HandoffState.TRAP_SHOT;
            } else if (requestIdle) {
                mNextState = HandoffState.IDLE;
            } else {
                allowFaultTolTransitions();
            }

        } else if (mCurrentState == HandoffState.SHOOTING) {
            // Actions
            mIndexer.setVelocity(80);

            // Transitions
            if (requestIdle) {
                mNextState = HandoffState.IDLE;
            } else if(requestIntake){   
                mNextState = HandoffState.INTAKING;
            } else {
                allowFaultTolTransitions();
            }

        } else if (mCurrentState == HandoffState.TRAP_SHOT) {
            // Actions
            mIndexer.setVelocity(-80);
            
            // Transitions
            if (requestIdle) {
                mNextState = HandoffState.IDLE;
            } else if (requestManualIntake) {
                mNextState = HandoffState.MANUAL_INTAKE;
            }

        } else if (mCurrentState == HandoffState.EJECT_AND_RESET) {
            // Actions
            mIntake.setVelocity(80);
            mIndexer.setVelocity(80);
       
            // Transitions
            if (requestIdle) {
                mNextState = HandoffState.IDLE;
            }

        } else if (mCurrentState == HandoffState.SWEEP_THRU) {
            // Actions
            mSweepers.sweepThrough(sweepRight);

            // Transitions
            if (requestIdle) {
                mNextState = HandoffState.IDLE;
            } else if (requestManualIntake) {
                mNextState = HandoffState.MANUAL_INTAKE;
            }

        } else if (mCurrentState == HandoffState.MANUAL_INTAKE) {
            // Actions
            if (manualIntakeFWD) {
                mIntake.setVelocity(20);
                mIndexer.setVelocity(20);
            } else {
                mIntake.setVelocity(-20);
                mIndexer.setVelocity(-20);
            }

            // Transitions
            if (requestIdle) {
                mNextState = HandoffState.IDLE;
                requestIdle = false;
            }
        }

        mCurrentState = mNextState;
    }

    /** 
    * checks for fault tolerance state requests
    */
    private void allowFaultTolTransitions() {
        if (requestManualIntake) {
            mNextState = HandoffState.MANUAL_INTAKE;
        } else if (requestEjectAndReset) {
            mNextState = HandoffState.EJECT_AND_RESET;
        }
    }

    /**
     * @return boolean - true if the intake beam break senses an object
     */
    public boolean noteInIntake() {
        return !mIntakeBeamBreak.get();
    }

    /**
     * @return boolean - true if the streetsweeper beam break senses an object
     */
    public boolean noteInFloor() {
        return !mFloorBeamBreak.get();
    }

    /**
     * @return boolean - true if the indexer beam break senses an object
     */
    public boolean noteInIndexer() {
        return !mIndexerBeamBreak.get();
    }

    /**
     * @return boolean - true if the shooter beam break senses an object
     */
    public boolean noteInShooter() {
        return !mShooterBeamBreak.get();
    }


    @Override
    public void periodic() {

        Logger.recordOutput("IntakeSequence/Note in intake", noteInIntake());
        Logger.recordOutput("IntakeSequence/Note in shooter", noteInShooter());
        Logger.recordOutput("IntakeSequence/Note in indexer", noteInIndexer());
        Logger.recordOutput("IntakeSequence/Note in floor", noteInFloor());
        Logger.recordOutput("IntakeSequence/Current State", mCurrentState.toString());
        Logger.recordOutput("IntakeSequence/Idle requested", requestIdle);
        Logger.recordOutput("IntakeSequence/EjectAndReset requested", requestEjectAndReset);
        Logger.recordOutput("IntakeSequence/Intake requested", requestIntake);
        Logger.recordOutput("IntakeSequence/ManualIntake requested", requestManualIntake);
        Logger.recordOutput("IntakeSequence/Shoot requested", requestShoot);
        Logger.recordOutput("IntakeSequence/SweepThru requested", requestSweepThru);
        Logger.recordOutput("IntakeSequence/Trap requested", requestTrap);
        
        updateState();
        // anti-flubbing technique (prevents an immediate shot before flywheels are ready)
        mShotDelay.update(Timer.getFPGATimestamp(), noteInShooter());
    
    }

    @Override
    public void simulationPeriodic() {
    }

}