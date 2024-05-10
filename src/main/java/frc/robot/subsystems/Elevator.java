package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.lib.Util;
import frc.robot.lib.motor.TalonFXFactory;

public class Elevator extends SubsystemBase {
    private static Elevator sInstance = null;
    private TalonFX mLeader, mFollower;
    private double mElevatorGoal;
    
    private MotionMagicVoltage mReqMotionMagic = new MotionMagicVoltage(0);
    private MotionMagicConfigs mMotionMagicConfigs = new MotionMagicConfigs();
  
    private TalonFXConfiguration mConfigs = new TalonFXConfiguration();
    private Slot0Configs mSlot0Configs = mConfigs.Slot0;
    private Slot1Configs mSlot1Configs = mConfigs.Slot1;
   
    private DutyCycleOut mReqDutyCycle = new DutyCycleOut(0, false, true, false, false);
  
    // For reference, use https://pro.docs.ctr-electronics.com/en/latest/docs/api-reference/device-specific/talonfx/motion-magic.html

    private Elevator() {

        mLeader = TalonFXFactory.createDefaultTalon(Constants.ElevatorConstants.kLeaderId);
        mFollower = TalonFXFactory.createPermanentSlaveTalon(Constants.ElevatorConstants.kFollowerId, Constants.ElevatorConstants.kLeaderId, Constants.ElevatorConstants.kOpposeMaster);

        mMotionMagicConfigs = mConfigs.MotionMagic;
        mMotionMagicConfigs.MotionMagicCruiseVelocity = Constants.ElevatorConstants.kMotionMagicCruiseVel;
        mMotionMagicConfigs.MotionMagicAcceleration = Constants.ElevatorConstants.kMotionMagicAcceleration;
        mMotionMagicConfigs.MotionMagicJerk = Constants.ElevatorConstants.kMotionMagicJerk;

        mConfigs.CurrentLimits.StatorCurrentLimit = 50;
        mConfigs.CurrentLimits.StatorCurrentLimitEnable = true;

        // Used during Auto and Tele-op
        mSlot0Configs.kP = Constants.ElevatorConstants.kMotionMagicKP;
        mSlot0Configs.kI = Constants.ElevatorConstants.kMotionMagicKI;
        mSlot0Configs.kD = Constants.ElevatorConstants.kMotionMagicKD;
        mSlot0Configs.kG = Constants.ElevatorConstants.kMotionMagicKG;
        mSlot0Configs.kS = Constants.ElevatorConstants.kMotionMagicKS;
        mSlot0Configs.kV = Constants.ElevatorConstants.kMotionMagicKV;

        // Used during the climb to account for the weight of the robot
        mSlot1Configs.kP = Constants.ElevatorConstants.kTrapP;
        mSlot1Configs.kI = Constants.ElevatorConstants.kTrapI;
        mSlot1Configs.kD = Constants.ElevatorConstants.kTrapD;
        mSlot1Configs.kG = Constants.ElevatorConstants.kTrapG;
        mSlot1Configs.kS = Constants.ElevatorConstants.kTrapS;
        mSlot1Configs.kV = Constants.ElevatorConstants.kTrapV;

        // Configure Ratio. Be careful when of float/int divison, as this led us to using the wrong gear ratio
        // Superstructure goal values were tuned based off the wrong gear ratio :(
        mConfigs.Feedback.SensorToMechanismRatio = Constants.ElevatorConstants.kGearRatio / Constants.ElevatorConstants.kWheelCircumference;
        
        mLeader.getConfigurator().apply(mConfigs);
        mFollower.getConfigurator().apply(mConfigs);

        resetPosition();

        setBrakeMode(false);

    }


    public static Elevator getInstance() {
        if (sInstance == null) sInstance = new Elevator();
        return sInstance;
    }
  
    public void setPower(double power){
        mLeader.set(power);
    }

    public double getCurrent(){
        return mLeader.getStatorCurrent().getValueAsDouble();
    }

    // public boolean hasPassedStowCurrentLimit(){
    //     return mFilteredCurrent > Constants.ElevatorConstants.kHardStopLimit;
    // }

    /**
     * Sets the position of the elevator
     * @param positionMeters - sets position in meters
     */
    public void setGoal(double positionMeters) {
        mElevatorGoal = positionMeters;
        // mLeader.setControl(mReqMotionMagic.withPosition(Conversions.MetersToFalcon(positionMeters, Constants.ElevatorConstants.kWheelCircumference, Constants.ElevatorConstants.kGearRatio)).withSlot(0));
        mLeader.setControl(mReqMotionMagic.withPosition(positionMeters).withSlot(0));
    }

    public void setClimbGoal(double positionMeters) {
        mElevatorGoal = positionMeters;
        mLeader.setControl(mReqMotionMagic.withPosition(positionMeters).withSlot(1));
    }

    public void setBrakeMode(boolean enableBrakes){
        mLeader.setNeutralMode(enableBrakes ?  NeutralModeValue.Brake : NeutralModeValue.Coast);
        mFollower.setNeutralMode(enableBrakes ?  NeutralModeValue.Brake : NeutralModeValue.Coast);
    }

    public void setPercent(double percent) {
        mLeader.setControl(mReqDutyCycle.withOutput(percent));
    }

    public double getPosition() {
        return mLeader.getPosition().getValueAsDouble();
    }
    
    public double getVelocity(){
        return mLeader.getVelocity().getValueAsDouble();
    }

    public void setPosition(double positionMeters) {
        mLeader.setPosition(positionMeters);
    }

    /**
     * Resets the elevator position to wherever, in meters
     * @param newPositionMeters - the new position to set
     */
    public void resetPosition() {
        mLeader.setPosition(0);
    }

   /**
     * Check if the elevator is at a position, with some tolerance
     * @param positionMeters - the position that the elevator should be at
     * @param tolerance - the "wiggle room" for how different the positions can be
     * @return - true if it is at position, false if not
     */
    public boolean isAtPositionWithTolerance(double positionMeters) {
        return Util.epsilonEquals(positionMeters, getPosition(), Constants.ElevatorConstants.kElevatorPositionalTolerance);
    }

    public boolean isAtPositionWithTolerance(double positionMeters, double tolerance) {
        return Util.epsilonEquals(positionMeters, getPosition(), tolerance) ;
    }

    public boolean atPosition(){
        return isAtPositionWithTolerance(mElevatorGoal);
    }

    public boolean isAtLastSetpoint() {
        return isAtPositionWithTolerance(mReqMotionMagic.Position, Constants.ElevatorConstants.kElevatorPositionalTolerance);
    }

    public boolean isStowed() {
        return Util.epsilonEquals(getPosition(), 0, Constants.ElevatorConstants.kElevatorPositionalTolerance);
    }

    @Override
    public void periodic() {

        Logger.recordOutput("Elevator/Position", getPosition());
        Logger.recordOutput("Elevator/Current", mLeader.getStatorCurrent().getValue());
        Logger.recordOutput("Elevator/Demand", mLeader.getMotorVoltage().getValue());
        Logger.recordOutput("Elevator/Error", mLeader.getClosedLoopError().getValueAsDouble());
        Logger.recordOutput("Elevator/Target Pos", mLeader.getClosedLoopReference().getValueAsDouble());
        Logger.recordOutput("Elevator/Velocity", mLeader.getVelocity().getValueAsDouble());
        Logger.recordOutput("Elevator/Desired Velocity", mLeader.getClosedLoopReferenceSlope().getValueAsDouble());
    
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
        // Unused
    }

}