package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.lib.Util;
import frc.robot.lib.motor.TalonFXFactory;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import org.littletonrobotics.junction.Logger;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;

public class Shooter extends SubsystemBase {

    private static Shooter sInstance = null;

    private TalonFX mBottomRoller, mTopRoller;
    private TalonFXConfiguration mBottomConfigs, mTopConfigs;
    private CurrentLimitsConfigs mBottomCurrentLimitsConfigs, mTopCurrentLimitsConfigs;
    private VelocityVoltage shooterVelocity = new VelocityVoltage(0);
    private double mStatorCurrentLimit = 60;
    private double mShootVelocity = 0;

    private Shooter() {
        mBottomRoller = TalonFXFactory.createDefaultTalon(Constants.ShooterConstants.kBottomRollerId);
        mTopRoller = TalonFXFactory.createPermanentSlaveTalon(Constants.ShooterConstants.kTopRollerId, Constants.ShooterConstants.kBottomRollerId, false);

        mBottomConfigs = new TalonFXConfiguration();
        mTopConfigs = new TalonFXConfiguration();

        mBottomConfigs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        mTopConfigs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        mBottomCurrentLimitsConfigs = mBottomConfigs.CurrentLimits;
        mBottomCurrentLimitsConfigs.StatorCurrentLimit = mStatorCurrentLimit;

        mTopCurrentLimitsConfigs = mBottomConfigs.CurrentLimits;
        mTopCurrentLimitsConfigs.StatorCurrentLimit = mStatorCurrentLimit;

        mBottomConfigs.MotorOutput.PeakReverseDutyCycle = Constants.ShooterConstants.kMinOutput;
        mTopConfigs.MotorOutput.PeakReverseDutyCycle = Constants.ShooterConstants.kMinOutput;

        mBottomConfigs.Slot0.kP = (Constants.ShooterConstants.kShooterPID.kP);
        mBottomConfigs.Slot0.kI = (Constants.ShooterConstants.kShooterPID.kI);
        mBottomConfigs.Slot0.kD = (Constants.ShooterConstants.kShooterPID.kD);
        mBottomConfigs.Slot0.kV = (Constants.ShooterConstants.kShooterFF);
        mBottomConfigs.Slot0.kS = Constants.ShooterConstants.kShooterKs;

        mTopConfigs.Slot0.kP = (Constants.ShooterConstants.kShooterPID.kP);
        mTopConfigs.Slot0.kI = (Constants.ShooterConstants.kShooterPID.kI);
        mTopConfigs.Slot0.kD = (Constants.ShooterConstants.kShooterPID.kD);
        mTopConfigs.Slot0.kV = (Constants.ShooterConstants.kShooterFF);
        mTopConfigs.Slot0.kS = Constants.ShooterConstants.kShooterKs;
        
        mBottomRoller.getConfigurator().apply(mBottomConfigs);
        mTopRoller.getConfigurator().apply(mTopConfigs);

        shooterVelocity = new VelocityVoltage(0);
    }
    
    public static Shooter getInstance() {
        if (sInstance == null) sInstance = new Shooter();
        return sInstance;
    }

    /**
     * Set the shooters velocity
     * @param velocityRPS - shooter velocity in RPS
     */
    public void setVelocity (double velocityRPS) {
        mBottomRoller.setControl(shooterVelocity.withVelocity(velocityRPS));
        Logger.recordOutput("Handoff/Shooter setpoint", velocityRPS);
    }

    public void setPower(double shooterPower) {
        mBottomRoller.set(shooterPower);
    }

    /**
    * used to fix a bug 
    * @return the last stored shoot velocity
    */
    public double getCachedVelocity() {
        // Logger.recordOutput("Gotten Target Velocity", mShootVelocity);
        return mShootVelocity;
    }

    /**
    * @return the shooter velocity as a double in rotations per minute
    */
    public double getShooterVelocity() {
        return mBottomRoller.getVelocity().getValueAsDouble();
    }

    /**
     * Check if the shooter is at velocity, with some tolerance
     * @param desiredVelocity - the velocity that the motor should be at
     * @param tolerance - the "wiggle room" for how different the velocity can be
     * @return - true if it is at velocity, false if not
     */
    public boolean isAtVelocityWithTolerance(double desiredVelocity, double tolerance) {
        return Util.epsilonEquals(desiredVelocity, getShooterVelocity(), tolerance);
    }

    /**
     * Check if the shooter is at the last stored velocity, with some tolerance
     * @param tolerance - the "wiggle room" for how different the velocity can be
     */
    public boolean isAtTargetVelocityWithTolerance(double tolerance) {
        return isAtVelocityWithTolerance(mShootVelocity, tolerance);
    }
    
    public boolean isAtOrAboveVelocityWithTolerance(double velocity, double tolerance) {
        return (isAtVelocityWithTolerance(velocity, tolerance) || getShooterVelocity() >= velocity);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run;
        Logger.recordOutput("Shooter/Top Velocity", mTopRoller.getVelocity().getValueAsDouble());
        Logger.recordOutput("Shooter/Bottom Velocity", mBottomRoller.getVelocity().getValueAsDouble());
        Logger.recordOutput("Shooter/ClosedLoopError", mBottomRoller.getClosedLoopError().getValue());
        Logger.recordOutput("Shooter/ClosedLoopTarget", mBottomRoller.getClosedLoopReference().getValue());
        Logger.recordOutput("Shooter/Demand Bottom", mBottomRoller.getClosedLoopOutput().getValueAsDouble());
        Logger.recordOutput("Shooter/Demand Top", mTopRoller.getClosedLoopOutput().getValueAsDouble());
    }

    @Override
    public void simulationPeriodic() {}

}