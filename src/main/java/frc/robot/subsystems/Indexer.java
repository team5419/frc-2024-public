package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;

import frc.robot.lib.motor.TalonFXFactory;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class Indexer extends SubsystemBase {

    private static Indexer sInstance = null;

    private TalonFX mIndexer;
    private TalonFXConfiguration mTalonConfig = new TalonFXConfiguration();
    private Slot0Configs slot0Configs = mTalonConfig.Slot0;
    private VelocityVoltage mReqVelTorque = new VelocityVoltage(0);

    private Indexer() {

        mIndexer = TalonFXFactory.createDefaultTalon(Constants.ShooterConstants.kIndexerId);

        slot0Configs = mTalonConfig.Slot0;
        slot0Configs.kP = Constants.ShooterConstants.kIndexerPID.kP;
        slot0Configs.kI = Constants.ShooterConstants.kIndexerPID.kI;
        slot0Configs.kD = Constants.ShooterConstants.kIndexerPID.kD;
        slot0Configs.kV = Constants.ShooterConstants.kIndexerFF;
        slot0Configs.kS = Constants.ShooterConstants.kIndexerKs;

        mTalonConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        mTalonConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        mIndexer.getConfigurator().apply(mTalonConfig);
    }
    
    // Get subsystem
    public static Indexer getInstance() {
        if (sInstance == null) sInstance = new Indexer();
        return sInstance;
    }

    /**
     * Set the indexers velocity
     * @param velocityRPM - the velocity to set
     */
    public void setVelocity (double velocityRPM) {
        mIndexer.setControl(mReqVelTorque.withVelocity(velocityRPM));
        Logger.recordOutput("Handoff/Indexer Velocity Target", velocityRPM);
    }

    /**
     * Set the indexer motor, in volts
     * @param power - the percent of power to set (0-1)
     */
    public void setPower(double power) {
        mIndexer.set(power);
    }

    @Override
    public void periodic() {
        Logger.recordOutput("Handoff/Indexer Velocity", mIndexer.getVelocity().getValueAsDouble());
    }

    @Override
    public void simulationPeriodic() {}

}