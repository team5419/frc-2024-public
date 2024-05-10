package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;

import frc.robot.lib.motor.TalonFXFactory;
import com.ctre.phoenix6.hardware.TalonFX;

public class Intake extends SubsystemBase {

    private static Intake sInstance = null;

    private TalonFX mIntake;
    private TalonFXConfiguration mTalonConfig = new TalonFXConfiguration();
    private Slot0Configs slot0Configs = mTalonConfig.Slot0;
    private VelocityVoltage mReqVelTorque = new VelocityVoltage(0);

    private Intake() {

        mIntake = TalonFXFactory.createDefaultTalon(Constants.IntakeConstants.kIntakeId);

        slot0Configs = mTalonConfig.Slot0;
        slot0Configs.kP = Constants.IntakeConstants.kIntakePID.kP;
        slot0Configs.kI = Constants.IntakeConstants.kIntakePID.kI;
        slot0Configs.kD = Constants.IntakeConstants.kIntakePID.kD;
        slot0Configs.kV = Constants.IntakeConstants.kIntakeFF;
        slot0Configs.kS = Constants.IntakeConstants.kS;

        mIntake.getConfigurator().apply(mTalonConfig);
        
    }

    // Get subsystem
    public static Intake getInstance() {
        if (sInstance == null) sInstance = new Intake();
        return sInstance;
    }

    /**
     * Sets individual motor voltage
     * @param sweeperVolts - the sweepers's volts
     * @param intakeVolts - the intake's volts
     */
    public void setVoltage(double intakeVolts) {
        mIntake.setVoltage(intakeVolts);
    }

    public void setVelocity(double velocityRPM) {
        mIntake.setControl(mReqVelTorque.withVelocity(velocityRPM));
        Logger.recordOutput("Handoff/Intake/Velocity Target", velocityRPM);
    }

    public void setPower(double power) {
        mIntake.set(power);
    }
    
    @Override
    public void periodic() {
        Logger.recordOutput("Handoff/Intake/Velocity", mIntake.getVelocity().getValueAsDouble());
        Logger.recordOutput("Handoff/Indexer/Demand", mIntake.getClosedLoopOutput().getValueAsDouble());
    }

    @Override
    public void simulationPeriodic() {}

}