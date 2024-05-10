package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.lib.motor.TalonFXFactory;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Sweepers extends SubsystemBase {

    private static Sweepers sInstance = null;

    private TalonFX mSweepersL, mSweepersR;

    private Sweepers() {

        mSweepersL = TalonFXFactory.createDefaultTalon(Constants.IntakeConstants.kLeftSweeperId);
        mSweepersR = TalonFXFactory.createDefaultTalon(Constants.IntakeConstants.kRightSweeperId);

        SmartDashboard.putNumber("Handoff/Intake Velocity", 0);
        SmartDashboard.putNumber("Handoff/Intake Velocity Target", 0);
        
    }

    // Get subsystem
    public static Sweepers getInstance() {
        if (sInstance == null) sInstance = new Sweepers();
        return sInstance;
    }

    /**
     * Sets individual motor voltage
     * @param sweeperVolts - the sweepers's volts
     */
    public void setVoltage(double sweeperVolts) {
        mSweepersR.setVoltage(sweeperVolts);
        mSweepersL.setVoltage(-sweeperVolts);
    }

    public void sweepOff() {
        setVoltage(0);
    }

    public void sweepIn() {
        setVoltage(12);
    }

    public void sweepOut() {
        setVoltage(-12);
    }
    
    public void sweepThrough(boolean sweepRight) {
        mSweepersL.setVoltage(sweepRight ? 12 : -12);
        mSweepersR.setVoltage(sweepRight ? -12 : 12);
    }
    
    @Override
    public void periodic() {
        Logger.recordOutput("Sweepers/Current Left", mSweepersL.getStatorCurrent().getValueAsDouble());
        Logger.recordOutput("Sweepers/Current Right", mSweepersR.getStatorCurrent().getValueAsDouble());
    }

    @Override
    public void simulationPeriodic() {}

}