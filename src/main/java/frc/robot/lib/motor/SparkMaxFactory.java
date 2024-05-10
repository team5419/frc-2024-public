package frc.robot.lib.motor;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import frc.robot.lib.CanDeviceId;

public class SparkMaxFactory {
  
    public CANSparkMax mSparkMax;
    public CanDeviceId canID;

    public boolean inverted;
    public boolean brakeMode;
    public static int stallLimit = 60;
    public static int freeLimit = 60;
    public static double rampTime = 0.5;

    public static CANSparkMax setUpSpark(CanDeviceId canID, boolean inverted, boolean brakeMode) {

        CANSparkMax mSparkMax = new CANSparkMax(canID.getDeviceNumber(), MotorType.kBrushless);

        mSparkMax.restoreFactoryDefaults();
        mSparkMax.setInverted(inverted);
        mSparkMax.setIdleMode(brakeMode ? IdleMode.kBrake : IdleMode.kCoast);
        mSparkMax.setSmartCurrentLimit(stallLimit, freeLimit);
        mSparkMax.setOpenLoopRampRate(rampTime);

        return mSparkMax;

    }
}
