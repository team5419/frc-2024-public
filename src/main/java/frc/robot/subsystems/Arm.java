package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;
import org.photonvision.PhotonUtils;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.lib.Util;
import frc.robot.lib.motor.TalonFXFactory;
import frc.robot.subsystems.swerve.generated.TunerConstants;
import edu.wpi.first.math.util.Units;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

/**
 * @author Grayson A - Dashiell J - Ari E - Penn L
 */

public class Arm extends SubsystemBase {

    private static Arm sInstance = null;

    private TalonFXConfiguration mTalonConfig = new TalonFXConfiguration();

    private Slot0Configs slot0Configs = mTalonConfig.Slot0;

    private TalonFX mPivotMotor;
    
    private MotionMagicVoltage mReqMotionMagic = new MotionMagicVoltage(0);
    private MotionMagicConfigs mMotionMagicConfigs = new MotionMagicConfigs();

    private double mArmGoal;
    private final Pigeon2 mPigeon = new Pigeon2(49);

    private Arm() { 

        mPivotMotor = TalonFXFactory.createDefaultTalon(Constants.ArmConstants.kPivotId);

        slot0Configs = mTalonConfig.Slot0;
        slot0Configs.kP = Constants.ArmConstants.kPivotP;
        slot0Configs.kI = Constants.ArmConstants.kPivotI;

        slot0Configs.kG = Constants.ArmConstants.kG;
        slot0Configs.kS = Constants.ArmConstants.kS;
        slot0Configs.GravityType = GravityTypeValue.Arm_Cosine;

        slot0Configs.kV = Constants.ArmConstants.kV;

        mTalonConfig.Feedback.SensorToMechanismRatio = Constants.ArmConstants.kArmRatio;

        mMotionMagicConfigs = mTalonConfig.MotionMagic;
        mMotionMagicConfigs.MotionMagicCruiseVelocity = Units.degreesToRotations(Constants.ArmConstants.kMotionMagicCruiseVel);
        mMotionMagicConfigs.MotionMagicAcceleration = Units.degreesToRotations(Constants.ArmConstants.kMotionMagicAcceleration); 
        mMotionMagicConfigs.MotionMagicJerk = Units.degreesToRotations(Constants.ArmConstants.kMotionMagicJerk); 

        var mInvertCfg = mTalonConfig.MotorOutput;
        mInvertCfg.Inverted = InvertedValue.Clockwise_Positive;

        mTalonConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        // mTalonConfig.CurrentLimits.StatorCurrentLimit = 40;
        // mTalonConfig.CurrentLimits.StatorCurrentLimitEnable = true;

        mPivotMotor.getConfigurator().apply(mTalonConfig);
        
        resetPositionUsingPigeon();
    }

    /**
     * @return - the instance of the class
     */
    public static Arm getInstance() {
        if (sInstance == null) sInstance = new Arm();
        return sInstance;
    }

    /**
     * @return - current shooter angle in degrees
     */

    public double getShooterAnglePigeon() {
        double angle = -180 - Units.radiansToDegrees(Math.atan2(mPigeon.getGravityVectorX().getValueAsDouble(), mPigeon.getGravityVectorZ().getValueAsDouble()));
        if(angle < -180) angle += 360;

        return angle;
    }

    /**
     * Resets the "zero" position
     */
    public void resetPositionUsingPigeon() {
        double pos = Units.degreesToRotations(getShooterAnglePigeon() - Constants.ArmConstants.kOffsetArmToShooter - Constants.ArmConstants.kOffsetCGToArm);
        mPivotMotor.setPosition(pos);
    }

    public void setDefaultMotionMagicConfigs() {
        // Todo
    }
    
    public void setAggressiveMotionMagicConfigs() {
        // Todo
    }

    /**
     * Set the position the pivot should go to, in degrees
     * Should ignore the input if it is not safe to move
     */
    public void setGoal(double targetPositionDegrees){
        mArmGoal = targetPositionDegrees;
        mReqMotionMagic.Slot = 0;
        mPivotMotor.setControl(mReqMotionMagic.withPosition(Units.degreesToRotations(targetPositionDegrees - Constants.ArmConstants.kOffsetCGToArm)));

        Logger.recordOutput("Arm/Arm Target degrees", targetPositionDegrees);
        Logger.recordOutput("Arm/Arm Calculated Target", Units.degreesToRotations(targetPositionDegrees - Constants.ArmConstants.kOffsetCGToArm));
    }

    /**
     * @return - the velocity of the arm in degrees per second
     */
    public double getVelocity() {
        return Units.rotationsToDegrees(mPivotMotor.getVelocity().getValueAsDouble());
    }

    public void setPower(double power) {
        mPivotMotor.set(power);
    }

    public double getCurrent() {
        return mPivotMotor.getStatorCurrent().getValueAsDouble();
    }

    /**
     * Get the position that the pivot motor is at 
     * @return the position in degrees
     */
    public double getPosition() {
        return Units.rotationsToDegrees(mPivotMotor.getPosition().getValueAsDouble()) + Constants.ArmConstants.kOffsetCGToArm;

    }

    public void stop() {
        mPivotMotor.set(0);
    }

    public boolean atPosition() {
        return isAtPositionWithTolerance(mArmGoal);
    }
    
    public boolean isAtLastSetpoint() {
        double distance = PhotonUtils.getDistanceToPose(TunerConstants.Swerve.getPose(), Util.getSpeakerPose());
        double tolerance = Util.clamp(6 - (distance), 0.6, 2);
        return isAtPositionWithTolerance(Units.rotationsToDegrees(mReqMotionMagic.Position) + Constants.ArmConstants.kOffsetCGToArm, tolerance);
    }

    /**
     * Check if the pivot is at a position, with some tolerance
     * @param positionDegrees - the position that the motor should be at
     * @param armTolerance - the "wiggle room" for how different the velocity can be
     * @return - true if it is at position, false if not
     */
    public boolean isAtPositionWithTolerance(double positionDegrees) {
        return Util.epsilonEquals(getPosition(), positionDegrees, Constants.ArmConstants.kArmPositionalToleranceDegrees); // Needed so that it compiles, remove when working on function
    }

    public boolean isAtPositionWithTolerance(double positionDegrees, double armTolerance) {
        return Util.epsilonEquals(getPosition(), positionDegrees, armTolerance); // Needed so that it compiles, remove when working on function
    }

    public boolean isAtVelocityWithTolerance(double speed) {
        return Math.abs(mPivotMotor.getVelocity().getValueAsDouble()) < Constants.ArmConstants.kArmVelocityTolerance;
    }

    public boolean isAtVelocityWithTolerance(double speed, double tolerance) {
        return Math.abs(mPivotMotor.getVelocity().getValueAsDouble()) < tolerance;
    }

    @Override
    public void periodic() {
        Logger.recordOutput("Arm/Position", getPosition());
        Logger.recordOutput("Arm/Current",mPivotMotor.getStatorCurrent().getValue());
        Logger.recordOutput("Arm/Demand",mPivotMotor.getMotorVoltage().getValue());
        Logger.recordOutput("Arm/Error",mPivotMotor.getClosedLoopError().getValue());
        Logger.recordOutput("Arm/Reference", mPivotMotor.getClosedLoopReference().getValueAsDouble());
        Logger.recordOutput("Arm/Deg Reference", Units.rotationsToDegrees(mPivotMotor.getClosedLoopReference().getValueAsDouble()) + Constants.ArmConstants.kOffsetCGToArm);
        Logger.recordOutput("Arm/Vel", getVelocity());
        Logger.recordOutput("Arm/Raw talon position", mPivotMotor.getPosition().getValue());
        Logger.recordOutput("Arm/Closed loop output", mPivotMotor.getClosedLoopOutput().getValueAsDouble());
        Logger.recordOutput("Arm/Falcon raw position",mPivotMotor.getPosition().getValue());
        Logger.recordOutput("Arm/Motion magic vel", mMotionMagicConfigs.MotionMagicCruiseVelocity);
        Logger.recordOutput("Arm/Shooter Angle Pigeon", getShooterAnglePigeon());
        Logger.recordOutput("Arm/Desired Velocity", Units.rotationsToDegrees(mPivotMotor.getClosedLoopReferenceSlope().getValueAsDouble()));
        Logger.recordOutput("Arm/Pigeon reset degrees", Units.degreesToRotations(getShooterAnglePigeon() - Constants.ArmConstants.kOffsetArmToShooter - Constants.ArmConstants.kOffsetCGToArm));
        Logger.recordOutput("Arm/Interpolated setpoint being checked", Units.rotationsToDegrees(mReqMotionMagic.Position) + Constants.ArmConstants.kOffsetCGToArm);
    }

    @Override
    public void simulationPeriodic() {}

}