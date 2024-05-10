package frc.robot.subsystems.misc;

import org.photonvision.PhotonUtils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import frc.robot.lib.Util;

/**
 * @author Grayson A
 */
public class ShotAngleInterpolator {

    private static ShotAngleInterpolator sInstance = null;
    private InterpolatingDoubleTreeMap mArmMap, mElevatorMap, mShooterMap;

    /**
     * InterpolatedSuperstructureState - a simple record to hold a interpolated state
     */
    public record InterpolatedSuperstructureState(double interpolatedElevatorHeight, double interpolatedArmAngle, double interpolatedShooterSpeed) {}

    public ShotAngleInterpolator() {
        mArmMap = new InterpolatingDoubleTreeMap();
        mElevatorMap = new InterpolatingDoubleTreeMap();
        mShooterMap = new InterpolatingDoubleTreeMap();
        clear();
    }

    // Get subsystem
    public static ShotAngleInterpolator getInstance() {
        if (sInstance == null) sInstance = new ShotAngleInterpolator();
        return sInstance;
    }
    

    public void addInitialInterpolationData() {
        addArmMeasurement(1.4,  -11);
        addArmMeasurement(2.4, 3.5); // Set: 2.6, [5.6, 1.5]
        // addArmMeasurement(2.7, 8.7); // Set: 2.6, [5.6, 1.5]
        addArmMeasurement(3.4, 13.5); // Set: 14, [14, --]
        addArmMeasurement(4.4, 18);
        addArmMeasurement(5.5, 22);

        addElevatorMeasurement(3, 0);
        addElevatorMeasurement(3.6, 0);

        addShooterMeasurement(1.4, 33.33);
        addShooterMeasurement(4, 66.67);
    }


    public void addArmMeasurement(double distance, double shotAngle) {
        mArmMap.put(distance, shotAngle);
    }

    public void addElevatorMeasurement(double distance, double elevatorHeight) {
        mElevatorMap.put(distance, elevatorHeight);
    }

    public void addShooterMeasurement(double distance, double shotSpeed) {
        mShooterMap.put(distance, shotSpeed);
    }

    public void clear() {
        mArmMap.clear();
        mElevatorMap.clear();
        mShooterMap.clear();
    }

    public double getInterpolatedArmAngle(double distance) {
        return mArmMap.get(distance);
    }

    public double getInterpolatedElevatorAngle(double distance) {
        return mElevatorMap.get(distance);
    }

    public double getInterpolatedShooterSpeed(double distance) {
        return mShooterMap.get(distance);
    }

    public InterpolatedSuperstructureState getInterpolatedSuperstructure(double distance) {
        return new InterpolatedSuperstructureState(getInterpolatedElevatorAngle(distance), getInterpolatedArmAngle(distance), getInterpolatedShooterSpeed(distance));
    }

    public InterpolatedSuperstructureState getVelocityCompensatedSuperstructure(double dt) {
        Pose2d newPos = Util.getFuturePosition(dt);
        double distance = PhotonUtils.getDistanceToPose(newPos, Util.getSpeakerPose());
        return getInterpolatedSuperstructure(distance);
    }
}