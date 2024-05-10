package frc.robot;

public enum SuperstructureGoal {

    STOW(0, -12.5, 0),
    SUBWOOFER(0, -12.5, 33.33),
    WING_LINE(0.03, 27, 58.33),
    AMP(0.11, 80, 15),
    TRAP(0.35, 50, 0),
    PODIUM(0.03, 21, 33.33),
    PODIUM_DUNK(0.3, 21, 33.33),
    RESET(0.04, -8, 0),
    STOCKPILE(0, -11, 30);

    public double elevatorHeight, armRotation, shooterSpeed;

    SuperstructureGoal(double height, double rotation, double speed){
        this.elevatorHeight = height;
        this.armRotation = rotation;
        this.shooterSpeed = speed;
    }

}
