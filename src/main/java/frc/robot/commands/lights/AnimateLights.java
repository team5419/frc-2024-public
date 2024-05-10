package frc.robot.commands.lights;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.misc.Lights;
import frc.robot.subsystems.statemachines.IntakeSequenceStateMachine;
import frc.robot.subsystems.statemachines.Superstructure;
import frc.robot.subsystems.statemachines.IntakeSequenceStateMachine.HandoffState;
import frc.robot.subsystems.statemachines.Superstructure.SuperState;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.swerve.generated.TunerConstants;


public class AnimateLights extends Command {

  private Lights mLights = Lights.getInstance();
  private Swerve mSwerve = TunerConstants.Swerve;
  private IntakeSequenceStateMachine mHandoffManager = IntakeSequenceStateMachine.getInstance();

  private int mBrightness = 0;
  private boolean mIsGoingUp = true;
  private int mChangeRate = 5;

  // variables for rainbow animation
  int mRainbowFirstPixelHue = 0; // red
  public int mHueShiftStep = 1;

  public AnimateLights() {
    addRequirements(mLights);
  }

  public void initialize() {}

  public void execute() {

    HandoffState mCurrentState = mHandoffManager.getState();

    if (mBrightness <= 255 && mIsGoingUp) {
      mBrightness += mChangeRate;
    } else if (!mIsGoingUp) {
      mBrightness -= mChangeRate;
    }

    if (mBrightness == 255) {
      mIsGoingUp = false;
    }

    if (mBrightness <= 50) {
      mIsGoingUp = true;
    }

    if (mLights.inDebugMode() || mLights.overrideColor()) {
      return;
    }

    if (DriverStation.isEStopped()) {
      mLights.strobe(255, 255, 255, 1); 
    } else if ((Superstructure.getInstance().getState() == SuperState.INTERPOLATING && Superstructure.getInstance().interpolationReady()) || mLights.rainbowEnabled()) {
      // TODO: test this to make sure that its working
      rainbowTick();
    } else if (mCurrentState == HandoffState.IDLE) {
      mLights.setColor(mBrightness, 0, 0);
    } else if (mCurrentState == HandoffState.WAITING) {
      mLights.strobe(255, 255, 255, 0.25);
    } else if (mCurrentState == HandoffState.INTAKING) {
      mLights.strobe(0, 0, 221, 0.5);
    } else if (mCurrentState == HandoffState.PASSING_TO_INDEXER || mCurrentState == HandoffState.ALIGNING_IN_INDEXER || mCurrentState == HandoffState.NOTE_ENTERED_INDEXER) {
      mLights.strobe(252, 236, 0, 0.5); // Strobe Orange
    } else if (mHandoffManager.getState() == HandoffState.NOTE_READY) {
      mLights.setColor(0, 255, 0); // Green
    } else {
      mLights.setColor(mBrightness, 0, 0);
    }
  }

  public void end(boolean interrupted) {

    mLights.setColor(0, 0, 0);

  }

  public boolean isFinished() {
    return false;
  }

  // from
  // https://docs.wpilib.org/en/stable/docs/software/hardware-apis/misc/addressable-leds.html#creating-a-rainbow-effect
  private void rainbowTick() {
    final AddressableLEDBuffer buffer = new AddressableLEDBuffer(Constants.LEDConstants.kLedLength);
    // For every pixel
    for (var i = 0; i < Constants.LEDConstants.kLedLength; i++) {
      // Calculate the hue - hue is easier for rainbows because the color
      // shape is a circle so only one value needs to precess
      final var hue = (mRainbowFirstPixelHue + (i * 180 / Constants.LEDConstants.kLedLength)) % 180;
      // Set the value
      buffer.setHSV(i, hue, 255, 128);
    }
    // Increase by to make the rainbow "move"
    mRainbowFirstPixelHue += 3;
    // Check bounds
    mRainbowFirstPixelHue %= 180;

    mLights.setColor(buffer);
  }
}
