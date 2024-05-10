package frc.robot.subsystems.misc;


import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/**
 * A class to encapsulate out lights, with helper getters and setters
 * 
 * Should probably be refactored into something a bit nicer, as at the moments its held together by hopes
 */
public class Lights extends SubsystemBase {

  // TODO: refactor this class 

  private static Lights sInstance = null;

  private AddressableLED mLEDs;
  private AddressableLEDBuffer mLEDBuffer;

  private int mBrightness, mChangeRate;
  private boolean mIsGoingUp, mDebugMode, mOverrideColor, mEnableRainbow;
  private double mWaveExponent;


  private Lights() {

    mLEDs = new AddressableLED(Constants.LEDConstants.kLedPort);
    mLEDs.setLength(Constants.LEDConstants.kLedLength);

    mLEDBuffer = new AddressableLEDBuffer(Constants.LEDConstants.kLedLength);
    mLEDs.start();

    mChangeRate = 5;
    mBrightness = 255;
    mIsGoingUp = false;
    mEnableRainbow = false;
    mWaveExponent = 0.5;

  }

  // Get subsystem
  public static Lights getInstance() {
    if (sInstance == null) sInstance = new Lights();

    return sInstance;
  }

  public boolean inDebugMode() {
    return mDebugMode;
  }

  public void setDebugMode(boolean debug) {
    mDebugMode = debug;
  }

  public boolean rainbowEnabled() {
    return mEnableRainbow;
  }

  public void setEnableRainbow(boolean rainbow) {
    mEnableRainbow = rainbow;
  }

  public boolean overrideColor() {
    return mOverrideColor;
  }

  public void setColor(int r, int g, int b) {
    for (int i = 0; i < Constants.LEDConstants.kLedLength; i++) {
      mLEDBuffer.setRGB(i, r, g, b);
    }
  }

  public void setColor(AddressableLEDBuffer buffer) {
    // mLEDs.setData(buffer);
    mLEDBuffer = buffer;
  }

  public void setSingleLedColor(int index, int r, int g, int b) {

    if (index < 0 || index > Constants.LEDConstants.kLedLength) {
      throw new IndexOutOfBoundsException(index);
    }

    mLEDBuffer.setRGB(index, r, g, b);
  }

  public void strobe(int r, int g, int b, double duration) {
    boolean isOn = ((Timer.getFPGATimestamp() % duration) / duration) > 0.5;
    if (isOn) {
      setColor(r, g, b);
    } else {
      setColor(0, 0, 0);
    }
  }

  public void tick() {
    if (DriverStation.isEStopped()) {
      strobe(255, 255, 255, 1);
      return;
    }

    if (mDebugMode || mOverrideColor) {
      return;
    }

    if (mBrightness <= 255 && mIsGoingUp) {
      mBrightness += mChangeRate;
    } else if (!mIsGoingUp) {
      mBrightness -= mChangeRate;
    }

    if (mBrightness == 255) {
      mIsGoingUp = false;
    }

    if (mBrightness <= 0) {
      mIsGoingUp = true;
    }

    setColor(mBrightness, 0, 0);
  }

  public void tickWave(Color c1, Color c2, double cycleLength, double duration) {
    if (DriverStation.isEStopped()) {
      strobe(255, 255, 255, 1);
      return;
    }

    if (mDebugMode || mOverrideColor) {
      return;
    }

    double x = (1 - ((Timer.getFPGATimestamp() % duration) / duration)) * 2.0 * Math.PI;
    double xDiffPerLed = (2.0 * Math.PI) / cycleLength;
    for (int i = 0; i < Constants.LEDConstants.kLedLength; i++) {
      x += xDiffPerLed;
      double ratio = (Math.pow(Math.sin(x), mWaveExponent) + 1.0) / 2.0;
      if (Double.isNaN(ratio)) {
        ratio = (-Math.pow(Math.sin(x + Math.PI), mWaveExponent) + 1.0) / 2.0;
      }
      if (Double.isNaN(ratio)) {
        ratio = 0.5;
      }
      double red = (c1.red * (1 - ratio)) + (c2.red * ratio);
      double green = (c1.green * (1 - ratio)) + (c2.green * ratio);
      double blue = (c1.blue * (1 - ratio)) + (c2.blue * ratio);
      mLEDBuffer.setLED(i, new Color(red, green, blue));
    }

  }

  public void off() {
    setColor(0, 0, 0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    mLEDs.setData(mLEDBuffer);
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
    
  }

}
