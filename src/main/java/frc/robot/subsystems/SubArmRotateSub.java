package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
import frc.robot.shufflecontrol.ShuffleControl;
import frc.robot.utils.logger.Logger;

/** Subsystems for controlling Subarm Rotation */
public class SubArmRotateSub extends SubsystemBase {
  private final Servo servo = new Servo(Constants.subarm.ROTATE_SERVO);
  private final Encoder encoder = Constants.subarm.ROTATE_ENCODER_INFO.build();

  private boolean calibrated = false;

  /**
   * Zeroes the encoder. Use this method when the subarm is rotated as
   * anticlockwise as possible.
   */
  public void calibrate() {
    encoder.reset();
    calibrated = true;
  }

  /** Warns user if uncalibrated */
  private void calibrationCheck() {
    if (!calibrated) {
      Logger.warn("SubArmRotateSub : Uncalibrated! Be careful!");
    }
  }

  /** Updates shuffleboard. Call this function regularly. */
  private void updateShuffleboard() {
    ShuffleControl.subArmTab.setRotationAngle(encoder.getDistance());
    ShuffleControl.calibrationTab.setSubArmRotateCalibrated(calibrated);
  }

  /**
   * Checks if the subarm has rotated beyond its limits.
   * 
   * @param stop Whether to stop the subarm if the limits have been surpassed or
   *             not.
   * @return True if limits have been surpassed, false if not
   */
  private boolean checkLimit(boolean stop) {
    double turnDegrees = encoder.getDistance();

    if (turnDegrees < Constants.subarm.ROTATE_LOWER_LIMIT) {
      if (stop) {
        stop();
      }
      Logger.warn("SubArmRotateSub : Lower limit reached! Stopping...");
      ShuffleControl.subArmTab.setRotationInBounds(false);
      return true;
    } else if (turnDegrees > Constants.subarm.ROTATE_UPPER_LIMIT) {
      if (stop) {
        stop();
      }
      Logger.warn("SubArmRotateSub : Upper limit reached! Stopping...");
      ShuffleControl.subArmTab.setRotationInBounds(false);
      return true;
    }

    ShuffleControl.subArmTab.setRotationInBounds(true);
    return false;
  }

  /**
   * Checks if the subarm has rotated beyond its limits.
   * 
   * @return True if limits have been surpassed, false if not
   */
  private boolean checkLimit() {
    return checkLimit(false);
  }

  /** Stops the servo. */
  public void stop() {
    calibrationCheck();
    servo.set(0.5);
  }

  /** Starts running the servo clockwise. */
  public void clockwise() {
    calibrationCheck();
    servo.set(1);
  }

  /** Starts running the servo anticlockwise. */
  public void anticlockwise() {
    calibrationCheck();
    servo.set(0);
  }

  /**
   * Makes a {@link Command} that rotates the servo to the specified value.
   * 
   * @param value The specified value.
   * @return The constructed {@link Command}
   */
  private Command turnCommand(double value) {
    return new FunctionalCommand(
        () -> {
          calibrationCheck();
        }, () -> {
          servo.set(value);
        },
        (interrupted) -> stop(),
        this::checkLimit,
        this);
  }

  /** @return a {@link Command} to rotate the subarm clockwise */
  public Command turnClockwise() {
    return turnCommand(1);
  }

  /** @return a {@link Command} to rotate the subarm anticlockwise */
  public Command turnAnticlockwise() {
    return turnCommand(0);
  }

  @Override
  public void periodic() {
    updateShuffleboard();
    if (!calibrated) {
      return;
    }
    checkLimit(true);
  }
}
