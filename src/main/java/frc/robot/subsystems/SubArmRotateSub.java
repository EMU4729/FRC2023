package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.utils.logger.Logger;

public class SubArmRotateSub extends SubsystemBase {
  private final Constants cnst = Constants.getInstance();

  private final Servo servo = new Servo(cnst.SUBARM_ROTATE_SERVO);
  private final Encoder encoder = cnst.SUBARM_ROTATE_ENCODER_INFO.createEncoder();

  private boolean calibrated = false;

  /** Zeroes the encoder. */
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

  @Override
  public void periodic() {
    double turnDegrees = encoder.getDistance();

    if (turnDegrees < cnst.SUBARM_ROTATE_LOWER_LIMIT) {
      stop();
      Logger.warn("SubArmRotateSub : Lower limit reached! Stopping...");
    } else if (turnDegrees > cnst.SUBARM_ROTATE_UPPER_LIMIT) {
      stop();
      Logger.warn("SubArmRotateSub : Upper limit reached! Stopping...");
    }
  }
}
