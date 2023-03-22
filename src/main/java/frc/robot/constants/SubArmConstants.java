package frc.robot.constants;

import frc.robot.utils.MotorInfo;
import frc.robot.utils.PIDControllerBuilder;

public class SubArmConstants {
  protected SubArmConstants() {
  }

  /** Subarm Rotation Servo Channel */
  public final int ROTATE_SERVO = 0;
  /** Subarm Rotation Encoder Lower Limit */
  public final double ROTATE_LOWER_LIMIT = -5;
  /** Subarm Rotation Encoder Upper Bound */
  public final double ROTATE_UPPER_LIMIT = 180;
  /**
   * Subarm Rotation Encoder Info. <strong>Do not try to create a motor controller
   * with this.</strong>
   */
  public final MotorInfo ROTATE_ENCODER_INFO = new MotorInfo(-1, MotorInfo.Type.Never)
      .encoder(new int[] { 2, 3 }, 1. / 20.);
  /**
   * Information for Subarm Pivot Motor
   */
  public final MotorInfo PIVOT_MOTOR_ID = new MotorInfo(9, MotorInfo.Type.TalonSRX)
      .encoder(new int[] { 0, 1 }, 360. / 44.4 / 4.);
  /** PID Constants for the Subarm Pivot */
  public final PIDControllerBuilder PIVOT_PID = new PIDControllerBuilder(0.05, 0, 0);
  /** Subarm Pivot Velocity (degrees per tick) */
  public final double PIVOT_VELOCITY = 1;
  /** Subarm Pivot Encoder lower limit */
  public final double PIVOT_LOWER_LIMIT = 0;
  /** Subarm Pivot Encoder upper limit */
  public final double PIVOT_UPPER_LIMIT = 180;
}
