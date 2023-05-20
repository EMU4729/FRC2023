package frc.robot.constants;

import frc.robot.utils.EncoderBuilder;
import frc.robot.utils.MotorBuilder;
import frc.robot.utils.PIDControllerBuilder;

public class SubArmConstants {
  protected SubArmConstants() {
  }

  /** Subarm Rotation Servo Channel */
  public final int ROTATE_SERVO = 9;
  /** Subarm Rotation Encoder Lower Limit */
  public final double ROTATE_LOWER_LIMIT = -5;
  /** Subarm Rotation Encoder Upper Bound */
  public final double ROTATE_UPPER_LIMIT = 180;

  /**
   * Subarm Rotation Encoder Info.
   */
  public final EncoderBuilder ROTATE_ENCODER_INFO = new EncoderBuilder(new int[] { 6, 7 }, 1. / 20.); // TODO: Gearing ratio is completely clapped

  /**
   * Information for Subarm Pivot Motor
   */
  public final MotorBuilder PIVOT_MOTOR_ID = new MotorBuilder(9, MotorBuilder.Type.TalonSRX);

  /** Information for Subarm Pivot Encoder */
  public final EncoderBuilder PIVOT_ENCODER_ID = new EncoderBuilder(new int[] { 10, 11 }, 360. / 44.4 / 4.);

  /** PID Constants for the Subarm Pivot */
  public final PIDControllerBuilder PIVOT_PID = new PIDControllerBuilder(0.05, 0, 0, 0);
  /** Subarm Pivot Velocity (degrees per tick) */
  public final double PIVOT_VELOCITY = 1;
  /** Subarm Pivot Encoder lower limit */
  public final double PIVOT_LOWER_LIMIT = -180;
  /** Subarm Pivot Encoder upper limit */
  public final double PIVOT_UPPER_LIMIT = 180;
}
