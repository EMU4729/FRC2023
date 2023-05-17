package frc.robot.constants;

import frc.robot.utils.CurveFit;
import frc.robot.utils.EncoderBuilder;
import frc.robot.utils.MotorBuilder;
import frc.robot.utils.PIDControllerBuilder;

public class ArmConstants {
  protected ArmConstants() {
  }

  /** Information for Arm Seg1 Master Motor */
  public final MotorBuilder SEG1_MASTER_MOTOR_ID = new MotorBuilder(7, MotorBuilder.Type.VictorSPX).withBrake();

  /** Arm Seg1 Encoder Builder */
  public final EncoderBuilder SEG1_ENCODER = new EncoderBuilder(new int[] { 4, 5 }, 360. / 16 / 2048.);

  // arm
  /** Information for Arm Seg1 Slave Motor */
  public final MotorBuilder SEG1_SLAVE_MOTOR_ID = new MotorBuilder(8, MotorBuilder.Type.VictorSPX).withBrake();
  /** Information for Arm Seg2 Master Motor */
  public final MotorBuilder SEG2_MASTER_MOTOR_ID = new MotorBuilder(5, MotorBuilder.Type.VictorSPX).withBrake();

  /** Arm Seg1 Encoder Builder */
  public final EncoderBuilder SEG2_ENCODER = new EncoderBuilder(new int[] { 8, 9 }, 360. / 2048.);

  // arm
  /** Information for Arm Seg2 Slave Motor */
  public final MotorBuilder SEG2_SLAVE_MOTOR_ID = new MotorBuilder(6, MotorBuilder.Type.VictorSPX).withBrake();
  /**
   * Length of the armseg2 (second segment)(between axles), in (mm) @wip update
   * arm length
   */ // wip
  public final double SEG2_LENGTH = 0.8; // UPDATE
  /**
   * Length of the arm seg1 (first segment)(between axles), in (mm) @wip update
   * arm length
   */ // wip
  public final double SEG1_LENGTH = 0.91; // UPDATE
  /** height off the carpet of the seg1 rm axle (m) @wip value just a guess */ // wip
  public final double SEG1_AXLE_HEIGHT = 0.2;
  /**
   * amount the arm seg1 axle is offset from the centerline in x (m)(forward
   * pos) @wip value guessed
   */ // wip
  public final double SEG1_X_OFFSET = 0.01;
  /**
   * distance in x and y from the arm seg1 axle to the max distance the robot is
   * allowed to reach
   * [[x- (behind), x+(in front)], [y-(below), y+(above)]] (m)
   */
  public final double[][] MAX_REACH_LEGAL = {
      { -(Constants.features.ROBOT_LENGTH / 2 + Constants.features.ROBOT_REACH_MAX[0]
          + SEG1_X_OFFSET),
          Constants.features.ROBOT_LENGTH / 2 + Constants.features.ROBOT_REACH_MAX[0]
              - SEG1_X_OFFSET },
      { -SEG1_AXLE_HEIGHT, Constants.features.ROBOT_REACH_MAX[1] - SEG1_AXLE_HEIGHT } };
  /**
   * length of the combined 2 segment arm, ARM_SEG1_X_OFFSET should be subtracted
   * to find true x (mm)
   */
  public final double MAX_REACH_PHYSICAL = SEG1_LENGTH + SEG2_LENGTH;
  /**
   * area the arm should never enter (mm)[[x-(behind), x+(in front)], [y (from
   * floor)]] @wip needs right y
   */ // wip
  public final double[][] REACH_EXCLUSION = {
      { -(Constants.features.ROBOT_LENGTH / 2 - SEG1_X_OFFSET),
          (Constants.features.ROBOT_LENGTH / 2 + SEG1_X_OFFSET), },
      { 0.200 } };
  /** Dimensions (width, height) of the robot that the arm should never reach. */
  public final double[][] REACH_ROBOT_EXCLUSION = {
      { -(Constants.features.ROBOT_LENGTH / 2), Constants.features.ROBOT_LENGTH / 2 },
      { -0.15, 0 }
  };
  /**
   * height the arm should seek to hold if moving or stored inside frame perimiter
   */
  public final double SWING_THROUGH_HEIGHT = SEG1_LENGTH - SEG2_LENGTH;

  /** Copilot input curve for seg 1 */
  public final CurveFit SEG1_INPUT_CURVE = new CurveFit(-1, 1, 0, 0.4, 1);
  /** Copilot input curve for seg 2 */
  public final CurveFit SEG2_INPUT_CURVE = new CurveFit(-1, 1, 0, 0.3, 1);

  /** The arm sustain strategies */
  public static enum SustainStrategy {
    /** No sustain strategy */
    NONE,
    /** Sustain using an angle-power curve in addition to driver input */
    CURVE,
    /** Sustain using a feedforward loop */
    FEEDFORWARD
  }

  /** The current arm sustain strategy being used. */
  public final SustainStrategy SUSTAIN_STRATEGY = SustainStrategy.CURVE;

  /** Seg2 Sustain curve for curve sustain strategy */
  public final CurveFit SUSTAIN_CURVE = new CurveFit(-180, 180, -0.15, 0.15, 1.5);

  /** KS constant for feedforward sustain strategy */
  public final double SUSTAIN_FEEDFORWARD_KS = 0;
  /** KV constant for feedforward sustain strategy */
  public final double SUSTAIN_FEEDFORWARD_KV = 0;
  /** KA constant for feedforward sustain strategy */
  public final double SUSTAIN_FEEDFORWARD_KA = 0;

  /** Flag controlling if integral sustain is used */
  public final boolean USE_INTEGRAL_SUSTAIN = false;

  /** PID controller builder for integral sustain */
  public final PIDControllerBuilder INTEGRAL_SUSTAIN = new PIDControllerBuilder(0, 0.05, 0);
}
