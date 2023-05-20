package frc.robot.constants;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import frc.robot.utils.EncoderBuilder;
import frc.robot.utils.MotorBuilder;

public class DriveConstants {
  protected DriveConstants() {
  }

  /**
   * Information for left master drive [Port,controller type,
   * {invert,brake,connectionSaftey}]
   */
  public final MotorBuilder MOTOR_ID_LM = new MotorBuilder(1, MotorBuilder.Type.TalonSRX)
      .withSafety().withInvert();

  /** Drive left encoder builder */
  public final EncoderBuilder ENCODER_ID_L = new EncoderBuilder(new int[] { 19, 20 }, 60.078 / 256. / 1000);

  /**
   * Information for right master drive [Port,controller type,
   * {invert,brake,connectionSaftey}]
   */
  public final MotorBuilder MOTOR_ID_RM = new MotorBuilder(3, MotorBuilder.Type.TalonSRX)
      .withSafety();

  /** Drive left encoder builder */
  public final EncoderBuilder ENCODER_ID_R = new EncoderBuilder(new int[] { 14, 15 }, 59.883 / 256. / 1000)
      .withInvert();

  /**
   * Information for left slave drive [Port,controller type,
   * {invert,brake,connectionSaftey}]
   */
  public final MotorBuilder MOTOR_ID_LS = new MotorBuilder(2, MotorBuilder.Type.TalonSRX)
      .withSafety().withInvert();
  /**
   * Information for right slave drive [Port,controller type,
   * {invert,brake,connectionSaftey}]
   */
  public final MotorBuilder MOTOR_ID_RS = new MotorBuilder(4, MotorBuilder.Type.TalonSRX)
      .withSafety();

  /** KS value from SysId */
  public final double KS_VOLTS = 0.88881;
  /** KV value from SysId */
  public final double KV_VOLT_SECONDS_PER_METER = 3.0288;
  /** KA value from SysId */
  public final double KA_VOLT_SECONDS_SQUARED_PER_METER = 1.036;
  /** Horizontal distance between the drive wheels, in meters */
  public final double TRACK_WIDTH_METERS = 0.55;
  /** Drive kinematics */
  public final DifferentialDriveKinematics KINEMATICS = new DifferentialDriveKinematics(
      TRACK_WIDTH_METERS);
  /** Auto max velocity */
  public final double AUTO_MAX_SPEED = 2;
  /** Auto max acceleration */
  public final double AUTO_MAX_ACCELERATION = 1;
  /** Auto ramsete b variable */
  public final double RAMSETE_B = 2;
  /** Auto ramsete zeta variable */
  public final double RAMSETE_ZETA = 0.7;

  // Drive Settings
  /** max speed of robot m/s */
  public double MAX_SPEED = 3.850;
  /** min throttle for movement */
  public double MIN_THROT = 0.3;
  /** min throttle for turning */
  public double MIN_TURN = 0.3;
  /**
   * settings for robot drive in default teleop
   * {min throt,max throt,curve power}, {min turn throt, max turn throt,curve
   * power}
   */
  public double[][] PILOT_SETTINGS = { { MIN_THROT, 1, 2 }, { MIN_TURN, 1, 3, 0.3 } };
  /**
   * settings for robot drive in demo mode
   * {min throt,max throt,curve power}, {min turn throt, max turn throt,curve
   * power}
   */
  public double[][] DEMO_SETTINGS = { { MIN_THROT, 0.5, 3 }, { MIN_TURN, 0.6, 3, 0.1 } };

  /**
   * settings for copilot drive
   * {min throt,max throt,curve power}, {min turn throt, max turn throt,curve
   * power}
   */
  public double[][] COPILOT_SETTINGS = { { MIN_THROT, 0.5, 3 }, { MIN_TURN, 0.6, 3, 0.1 } };

  /**
   * settings for robot drive in PID drive
   * {min throt,max throt,curve power}, {min turn throt, max turn throt,curve
   * power}
   */
  public double[][] PID1_SETTINGS = { { 0, MAX_SPEED, 3 }, { 0, 1, 3, 0.3 } };
  /**
   * settings for robot drive in PID drive
   * {min throt,max throt,curve power}, {min turn throt, max turn throt,curve
   * power}
   */
  public double[][] PID2_SETTINGS = { { MIN_THROT, 1, 1 }, { MIN_TURN, 1, 1, 0.3 } };
}
