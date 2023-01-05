// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Map;
import java.util.Optional;

import frc.robot.utils.MotorInfo;

/**
 * Constants - use this class to store any port ids, file paths, or basically
 * anything that will not change.
 */
public final class Constants {
  private static Optional<Constants> inst = Optional.empty();

  private Constants() {
  }

  public static Constants getInstance() {
    if (!inst.isPresent()) {
      inst = Optional.of(new Constants());
    }
    return inst.get();
  }

  // Envars
  public final Map<String, String> ENV = System.getenv();

  // Drive
  /**
   * Information for left master drive [Port,controller type,
   * {invert,brake,connectionSaftey}]
   */
  public final MotorInfo DRIVE_MOTOR_ID_LM = new MotorInfo(4, MotorInfo.Type.TalonSRX)
      .withSafety().encoder(new int[] { 0, 1 }, 60.078 / 256. / 1000);
  /**
   * Information for right master drive [Port,controller type,
   * {invert,brake,connectionSaftey}]
   */
  public final MotorInfo DRIVE_MOTOR_ID_RM = new MotorInfo(1, MotorInfo.Type.TalonSRX)
      .withInvert().withSafety().encoder(new int[] { 2, 3 }, 59.883 / 256. / 1000);
  /**
   * Information for left slave drive [Port,controller type,
   * {invert,brake,connectionSaftey}]
   */
  public final MotorInfo DRIVE_MOTOR_ID_LS = new MotorInfo(5, MotorInfo.Type.TalonSRX)
      .withSafety();
  /**
   * Information for right slave drive [Port,controller type,
   * {invert,brake,connectionSaftey}]
   */
  public final MotorInfo DRIVE_MOTOR_ID_RS = new MotorInfo(2, MotorInfo.Type.TalonSRX)
      .withInvert().withSafety();

  // Turret
  /** Limit switch for turret slew angle */
  public final int TURRET_SLEW_LIMIT = 4;
  /** Limit switch for turret slew angle */
  public final int TURRET_HOOD_LIMIT = -1;
  /**
   * Information for turret slew motor [Port,controller type,
   * {invert,brake,connectionSaftey}, Encoder]
   */
  public final MotorInfo TURRET_SLEW_MOTOR_ID = new MotorInfo(7, MotorInfo.Type.VictorSPX).withBrake()
      .encoder(new int[] { 8, 9 }, 3. / 4.);
  /**
   * Information for turret slew motor [Port,controller type,
   * {invert,brake,connectionSaftey}, Encoder]
   */
  public final MotorInfo TURRET_HOOD_MOTOR_ID = new MotorInfo(-1, MotorInfo.Type.Never).withBrake()
      .encoder(new int[] { -1, -1 }, -1);
  /** min max degree range for turret slew */
  public final double[] TURRET_SLEW_RANGE = { 0, 270 };
  /** min max degree range for turret hood */
  public final double[] TURRET_HOOD_RANGE = { 0, 300 };
  /** PID Constants for turret slew movement */
  public final double[] TURRET_SLEW_PID = { 0.01, 0.0015, 0 };

  // Test
  /**
   * Information for turret test motor [Port,controller type,
   * {invert,brake,connectionSaftey}, Encoder]
   */
  public final MotorInfo TEST_1_MOTOR_ID = new MotorInfo(6, MotorInfo.Type.VictorSPX).withBrake();
  /**
   * Information for turret test motor [Port,controller type,
   * {invert,brake,connectionSaftey}, Encoder]
   */
  public final MotorInfo TEST_2_MOTOR_ID = new MotorInfo(7, MotorInfo.Type.VictorSPX).withBrake();

  // Controller
  /** Port Number for xbox controller input device */
  public final int DEVICE_PORT_XBOX_CONTROLLER = 0; // WORKING
  /** Threshold for triggering the controller right and left triggers */
  public final double CONTROLLER_TRIGGER_THRESHOLD = 0.5;
  /** deadband for controller axies either side of 0 */
  public final double CONTROLLER_AXIS_DEADZONE = 0.1;

  // File Paths
  /** file path header for files on usb storage */
  public final String[] PATH_USB = { "u//", "v//" };
  /** file path header for files on internal storage */
  public final String PATH_INTERNAL = ENV.get("HOME");
  /** file name for default auto @deprecated */
  public final String PATH_AUTO_FILE_NAME = "autoCommands.txt";

  // Error Retry Limit
  /** limit for repeated attempts to create log file on USB storage */
  public final int REPEAT_LIMIT_LOGGER_CREATION = 10;
  /** limit for repeated attempts to read auto from internal storage */
  public final int REPEAT_LIMIT_AUTO_READ = 10;
  /** save attempts per second for the logger */
  public int LOGGER_SAVE_RATE = 10;

  // Robot features
  /** distance between wheel center side to side (m) */
  public final double ROBOT_WHEEL_WIDTH = 0.870;

  /** Auto straight PID Constants */
  public double[] AUTO_STRAIGHT_PID = { 0.2, 0.8, 0 }; // UPDATE

  /** PID constants for throttle during teleop */
  public double[] TELEOP_THROTTLE_PID = { 0.2, 0, 0.8 }; // UPDATE
  /** PID constants for steering during teleop */
  public double[] TELEOP_STEERING_PID = { 0.2, 0, 0.8 }; // UPDATE
  /** Encoder max rate for PID loop */
  public double DRIVE_ENCODER_MAX_RATE = 1; // UPDATE
}
