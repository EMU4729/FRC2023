// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Map;
import java.util.Optional;

import edu.wpi.first.math.util.Units;
import frc.robot.utils.MotorInfo;
import frc.robot.utils.PIDControllerConstants;

/**
 * Constants - use this class to store any port ids, file paths, or basically
 * anything that will not change.
 */
public final class Constants {
  private static Optional<Constants> inst = Optional.empty();
  public static Constants getInstance() {
          if (!inst.isPresent())
                  inst = Optional.of(new Constants());
          return inst.get();
  }

  private Constants(){
    if(checkLEDs()) throw new IllegalArgumentException("num of leds or segments sizes is wrong");
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

  // Gripper + Arm
  /** Information for Upper Arm Motor */
  public final MotorInfo UPPER_ARM_MOTOR_ID = new MotorInfo(-1, MotorInfo.Type.Never).withBrake();
  /** Information for Fore Arm Motor */
  public final MotorInfo FORE_ARM_MOTOR_ID = new MotorInfo(-1, MotorInfo.Type.Never).withBrake();
  /** Length of the forearm, in metres @wip update arm length*/
  public final double FORE_ARM_LENGTH = 1;                                                          // UPDATE
  /** Length of the upper arm, in metres @wip update arm length*/
  public final double UPPER_ARM_LENGTH = 1;                                                         // UPDATE
  /** PID Constants for Upper Arm Movement @wip update constants*/
  public final PIDControllerConstants UPPER_ARM_PID = new PIDControllerConstants(0.2, 0, 0.8);     // UPDATE
  /** PID Constants for Fore Arm Movement */
  public final PIDControllerConstants FORE_ARM_PID = new PIDControllerConstants(0.2, 0, 0.8);      // UPDATE
  /** Gripper Grip Servo 1 Channel @wip update servo port*/
  public final int GRIPPER_GRIP_SERVO_1 = 0;                                                        // UPDATE
  /** Gripper Grip Servo 2 Channel @wip update servo port*/
  public final int GRIPPER_GRIP_SERVO_2 = 1;                                                        // UPDATE
  // TODO: Add gripper pivot servo(s)

  // Controllers
  /** Port Number for Pilot Xbox Controller */
  public final int PILOT_XBOX_CONTROLLER_PORT = 0; // WORKING
  /** Port Number for Copilot Xbox Controller */
  public final int COPILOT_XBOX_CONTROLLER_PORT = 1; // TEST
  /** Threshold for triggering the controller right and left triggers */
  public final double CONTROLLER_TRIGGER_THRESHOLD = 0.5;
  /** deadband for controller axies either side of 0 */
  public final double CONTROLLER_AXIS_DEADZONE = 0.1;

  // File Paths
  /** file path header for files on usb storage */
  public final String[] PATH_USB = { "u//", "v//" };
  /** file path header for files on internal storage */
  public final String PATH_INTERNAL = ENV.get("HOME");

  // Logger
  /** limit for repeated attempts to create log file on USB storage */
  public final int REPEAT_LIMIT_LOGGER_CREATION = 10;
  /** limit for repeated attempts to read auto from internal storage */
  public final int REPEAT_LIMIT_AUTO_READ = 10;
  /** save attempts per second for the logger */
  public int LOGGER_SAVE_RATE = 10;

  // Robot features
  /** distance between wheel center side to side (m) */
  public final double ROBOT_WHEEL_WIDTH = 0.870;
  /** radius of the drive wheels (m) */
  public final double ROBOT_WHEEL_RAD = Units.inchesToMeters(3);

  //LEDs
  /** LED string length (in leds) */
  public final int LED_STRING_LENGTH = 42;
  /** LED string port num */
  public final int LED_STRING_PORT = 9;
  /** Max number of colour changes/s (red -> black -> red -> black = 4) for leds */
  public final int LED_MAX_FLASH_RATE = 8;
  /**  */
  public final int[] LED_ZONE_SIZES = {3,36,3};
  /** number of zones currently on the robot */
  public final int LED_ZONES = LED_ZONE_SIZES.length;

  private boolean checkLEDs(){
    int tmp = 0;
    for(int i = 0; i < LED_ZONES; i++) tmp += LED_ZONE_SIZES[i];
    return tmp == LED_STRING_LENGTH;
  }
}
