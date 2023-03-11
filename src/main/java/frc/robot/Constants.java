// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Map;
import java.util.Optional;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
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

    private Constants() {
    }

    // Robot features
    /** distance between wheel center side to side (m) */
    public final double ROBOT_WHEEL_WIDTH = 0.870;
    /** radius of the drive wheels (m) */
    public final double ROBOT_WHEEL_RAD = Units.inchesToMeters(3);
    /** length of the robot frame (m) */
    public final double ROBOT_LENGTH = Units.inchesToMeters(28.3);
    /** width of the robot frame (m) @wip check num */
    public final double ROBOT_WIDTH = Units.inchesToMeters(24);
    /** max reach outside frame perim [x(from frame),y(from floor)] (m) */
    public final double[] ROBOT_REACH_MAX = { (Units.inchesToMeters(48)),
            (Units.inchesToMeters(78)) };

    // Envars
    public final Map<String, String> ENV = System.getenv();

    // Drive
    /**
     * Information for left master drive [Port,controller type,
     * {invert,brake,connectionSaftey}]
     */
    public final MotorInfo DRIVE_MOTOR_ID_LM = new MotorInfo(1, MotorInfo.Type.TalonSRX)
            .withSafety().encoder(new int[] { 4, 5 }, 60.078 / 256. / 1000);
    /**
     * Information for right master drive [Port,controller type,
     * {invert,brake,connectionSaftey}]
     */
    public final MotorInfo DRIVE_MOTOR_ID_RM = new MotorInfo(3, MotorInfo.Type.TalonSRX)
            .withInvert().withSafety().encoder(new int[] { 6, 7 }, 59.883 / 256. / 1000);
    /**
     * Information for left slave drive [Port,controller type,
     * {invert,brake,connectionSaftey}]
     */
    public final MotorInfo DRIVE_MOTOR_ID_LS = new MotorInfo(2, MotorInfo.Type.TalonSRX)
            .withSafety();
    /**
     * Information for right slave drive [Port,controller type,
     * {invert,brake,connectionSaftey}]
     */
    public final MotorInfo DRIVE_MOTOR_ID_RS = new MotorInfo(4, MotorInfo.Type.TalonSRX)
            .withInvert().withSafety();

    /** KS value from SysId */
    public final double DRIVE_KS_VOLTS = 0.88881;
    /** KV value from SysId */
    public final double DRIVE_KV_VOLT_SECONDS_PER_METER = 3.0288;
    /** KA value from SysId */
    public final double DRIVE_KA_VOLT_SECONDS_SQUARED_PER_METER = 1.036;
    /** Horizontal distance between the drive wheels, in meters */
    public final double DRIVE_TRACK_WIDTH_METERS = 0.55;
    /** Drive kinematics */
    public final DifferentialDriveKinematics DRIVE_KINEMATICS = new DifferentialDriveKinematics(
            DRIVE_TRACK_WIDTH_METERS);
    /** Auto max velocity */
    public final double DRIVE_MAX_METERS_PER_SECOND = 2;
    /** Auto max acceleration */
    public final double DRIVE_MAX_ACCELERATION_METERS_PER_SECOND_SQUARED = 1;
    /** Auto ramsete b variable */
    public final double DRIVE_RAMSETE_B = 2;
    /** Auto ramsete zeta variable */
    public final double DRIVE_RAMSETE_ZETA = 0.7;

    /** PID Constants for Charge Pad Balance Command */
    public final PIDControllerConstants BALANCE_CHARGE_PAD_PID = new PIDControllerConstants(0.1, 0, 0); // UPDATE
    /** Charge Pad Balance Angle Deadband */
    public final double BALANCE_CHARGE_PAD_DEADBAND = 2;

    // Gripper + Arm
    /** Information for Arm Seg1 Master Motor */
    public final MotorInfo ARM_SEG1_MASTER_MOTOR_ID = new MotorInfo(7, MotorInfo.Type.VictorSPX).withBrake()
            .encoder(new int[] { 10, 11 }, 360. / 16 / 2048.); // @wip account for gearing ratios to the actual
    // arm
    /** Information for Arm Seg1 Slave Motor */
    public final MotorInfo ARM_SEG1_SLAVE_MOTOR_ID = new MotorInfo(8, MotorInfo.Type.VictorSPX).withBrake();
    /** Information for Arm Seg2 Master Motor */
    public final MotorInfo ARM_SEG2_MASTER_MOTOR_ID = new MotorInfo(5, MotorInfo.Type.VictorSPX).withBrake()
            .encoder(new int[] { 8, 9 }, 360. / 4 / 2048.); // @wip account for gearing ratios to the actual
    // arm
    /** Information for Arm Seg2 Slave Motor */
    public final MotorInfo ARM_SEG2_SLAVE_MOTOR_ID = new MotorInfo(6, MotorInfo.Type.VictorSPX).withBrake();
    /**
     * Length of the armseg2 (second segment)(between axles), in (mm) @wip update
     * arm length
     */ // wip
    public final double ARM_SEG2_LENGTH = 0.8; // UPDATE
    /**
     * Length of the arm seg1 (first segment)(between axles), in (mm) @wip update
     * arm length
     */ // wip
    public final double ARM_SEG1_LENGTH = 0.91; // UPDATE
    /** height off the carpet of the seg1 rm axle (m) @wip value just a guess */ // wip
    public final double ARM_SEG1_AXLE_HEIGHT = 0.2;
    /**
     * amount the arm seg1 axle is offset from the centerline in x (m)(forward
     * pos) @wip value guessed
     */ // wip
    public final double ARM_SEG1_X_OFFSET = 0.01;
    /** Velocity of the arm movements */
    public final double ARM_VELOCITY = 0.002;
    /** Interpolation step of the arm */
    public final double ARM_INTERPOLATION_STEP = 0.05;
    /** PID Constants for Arm Seg1 Movement @wip update constants */ // wip
    public final PIDControllerConstants ARM_SEG1_PID = new PIDControllerConstants(0.03, 0.01, 0); // UPDATE
    /** PID Constants for Arm Seg2 Movement */ // wip
    public final PIDControllerConstants ARM_SEG2_PID = new PIDControllerConstants(0.02, 0.007, 0); // UPDATE
    /** Gripper Grip Servo 1 Channel @wip update servo port */ // wip
    public final int GRIPPER_GRIP_SERVOS_ID = 1;
    /**
     * distance in x and y from the arm seg1 axle to the max distance the robot is
     * allowed to reach
     * [[x- (behind), x+(in front)], [y-(below), y+(above)]] (m)
     */
    public final double[][] MAX_ARM_REACH_LEGAL = {
            { -(ROBOT_LENGTH / 2 + ROBOT_REACH_MAX[0] + ARM_SEG1_X_OFFSET),
                    ROBOT_LENGTH / 2 + ROBOT_REACH_MAX[0] - ARM_SEG1_X_OFFSET },
            { -ARM_SEG1_AXLE_HEIGHT, ROBOT_REACH_MAX[1] - ARM_SEG1_AXLE_HEIGHT } };
    /**
     * length of the combined 2 segment arm, ARM_SEG1_X_OFFSET should be subtracted
     * to find true x (mm)
     */
    public final double MAX_ARM_REACH_PHYSICAL = ARM_SEG1_LENGTH + ARM_SEG2_LENGTH;
    /**
     * area the arm should never enter (mm)[[x-(behind), x+(in front)], [y (from
     * floor)]] @wip needs right y
     */ // wip
    public final double[][] ARM_REACH_EXCLUSION = {
            { -(ROBOT_LENGTH / 2 - ARM_SEG1_X_OFFSET), (ROBOT_LENGTH / 2 + ARM_SEG1_X_OFFSET), },
            { 0.200 } };
    /** Dimensions (width, height) of the robot that the arm should never reach. */
    public final double[][] ARM_REACH_ROBOT_EXCLUSION = {
            { -(ROBOT_LENGTH / 2), ROBOT_LENGTH / 2 },
            { -0.15, 0 }
    };
    /**
     * height the arm should seek to hold if moving or stored inside frame perimiter
     */
    public final double ARM_SWING_THROUGH_HEIGHT = ARM_SEG1_LENGTH - ARM_SEG2_LENGTH;

    // Subarm
    /** Subarm Rotation Servo Channel */
    public final int SUBARM_ROTATE_SERVO = 0;
    /** Subarm Rotation Encoder Lower Limit */
    public final double SUBARM_ROTATE_LOWER_LIMIT = -5;
    /** Subarm Rotation Encoder Upper Bound */
    public final double SUBARM_ROTATE_UPPER_LIMIT = 180;
    /**
     * Subarm Rotation Encoder Info. <strong>Do not try to create a motor controller
     * with this.</strong>
     */
    public final MotorInfo SUBARM_ROTATE_ENCODER_INFO = new MotorInfo(-1, MotorInfo.Type.Never)
            .encoder(new int[] { 2, 3 }, 1. / 20.);
    /**
     * Information for Subarm Pivot Motor
     */
    public final MotorInfo SUBARM_PIVOT_MOTOR_ID = new MotorInfo(9, MotorInfo.Type.TalonSRX)
            .encoder(new int[] { 0, 1 }, 360. / 44.4 / 4.);
    /** PID Constants for the Subarm Pivot */
    public final PIDControllerConstants SUBARM_PIVOT_PID = new PIDControllerConstants(0.05, 0, 0);
    /** Subarm Pivot Velocity (degrees per tick) */
    public final double SUBARM_PIVOT_VELOCITY = 1;
    /** Subarm Pivot Encoder lower limit */
    public final double SUBARM_PIVOT_LOWER_LIMIT = 0;
    /** Subarm Pivot Encoder upper limit */
    public final double SUBARM_PIVOT_UPPER_LIMIT = 180;

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
    public final int LOGGER_SAVE_RATE = 10;

    // LEDs
    /** LED string length (in leds) */
    public final int LED_STRING_LENGTH = 42;
    /** LED string port num */
    public final int LED_STRING_PORT = 9;
    /**
     * Max number of colour changes/s (red -> black -> red -> black = 4) for leds
     */
    public final int LED_MAX_FLASH_RATE = 8;
    /**  */
    public final int[] LED_ZONES = { 0, 3, 39, 42 };
}
