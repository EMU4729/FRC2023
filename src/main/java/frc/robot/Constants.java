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

        // Envars
        public final Map<String, String> ENV = System.getenv();

        // Drive
        /**
         * Information for left master drive [Port,controller type,
         * {invert,brake,connectionSaftey}]
         */
        public final MotorInfo DRIVE_MOTOR_ID_LM = new MotorInfo(1, MotorInfo.Type.TalonSRX)
                        .withSafety().encoder(new int[] { 0, 1 }, 60.078 / 256. / 1000);
        /**
         * Information for right master drive [Port,controller type,
         * {invert,brake,connectionSaftey}]
         */
        public final MotorInfo DRIVE_MOTOR_ID_RM = new MotorInfo(3, MotorInfo.Type.TalonSRX)
                        .withInvert().withSafety().encoder(new int[] { 2, 3 }, 59.883 / 256. / 1000);
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
        /** Path to the auto pathweaver json, relative to the robot deploy directory */
        public final String PATHWEAVER_PATH = "paths/GoToGameObject.wpilib.json";

        /** PID Constants for Charge Pad Balance Command */
        public final PIDControllerConstants BALANCE_CHARGE_PAD_PID = new PIDControllerConstants(0.1, 0, 0); // UPDATE
        /** Charge Pad Balance Angle Deadband */
        public final double BALANCE_CHARGE_PAD_DEADBAND = 2;

        // Gripper + Arm
        /** Information for Upper Arm Master Motor */
        public final MotorInfo UPPER_ARM_MASTER_MOTOR_ID = new MotorInfo(5, MotorInfo.Type.VictorSPX).withBrake()
                        .encoder(new int[] { 0, 1 }, 360. / 2048.); // @wip account for gearing ratios to the actual
                                                                    // arm
        /** Information for Upper Arm Slave Motor */
        public final MotorInfo UPPER_ARM_SLAVE_MOTOR_ID = new MotorInfo(6, MotorInfo.Type.VictorSPX).withBrake();
        /** Information for Fore Arm Master Motor */
        public final MotorInfo FORE_ARM_MASTER_MOTOR_ID = new MotorInfo(7, MotorInfo.Type.VictorSPX).withBrake()
                        .encoder(new int[] { 2, 3 }, 360. / 2048.); // @wip account for gearing ratios to the actual
                                                                    // arm
        /** Information for Fore Arm Slave Motor */
        public final MotorInfo FORE_ARM_SLAVE_MOTOR_ID = new MotorInfo(8, MotorInfo.Type.VictorSPX).withBrake();
        /** Length of the forearm, in metres @wip update arm length */
        public final double FORE_ARM_LENGTH = 1; // UPDATE
        /** Length of the upper arm, in metres @wip update arm length */
        public final double UPPER_ARM_LENGTH = 1; // UPDATE
        /** Velocity of the arm movements */
        public final double ARM_VELOCITY = 0.05;
        /** PID Constants for Upper Arm Movement @wip update constants */
        public final PIDControllerConstants UPPER_ARM_PID = new PIDControllerConstants(0.001, 0, 0.008); // UPDATE
        /** PID Constants for Fore Arm Movement */
        public final PIDControllerConstants FORE_ARM_PID = new PIDControllerConstants(0.001, 0, 0.008); // UPDATE
        /** Gripper Grip Servo 1 Channel @wip update servo port */
        public final int GRIPPER_GRIP_SERVO_1 = 0; // UPDATE
        /** Gripper Grip Servo 2 Channel @wip update servo port */
        public final int GRIPPER_GRIP_SERVO_2 = 1; // UPDATE

        // Subarm
        /** Subarm Pivot Servo Channel @wip update servo port */
        public final int SUBARM_PIVOT_SERVO = 2;
        /** Subarm Pivot Encoder Lower Limit */
        public final double SUBARM_PIVOT_LOWER_LIMIT = -5;
        /** Subarm Pivot Endoer Upper Bound */
        public final double SUBARM_PIVOT_UPPER_LIMIT = 180;
        /**
         * Information for Subarm Rotation Motor
         * 
         * @wip update everything but the encoder steps, that's fine
         */
        public final MotorInfo SUBARM_ROTATE_MOTOR_ID = new MotorInfo(9, MotorInfo.Type.TalonSRX)
                        .encoder(new int[] { -1, -1 }, 360. / 44.4 / 4.);

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

        // Robot features
        /** distance between wheel center side to side (m) */
        public final double ROBOT_WHEEL_WIDTH = 0.870;
        /** radius of the drive wheels (m) */
        public final double ROBOT_WHEEL_RAD = Units.inchesToMeters(3);

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
