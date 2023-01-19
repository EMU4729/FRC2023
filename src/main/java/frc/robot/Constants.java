// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Map;
import java.util.Optional;

import edu.wpi.first.math.util.Units;
import frc.robot.utils.MotorInfo;

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
        /** Length of the forearm, in metres */
        public final double FORE_ARM_LENGTH = 1; // UPDATE
        /** Length of the upper arm, in metres */
        public final double UPPER_ARM_LENGTH = 1; // UPDATE
        // TODO: Add gripper servos

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

        /** PID constants for throttle during teleop */
        public double[] TELEOP_THROTTLE_PID = { 0.2, 0, 0.8 }; // UPDATE
        /** PID constants for steering during teleop */
        public double[] TELEOP_STEERING_PID = { 0.2, 0, 0.8 }; // UPDATE
        /** Encoder max rate for PID loop */
        public double DRIVE_ENCODER_MAX_RATE = 1; // UPDATE

        /** LED string length (in leds) */
        public int LED_STRING_LENGTH = 60;
        /** LED string port num */
        public int LED_STRING_PORT = 0;
}
