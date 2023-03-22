package frc.robot.constants;

import frc.robot.utils.MotorInfo;
import frc.robot.utils.PIDControllerBuilder;

public class ArmConstants {
        protected ArmConstants() {
        }

        /** Information for Arm Seg1 Master Motor */
        public final MotorInfo SEG1_MASTER_MOTOR_ID = new MotorInfo(7, MotorInfo.Type.VictorSPX).withBrake()
                        .encoder(new int[] { 10, 11 }, 360. / 16 / 2048.); // @wip account for gearing ratios to the
                                                                           // actual
        // arm
        /** Information for Arm Seg1 Slave Motor */
        public final MotorInfo SEG1_SLAVE_MOTOR_ID = new MotorInfo(8, MotorInfo.Type.VictorSPX).withBrake();
        /** Information for Arm Seg2 Master Motor */
        public final MotorInfo SEG2_MASTER_MOTOR_ID = new MotorInfo(5, MotorInfo.Type.VictorSPX).withBrake()
                        .encoder(new int[] { 8, 9 }, 360. / 4 / 2048.); // @wip account for gearing ratios to the actual
        // arm
        /** Information for Arm Seg2 Slave Motor */
        public final MotorInfo SEG2_SLAVE_MOTOR_ID = new MotorInfo(6, MotorInfo.Type.VictorSPX).withBrake();
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
        /** Velocity of the arm movements */
        public final double VELOCITY = 0.002;
        /** Interpolation step of the arm */
        public final double INTERPOLATION_STEP = 0.05;
        /** PID Constants for Arm Seg1 Movement @wip update constants */ // wip
        public final PIDControllerBuilder SEG1_PID = new PIDControllerBuilder(0.03, 0.01, 0); // UPDATE
        /** PID Constants for Arm Seg2 Movement */ // wip
        public final PIDControllerBuilder SEG2_PID = new PIDControllerBuilder(0.01, 0.007, 0); // UPDATE
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
}
