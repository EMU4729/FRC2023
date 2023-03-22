package frc.robot.constants;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import frc.robot.utils.MotorInfo;

public class DriveConstants {
    protected DriveConstants() {
    }

    /**
     * Information for left master drive [Port,controller type,
     * {invert,brake,connectionSaftey}]
     */
    public final MotorInfo MOTOR_ID_LM = new MotorInfo(1, MotorInfo.Type.TalonSRX)
            .withSafety().encoder(new int[] { 4, 5 }, 60.078 / 256. / 1000);
    /**
     * Information for right master drive [Port,controller type,
     * {invert,brake,connectionSaftey}]
     */
    public final MotorInfo MOTOR_ID_RM = new MotorInfo(3, MotorInfo.Type.TalonSRX)
            .withInvert().withSafety().encoder(new int[] { 6, 7 }, 59.883 / 256. / 1000);
    /**
     * Information for left slave drive [Port,controller type,
     * {invert,brake,connectionSaftey}]
     */
    public final MotorInfo MOTOR_ID_LS = new MotorInfo(2, MotorInfo.Type.TalonSRX)
            .withSafety();
    /**
     * Information for right slave drive [Port,controller type,
     * {invert,brake,connectionSaftey}]
     */
    public final MotorInfo MOTOR_ID_RS = new MotorInfo(4, MotorInfo.Type.TalonSRX)
            .withInvert().withSafety();

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
    public final double MAX_METERS_PER_SECOND = 2;
    /** Auto max acceleration */
    public final double MAX_ACCELERATION_METERS_PER_SECOND_SQUARED = 1;
    /** Auto ramsete b variable */
    public final double RAMSETE_B = 2;
    /** Auto ramsete zeta variable */
    public final double RAMSETE_ZETA = 0.7;
}
