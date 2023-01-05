package frc.robot.subsystems;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.ShuffleControl.ShuffleControl;
import frc.robot.utils.logger.Logger;

public class NavigationSub extends SubsystemBase {
  private final Constants cnst = Constants.getInstance();

  public final ADIS16470_IMU imu = new ADIS16470_IMU();
  private final DifferentialDriveOdometry odometry = new DifferentialDriveOdometry(
      Rotation2d.fromDegrees(imu.getAngle()));

  public final Encoder drvLeftEncoder = cnst.DRIVE_MOTOR_ID_LM.createEncoder();
  public final Encoder drvRightEncoder = cnst.DRIVE_MOTOR_ID_RM.createEncoder();

  /** yaw offset at zeroing relitive to the field zero (deg) */
  private double yawOffset = 0;

  /** m from start pos in x rel to start angle @WIP not implimented */
  public double xPos = 0;
  /** m from start pos in y rel to start angle @WIP not implimented */
  public double yPos = 0;
  /** rotation rel to start in deg */
  public double rot = 0;
  /** m/s of speed */
  public double speed = 0;
  /** deg/s of rotation (CW = pos) */
  public double turn = 0;

  int n = 0;

  @Override
  public void periodic() {
    odometry.update(Rotation2d.fromDegrees(imu.getAngle()), drvLeftEncoder.getDistance(),
        drvRightEncoder.getDistance());
    ShuffleControl.field.setRobotPose(odometry.getPoseMeters());

    

    if(RobotController.getUserButton()){
      Logger.info("Resetting Odometry (0,0,0)");
      imu.calibrate();
      resetOdometry(new Pose2d(0,0, new Rotation2d(0)));
    }
  }

  /**
   * Returns the currently-estimated pose of the robot.
   * 
   * @return The pose.
   */
  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }

  /**
   * 
   * @return direction in 
   */
  public double getHeadingDeg(){
    return imu.getAngle() + yawOffset;
  }
  public Rotation2d getHeadingRot2d(){
    return Rotation2d.fromDegrees(getHeadingDeg());
  }

  /** Gets the left encoder rate. @return The speed. m/s */
  public double getLeftEncoderRate() {
    return drvLeftEncoder.getRate();
  }

  /** Gets the right encoder rate. @return The speed. m/s */
  public double getRightEncoderRate() {
    return drvRightEncoder.getRate();
  }

  /** Gets the average encoder rate. @return speed of COM. m/s */
  private double getCOMSpeed() {
    return (getLeftEncoderRate() + getRightEncoderRate()) / 2;
  }

  /** gets the average turn rate. @return rate of turn. deg/s */
  private double getTurnRate() {
    double arcL = drvRightEncoder.getDistance() - drvLeftEncoder.getDistance();

    return (arcL / (Math.PI * cnst.ROBOT_WHEEL_WIDTH)) * 360;

    //if (vL == vR) {
    //  double centRad = (vL * cnst.ROBOT_WHEEL_WIDTH) / (vR - vL) + 0.5 * cnst.ROBOT_WHEEL_WIDTH;
    //  return centRad == 0 ? 0 : Math.toDegrees(getCOMSpeed() / centRad);
    //}
    //return 0;
  }

  public void resetEncoders() {
    drvLeftEncoder.reset();
    drvRightEncoder.reset();
  }

  /**
   * Resets the odometry to the specified pose.
   * 
   * @param pose The pose.
   */
  public void resetOdometry(Pose2d pose) {
    resetEncoders();
    imu.reset();
    odometry.resetPosition(pose, getHeadingRot2d());
  }
}
