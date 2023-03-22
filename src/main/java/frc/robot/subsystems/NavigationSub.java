package frc.robot.subsystems;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.ADIS16470_IMUSim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Subsystems;
import frc.robot.constants.Constants;
import frc.robot.shufflecontrol.ShuffleControl;
import frc.robot.utils.logger.Logger;

public class NavigationSub extends SubsystemBase {
  public final ADIS16470_IMU imu = new ADIS16470_IMU();
  private final DifferentialDriveOdometry odometry = new DifferentialDriveOdometry(
      Rotation2d.fromDegrees(imu.getAngle()),
      0., 0.);

  public final Encoder drvLeftEncoder = Constants.drive.ENCODER_ID_L.build();
  public final Encoder drvRightEncoder = Constants.drive.ENCODER_ID_R.build();

  public Field2d field = new Field2d();

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

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  // Simulation Variables

  private final ADIS16470_IMUSim imuSim = new ADIS16470_IMUSim(imu);
  private final EncoderSim drvLeftEncoderSim = new EncoderSim(drvLeftEncoder);
  private final EncoderSim drvRightEncoderSim = new EncoderSim(drvRightEncoder);

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  @Override
  public void periodic() {
    odometry.update(Rotation2d.fromDegrees(imu.getAngle()), drvLeftEncoder.getDistance(),
        drvRightEncoder.getDistance());
    updateShuffleboard();

    if (RobotController.getUserButton() && false) {
      Logger.info("Resetting Odometry (0,0,0)");
      imu.calibrate();
      resetOdometry(new Pose2d(0, 0, new Rotation2d(0)));
    }
  }

  public void updateShuffleboard() {
    ShuffleControl.navTab.setPose(getPose());
    ShuffleControl.navTab.setRotation(getHeadingDeg(), getPitch(), getRoll());
    ShuffleControl.navTab.setEncoderDistances(drvLeftEncoder.getDistance(), drvRightEncoder.getDistance());
  }

  /** @return The currently-estimated pose of the robot. */
  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }

  /** @return The wheel speeds of the robot */
  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(getLeftEncoderRate(), getRightEncoderRate());
  }

  /** @return direction the robot is facing in degrees */
  public double getHeadingDeg() {
    return getHeadingRot2d().getDegrees();
  }

  /** @return direction the robot is facing as a Rotation2d */
  public Rotation2d getHeadingRot2d() {
    return getPose().getRotation();
  }

  /** @return The roll angle of the robot */
  public double getRoll() {
    return imu.getXComplementaryAngle();

  }

  /** @return The pitch angle of the robot */
  public double getPitch() {
    return imu.getYComplementaryAngle();
  }

  /** Gets the left encoder rate. @return The speed. m/s */
  public double getLeftEncoderRate() {
    return drvLeftEncoder.getRate();
  }

  /** Gets the right encoder rate. @return The speed. m/s */
  public double getRightEncoderRate() {
    return drvRightEncoder.getRate();
  }

  /** Resets the drive base encoders. */
  public void resetEncoders() {
    drvLeftEncoder.reset();
    drvRightEncoder.reset();
  }

  /** @return the reported drive encoder distances */
  public Pair<Double, Double> getEncoderDistances() {
    return new Pair<Double, Double>(drvLeftEncoder.getDistance(), drvRightEncoder.getDistance());
  }

  /** @return the average reported drive encoder distance */
  public double getAvgEncoderDistance() {
    Pair<Double, Double> encoderDistances = getEncoderDistances();
    return (encoderDistances.getFirst() + encoderDistances.getSecond()) / 2.0;
  }

  /**
   * Resets the odometry to the specified pose.
   * 
   * @param pose The pose.
   */
  public void resetOdometry(Pose2d pose) {
    resetEncoders();
    imu.reset();
    odometry.resetPosition(
        Rotation2d.fromDegrees(imu.getAngle()),
        0., 0., pose);

    // sim
    Subsystems.drive.drivetrainSimulator.setPose(pose);
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  // Simulation Functions

  @Override
  public void simulationPeriodic() {
    DifferentialDrivetrainSim drvTrnSim = Subsystems.drive.drivetrainSimulator;

    drvLeftEncoderSim.setDistance(drvTrnSim.getLeftPositionMeters());
    drvLeftEncoderSim.setRate(drvTrnSim.getLeftVelocityMetersPerSecond());

    drvRightEncoderSim.setDistance(drvTrnSim.getRightPositionMeters());
    drvRightEncoderSim.setRate(drvTrnSim.getRightVelocityMetersPerSecond());

    imuSim.setGyroAngleZ(drvTrnSim.getHeading().getDegrees());

  }
}
