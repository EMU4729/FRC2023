package frc.robot.subsystems;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;

import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.simulation.ADIS16470_IMUSim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Subsystems;
import frc.robot.constants.Constants;
import frc.robot.shufflecontrol.ShuffleControl;
import frc.robot.utils.PhotonBridge;

/** Subsystem that handles all robot navigation */
public class NavigationSub extends SubsystemBase {
  public final ADIS16470_IMU imu = new ADIS16470_IMU();

  private final Encoder drvLeftEncoder = Constants.drive.ENCODER_ID_L.build();
  private final Encoder drvRightEncoder = Constants.drive.ENCODER_ID_R.build();

  private final Field2d field = new Field2d();
  private final PhotonBridge photon = new PhotonBridge();
  private final DifferentialDrivePoseEstimator poseEstimator = new DifferentialDrivePoseEstimator(
      Constants.drive.KINEMATICS, getHeadingRot2d(), 0, 0, new Pose2d());

  // Simulation Variables
  private final ADIS16470_IMUSim imuSim = new ADIS16470_IMUSim(imu);
  private final EncoderSim drvLeftEncoderSim = new EncoderSim(drvLeftEncoder);
  private final EncoderSim drvRightEncoderSim = new EncoderSim(drvRightEncoder);

  public NavigationSub() {
    SmartDashboard.putData("Field", field);
  }

  @Override
  public void periodic() {
    updateOdometry();
    updateShuffleboard();
  }

  private void updateOdometry() {
    poseEstimator.update(
        Rotation2d.fromDegrees(imu.getAngle()), drvLeftEncoder.getDistance(), drvRightEncoder.getDistance());

    Optional<EstimatedRobotPose> result = photon.getEstimatedGlobalPose(poseEstimator.getEstimatedPosition());

    if (result.isPresent()) {
      EstimatedRobotPose camPose = result.get();
      poseEstimator.addVisionMeasurement(
          camPose.estimatedPose.toPose2d(), camPose.timestampSeconds);
      field.getObject("Cam Est Pos").setPose(camPose.estimatedPose.toPose2d());
    } else {
      // move it way off the screen to make it disappear
      field.getObject("Cam Est Pos").setPose(new Pose2d(-100, -100, new Rotation2d()));
    }

    field.getObject("Actual Pos").setPose(Subsystems.drive.drivetrainSimulator.getPose());
    field.setRobotPose(poseEstimator.getEstimatedPosition());
  }

  public void updateShuffleboard() {
    ShuffleControl.navTab.setRotation(getHeadingDeg(), getPitch(), getRoll());
    ShuffleControl.navTab.setEncoderDistances(
        drvLeftEncoder.getDistance(),
        drvRightEncoder.getDistance());
  }

  /** @return The currently-estimated pose of the robot. */
  public Pose2d getPose() {
    // Not sure why, but poseEstimator is null for the first few seconds of runtime.
    // Too bad!
    if (poseEstimator == null)
      return new Pose2d();

    return poseEstimator.getEstimatedPosition();
  }

  /** @return The wheel speeds of the robot */
  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(
        getLeftEncoderRate(),
        getRightEncoderRate());
  }

  /** @return direction the robot is facing in degrees */
  public double getHeadingDeg() {
    return getHeadingRot2d().getDegrees();
  }

  /** @return direction the robot is facing as a Rotation2d */
  public Rotation2d getHeadingRot2d() {
    return getPose().getRotation();
  }

  /** @return The yaw angle of the robot in degrees */
  public double getYaw() {
    return imu.getAngle();
  }

  /** @return The roll angle of the robot in degrees */
  public double getRoll() {
    return imu.getXComplementaryAngle();
  }

  /** @return The pitch angle of the robot in degrees */
  public double getPitch() {
    return imu.getYComplementaryAngle();
  }

  /** Gets the left encoder rate. @return The speed in m/s */
  public double getLeftEncoderRate() {
    return drvLeftEncoder.getRate();
  }

  /** Gets the right encoder rate. @return The speed in m/s */
  public double getRightEncoderRate() {
    return drvRightEncoder.getRate();
  }

  /** Resets the drive base encoders. */
  public void resetEncoders() {
    drvLeftEncoder.reset();
    drvRightEncoder.reset();
  }

  /** @return the reported drive encoder distances */
  public Translation2d getEncoderDistances() {
    return new Translation2d(drvLeftEncoder.getDistance(), drvRightEncoder.getDistance());
  }

  /** @return the average reported drive encoder distance */
  public double getAvgEncoderDistance() {
    Translation2d encoderDistances = getEncoderDistances();
    return (encoderDistances.getX() + encoderDistances.getY()) / 2.0;
  }

  /**
   * Resets the odometry to the specified pose.
   * 
   * @param pose The pose.
   */
  public void resetOdometry(Pose2d pose) {
    resetEncoders();
    imu.reset();
    poseEstimator.resetPosition(
        Rotation2d.fromDegrees(imu.getAngle()),
        0., 0., pose);

    // sim
    Subsystems.drive.drivetrainSimulator.setPose(pose);
  }

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
