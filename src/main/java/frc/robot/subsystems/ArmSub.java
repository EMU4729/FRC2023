package frc.robot.subsystems;

import java.io.FileWriter;
import java.io.IOException;
import java.time.Duration;
import java.time.Instant;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.OI;
import frc.robot.constants.Constants;
import frc.robot.shufflecontrol.ShuffleControl;
import frc.robot.utils.logger.Logger;

public class ArmSub extends SubsystemBase {
  private final WPI_VictorSPX seg1MasterMotor = (WPI_VictorSPX) Constants.arm.SEG1_MASTER_MOTOR_ID.build();
  private final WPI_VictorSPX seg1SlaveMotor = (WPI_VictorSPX) Constants.arm.SEG1_SLAVE_MOTOR_ID.build();

  private final WPI_TalonSRX seg2MasterMotor = (WPI_TalonSRX) Constants.arm.SEG2_MASTER_MOTOR_ID.build();
  private final WPI_TalonSRX seg2SlaveMotor = (WPI_TalonSRX) Constants.arm.SEG2_SLAVE_MOTOR_ID.build();

  private final MotorControllerGroup seg1Motors = new MotorControllerGroup(seg1MasterMotor, seg1SlaveMotor);
  private final MotorControllerGroup seg2Motors = new MotorControllerGroup(seg2MasterMotor, seg2SlaveMotor);

  private final Encoder seg1Encoder = Constants.arm.SEG1_ENCODER.build();
  private final Encoder seg2Encoder = Constants.arm.SEG2_ENCODER.build();

  private final PIDController integralSustainController = Constants.arm.INTEGRAL_SUSTAIN.build();

  private Instant lastUpdate = Instant.now();

  private boolean calibrated = false;

  private double seg1Output = 0;
  private double seg2Output = 0;

  private boolean angleIsControlled = false;

  private FileWriter logFile;

  public ArmSub() {
    if (RobotBase.isReal())
      try {
        logFile = new FileWriter("/home/lvuser/arm.csv");
        logFile.write(
            // Counts
            "seg1_counts," +
                "seg2_counts," +
                // Voltage
                "seg1_master_voltage," +
                "seg2_master_voltage," +
                "seg1_slave_voltage," +
                "seg2_slave_voltage," +
                // Current
                "seg1_master_current," +
                "seg2_master_current," +
                "seg1_slave_current," +
                "seg2_slave_current," +
                // Output
                "seg1_output," +
                "seg2_output," +
                // Angle
                "seg1_angle," +
                "seg2_angle," +
                // Angular Velocity
                "seg1_angular_velocity," +
                "seg2_angular_velocity," +
                // Kinematics Coords
                "kinematics_x," +
                "kinematics_y," +
                // Update Delta
                "update_delta\n");
      } catch (IOException e) {
        throw new RuntimeException("ArmSub: Error opening csv file: " + e.toString());
      }
  }

  private double getSeg1Angle() {
    return seg1Encoder.getDistance();
  }

  private double getSeg2Angle() {
    return seg2Encoder.getDistance() + getSeg1Angle();
  }

  /**
   * Updates the arm tab in shuffleboard. Call this function regularly.
   * 
   * @param armSeg1Output The speed of the upper arm motors
   * @param armSeg2Output The speed of the fore arm motors
   */
  private void updateShuffleboard() {
    ShuffleControl.calibrationTab.setArmCalibrated(calibrated);

    ShuffleControl.armTab.setOutputs(seg1Output, seg2Output);
    ShuffleControl.armTab.setEncoderAngles(getSeg1Angle(), getSeg2Angle());
    ShuffleControl.armTab.setEncoderCounts(seg1Encoder.get(), seg2Encoder.get());
    ShuffleControl.armTab.setEncoderRates(seg1Encoder.getRate(), seg2Encoder.getRate());
    ShuffleControl.armTab.setVoltages(seg1MasterMotor.getMotorOutputVoltage(), seg2MasterMotor.getMotorOutputVoltage());
    ShuffleControl.armTab.setCurrents(Constants.features.PDB.getCurrent(Constants.arm.SEG1_MASTER_MOTOR_ID.port),
        Constants.features.PDB.getCurrent(Constants.arm.SEG2_MASTER_MOTOR_ID.port));

    Translation2d kinematicsCoords = forK();
    ShuffleControl.armTab.setKinematicsCoords(kinematicsCoords.getX(), kinematicsCoords.getY());

    Instant nextUpdate = Instant.now();
    ShuffleControl.armTab.setUpdateDelta(Duration.between(lastUpdate, nextUpdate).toMillis());
    if (calibrated && RobotBase.isReal())
      try {
        logFile.append(
            // Encoder Counts
            seg1Encoder.get() + "," +
                seg2Encoder.get() + "," +
                // Voltage
                seg1MasterMotor.getMotorOutputVoltage() + "," +
                seg2MasterMotor.getMotorOutputVoltage() + "," +
                seg1SlaveMotor.getMotorOutputVoltage() + "," +
                seg2SlaveMotor.getMotorOutputVoltage() + "," +
                // Current
                Constants.features.PDB.getCurrent(Constants.arm.SEG1_MASTER_MOTOR_ID.port) +
                "," +
                Constants.features.PDB.getCurrent(Constants.arm.SEG2_MASTER_MOTOR_ID.port) +
                "," +
                Constants.features.PDB.getCurrent(Constants.arm.SEG1_SLAVE_MOTOR_ID.port) +
                "," +
                Constants.features.PDB.getCurrent(Constants.arm.SEG2_SLAVE_MOTOR_ID.port) +
                "," +
                // Output
                seg1Output + "," +
                seg2Output + "," +
                // Angles
                getSeg1Angle() + "," +
                getSeg2Angle() + "," +
                // Angular Velocity
                seg1Encoder.getRate() + "," +
                seg2Encoder.getRate() + "," +
                // Kinematics Coords
                kinematicsCoords.getX() + "," +
                kinematicsCoords.getY() + "," +
                // Update Delta
                Duration.between(lastUpdate, nextUpdate).toMillis() +
                "\n");
      } catch (IOException e) {
        Logger.warn("ArmSub : Error writing to arm csv : " + e.toString());
      }

    lastUpdate = nextUpdate;
  }

  /**
   * Forward kinematics.
   * 
   * @param armSeg1Angle The angle of the upper arm
   * @param armSeg2Angle The angle of the fore arm
   * @return The calculated coordinates of the end of the arm.
   */
  private Translation2d forK(double armSeg1Angle, double armSeg2Angle) {
    armSeg2Angle *= -1;
    double l1 = Constants.arm.SEG1_LENGTH;
    double l2 = Constants.arm.SEG2_LENGTH;

    double x1 = l1 * Math.cos(Math.toRadians(armSeg1Angle + 90));
    double y1 = l1 * Math.sin(Math.toRadians(armSeg1Angle + 90));

    double x2 = l2 * Math.cos(Math.toRadians(armSeg2Angle - 90)) + x1;
    double y2 = l2 * Math.sin(Math.toRadians(armSeg2Angle - 90)) + y1;

    return new Translation2d(x2, y2);
  }

  /**
   * Forward kinematics. When run without arguments, this uses the arms' encoder
   * values.
   * 
   * @return The calculated coordinates of the end of the arm.
   */
  private Translation2d forK() {
    return forK(getSeg1Angle(), getSeg2Angle());
  }

  /** @return The angle that the end of the arm makes with the robot horizontal */
  public double getEndAngle() {
    // How on earth does this geometrically work?
    return getSeg2Angle() - 90;
  }

  /**
   * Zeroes the upper and fore arm encoders.
   * Use this method when the upper arm pointing up and the fore arm is pointed
   * down.
   */
  public void calibrate() {
    seg1Encoder.reset();
    seg2Encoder.reset();
    calibrated = true;
    Logger.info("ArmSub : Calibrated!");
  }

  /** Kills the robot if an illegal arm angle is reached. */
  private void killCheck() {
    double armSeg1Angle = getSeg1Angle();
    double armSeg2Angle = getSeg2Angle();

    if (Math.abs(armSeg1Angle) > 90 || Math.abs(armSeg2Angle) > 185) {
      throw new IllegalStateException(
          String.format("ArmSub::killCheck : Illegal angles reached, killing robot! (armSeg2: %f, armSeg1: %f)",
              armSeg2Angle, armSeg1Angle));
    }
  }

  /**
   * Inverse cosine rule.
   * 
   * @param a Length of side a
   * @param b Length of side b
   * @param c Length of side c
   * @return The included angle C
   */
  private double invCosRule(double a, double b, double c) {
    double C = Math.acos(
        (Math.pow(a, 2) + Math.pow(b, 2) - Math.pow(c, 2))
            / (2 * a * b));
    return C;
  }

  @Override
  public void periodic() {
    if (!calibrated) {
      // Don't do anything if no calibration has happened.
      updateShuffleboard();
      return;
    }

    seg1Output = Constants.arm.SEG1_INPUT_CURVE
        .fit(OI.applyAxisDeadband(OI.copilot.getRawAxis(XboxController.Axis.kLeftX.value)));
    seg2Output = Constants.arm.SEG2_INPUT_CURVE
        .fit(OI.applyAxisDeadband(OI.copilot.getRawAxis(XboxController.Axis.kRightX.value)));

    if (seg1Output == 0 && seg2Output == 0) {
      if (angleIsControlled) {
        angleIsControlled = false;
        integralSustainController.setSetpoint(getSeg2Angle());
      }
    } else {
      angleIsControlled = true;
    }

    switch (Constants.arm.SUSTAIN_STRATEGY) {
      case CURVE:
        seg2Output += Constants.arm.SUSTAIN_CURVE.fit(MathUtil.applyDeadband(getSeg2Angle(), 10));

        if (Constants.arm.USE_INTEGRAL_SUSTAIN) {
          seg2Output += integralSustainController.calculate(getSeg2Angle());
        }

        break;
      case FEEDFORWARD:
        break;
      case NONE:
        break;
    }

    seg1Output = MathUtil.clamp(seg1Output, -0.4, 0.4);
    seg2Output = MathUtil.clamp(seg2Output, -0.4, 0.4);

    seg1Motors.set(seg1Output);
    seg2Motors.set(seg2Output);

    updateShuffleboard();
  }
}
