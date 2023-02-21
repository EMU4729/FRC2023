package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.shufflecontrol.ShuffleControl;
import frc.robot.utils.logger.Logger;
import edu.wpi.first.math.Pair;

public class ArmSub extends SubsystemBase {
  private final Constants cnst = Constants.getInstance();

  private final PIDController upperArmController = cnst.UPPER_ARM_PID.createPIDController();
  private final PIDController foreArmController = cnst.FORE_ARM_PID.createPIDController();

  private final MotorController upperArmMasterMotor = cnst.UPPER_ARM_MASTER_MOTOR_ID.createMotorController();
  private final MotorController upperArmSlaveMotor = cnst.UPPER_ARM_SLAVE_MOTOR_ID.createMotorController();

  private final MotorController foreArmMasterMotor = cnst.FORE_ARM_MASTER_MOTOR_ID.createMotorController();
  private final MotorController foreArmSlaveMotor = cnst.FORE_ARM_SLAVE_MOTOR_ID.createMotorController();

  private final MotorControllerGroup upperArmMotors = new MotorControllerGroup(upperArmMasterMotor, upperArmSlaveMotor);
  private final MotorControllerGroup foreArmMotors = new MotorControllerGroup(foreArmMasterMotor, foreArmSlaveMotor);

  private final Encoder upperArmEncoder = cnst.UPPER_ARM_MASTER_MOTOR_ID.createEncoder();
  private final Encoder foreArmEncoder = cnst.FORE_ARM_MASTER_MOTOR_ID.createEncoder();

  private boolean invert = false;
  private boolean calibrated = false;

  private List<Pair<Double, Double>> targets = new ArrayList<Pair<Double, Double>>();
  private double upperArmTargetAngle;
  private double foreArmTargetAngle;

  public ArmSub() {
    upperArmController.setTolerance(3);
    foreArmController.setTolerance(3);
  }

  /**
   * Updates the arm tab in shuffleboard. Call this function regularly.
   * 
   * @param upperArmOutput The speed of the upper arm motors
   * @param foreArmOutput  The speed of the fore arm motors
   */
  private void updateShuffleboard(double upperArmOutput, double foreArmOutput) {
    ShuffleControl.armTab.setOutputs(upperArmOutput, foreArmOutput);
    ShuffleControl.armTab.setTargetCoords(targetX, targetY);
    ShuffleControl.armTab.setTargetAngles(upperArmTargetAngle, foreArmTargetAngle);
    ShuffleControl.armTab.setEncoderAngles(upperArmEncoder.getDistance(), foreArmEncoder.getDistance());
    ShuffleControl.armTab.setControllerErrors(upperArmController.getPositionError(),
        foreArmController.getPositionError());
  }

  /**
   * Calculates the angles of the two arms from a given pose with
   * inverse kinematics.
   * 
   * @param pose The desired pose
   * @return An array where the 1st element is the angle between the upper arm and
   *         the robot, and the 2nd element is the angle between the fore arm and
   *         the upper arm. All angles are in degrees.
   */
  private double[] ik(double x, double y) {
    double a = cnst.UPPER_ARM_LENGTH;
    double b = cnst.FORE_ARM_LENGTH;

    double r = Math.sqrt(Math.pow(x, 2) + Math.pow(y, 2));

    double theta = Math.atan2(y, x);
    double alpha = Math.acos(
        (Math.pow(a, 2) + Math.pow(r, 2) - Math.pow(b, 2))
            / (2 * a * r));
    double beta = Math.asin(
        (r * Math.sin(alpha)) / b);

    alpha = Math.toDegrees(alpha);
    beta = Math.toDegrees(beta);
    theta = Math.toDegrees(theta);

    double foreArmAngle = alpha + theta - 90;
    double upperArmAngle = beta;

    // Return previous results if coordinates are invalid
    if (Double.isNaN(foreArmAngle) || Double.isNaN(upperArmAngle)) {
      return new double[] { foreArmTargetAngle, upperArmTargetAngle };
    }

    return new double[] { foreArmAngle, upperArmAngle };
  }

  /** Inverts the arm. */
  public void invert() {
    invert = !invert;
    setCoords(targetX, targetY);
  }

  /**
   * Sets the target angles for the arm
   * 
   * @param upperArm The target angle for the upper arm
   * @param foreArm  The target angle for the fore arm
   */
  private void setAngles(double upperArm, double foreArm) {
    if (!calibrated) {
      Logger.warn("ArmSub : Arm hasn't been calibrated yet!");
    }

    upperArmTargetAngle = MathUtil.clamp(upperArm, 0, 90);
    foreArmTargetAngle = MathUtil.clamp(foreArm, 0, 180);

    if (invert) {
      upperArmTargetAngle *= -1;
      foreArmTargetAngle *= -1;
    }

    upperArmController.setSetpoint(upperArmTargetAngle);
    foreArmController.setSetpoint(foreArmTargetAngle);
  }

  /**
   * Sets the target coordinates for the arm.
   * 
   * @param x The target x
   * @param y The target y
   */
  public void setCoord(Pair<Double, Double> coord, double x, double y) {
    targets.set(targets.indexOf(coord), new Pair<Double, Double> (x, y));
    double[] res = ik(targetX, targetY);
    setAngles(res[0], res[1]);
  }
  public void setDestCoord(double x, double y){
    setCoord(getFinalTarget(), x, y);
  }

  public void shiftCoord(Pair<Double, Double> coord, double x, double y){
    setCoord(coord, coord.getFirst() + x, coord.getSecond() + y);
  }
  public void shiftDestCoord(double x, double y){
    shiftCoord(getFinalTarget(), x, y);
  }

  /** Returns a {@link Command} that moves the arm up indefinitely. */
  public Command moveUp() {
    return this.run(() -> {
      shiftDestCoord(0, cnst.ARM_VELOCITY);
    });
  }

  /** Returns a {@link Command} that moves the arm down indefinitely. */
  public Command moveDown() {
    return this.run(() -> {
      shiftDestCoord(0, -cnst.ARM_VELOCITY);
    });
  }

  /** Returns a {@link Command} that moves the arm forward indefinitely. */
  public Command moveForward() {
    return this.run(() -> {
      shiftDestCoord(cnst.ARM_VELOCITY, 0);
    });
  }

  /** Returns a {@link Command} that moves the arm backward indefinitely. */
  public Command moveBack() {
    return this.run(() -> {
      shiftDestCoord(-cnst.ARM_VELOCITY, 0);
    });
  }

  // TODO: Update the coordinates on all the preconfigured methods
  /** @return A {@link Command} that moves the arm to the low field position */
  public Command lowField() {
    return moveTo(1, 0.5);
  }

  /** @return A {@link Command} that moves the arm to the far field position */
  public Command farField() {
    return moveTo(2, 0.5);
  }

  /** @return a {@link Command} that moves the arm to the lower rung position */
  public Command lowerRung() {
    return moveTo(1.5, 1);
  }

  /** @return a {@link Command} that moves the arm to the upper rung position */
  public Command upperRung() {
    return moveTo(2, 1.5);
  }

  public Command moveTo(double x, double y) {
    return new FunctionalCommand(
        () -> this.setDestCoord(x, y),
        () -> {
        },
        (interrupted) -> {
        },
        () -> foreArmController.atSetpoint() && upperArmController.atSetpoint(), this);
  }

  /**
   * Zeroes the upper and fore arm encoders.
   * Use this method when the upper arm pointing up and the fore arm is pointed
   * down.
   */
  public void calibrate() {
    upperArmEncoder.reset();
    foreArmEncoder.reset();
    setAngles(0, 0);
    calibrated = true;
    Logger.info("ArmSub : Calibrated!");
  }

  @Override
  public void periodic() {
    // PRAY TO GOD THAT THIS CODE WORKS.

    if (!calibrated) {
      // Don't do anything if no calibration has happened.
      updateShuffleboard(0, 0);
      return;
    }

    double upperArmOutput = upperArmController.calculate(upperArmEncoder.getDistance() * -1);
    double foreArmOutput = foreArmController.calculate(foreArmEncoder.getDistance());

    upperArmOutput = MathUtil.clamp(upperArmOutput, -0.2, 0.2);
    foreArmOutput = MathUtil.clamp(foreArmOutput, -0.2, 0.2);

    if (!upperArmController.atSetpoint()) {
      upperArmMotors.set(upperArmOutput);
    } else {
      upperArmMotors.stopMotor();
    }

    if (!foreArmController.atSetpoint()) {
      foreArmMotors.set(foreArmOutput);
    } else {
      foreArmMotors.stopMotor();
    }

    updateShuffleboard(upperArmOutput, foreArmOutput);
  }

  Pair<Double, Double> getFinalTarget(){
    if(targets.isEmpty()) return new Pair<Double, Double>(0.0,0.0);
    return targets.get(targets.size() - 1);
  }
  Pair<Double, Double> getCurTarget(){
    if(targets.isEmpty()) return new Pair<Double, Double>(0.0,0.0);
    return targets.get(0);
  }  

  boolean targetIsValid(double x, double y){
    if(x < cnst.MAX_ARM_REACH_LEGAL[0][0] || x > cnst.MAX_ARM_REACH_LEGAL[0][1]) return false;
    if(y < cnst.MAX_ARM_REACH_LEGAL[1][0] || y > cnst.MAX_ARM_REACH_LEGAL[1][1]) return false;
    //max arm reach is around the axle x is around robot center x is adjusted to be around axle
    //before checking
    if(Math.hypot(x + cnst.UPPER_ARM_X_OFFSET, y) > cnst.MAX_ARM_REACH_PHYSICAL) return false;
    return true;
  }
}
