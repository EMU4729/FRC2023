package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.shufflecontrol.ShuffleControl;
import frc.robot.utils.CurveFit;
import frc.robot.utils.logger.Logger;

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

  private CurveFit upperCurve = new CurveFit(-0.5, 0.5, 0.1, 0.5, 1);
  private CurveFit lowerCurve = new CurveFit(-0.5, 0.5, 0.1, 0.5, 1);

  private List<Pair<Double, Double>> targets = new ArrayList<Pair<Double, Double>>();
  private double upperArmTargetAngle;
  private double foreArmTargetAngle;
  private Pair<Double, Double> prevArmTargetPoints;

  public ArmSub() {
    targets.add(0, new Pair<Double, Double>(0.0, 0.0));
    upperArmController.setTolerance(0);
    foreArmController.setTolerance(0);
  }

  /**
   * Updates the arm tab in shuffleboard. Call this function regularly.
   * 
   * @param upperArmOutput The speed of the upper arm motors
   * @param foreArmOutput  The speed of the fore arm motors
   */
  private void updateShuffleboard(double upperArmOutput, double foreArmOutput) {
    ShuffleControl.armTab.setOutputs(upperArmOutput, foreArmOutput);
    ShuffleControl.armTab.setTargetCoords(getFinalTarget().getFirst(), getFinalTarget().getSecond());
    ShuffleControl.armTab.setTargetAngles(upperArmTargetAngle, foreArmTargetAngle);
    ShuffleControl.armTab.setEncoderAngles(upperArmEncoder.getDistance(), foreArmEncoder.getDistance());
    ShuffleControl.armTab.setControllerErrors(upperArmController.getPositionError(),
        foreArmController.getPositionError());
    ShuffleControl.calibrationTab.setArmCalibrated(calibrated);
  }

  /**
   * Forward kinematics.
   * 
   * @param upperArmAngle The angle of the upper arm
   * @param foreArmAngle  The angle of the fore arm
   * @return The calculated coordinates of the end of the arm.
   */
  private Pair<Double, Double> forK(double upperArmAngle, double foreArmAngle) {
    foreArmAngle *= -1;
    double l1 = cnst.UPPER_ARM_LENGTH;
    double l2 = cnst.FORE_ARM_LENGTH;

    double x1 = l1 * Math.cos(Math.toRadians(upperArmAngle + 90));
    double y1 = l1 * Math.sin(Math.toRadians(upperArmAngle + 90));

    double x2 = l2 * Math.cos(Math.toRadians(foreArmAngle - (180 - (upperArmAngle + 90)))) + x1;
    double y2 = l2 * Math.sin(Math.toRadians(foreArmAngle - (180 - (upperArmAngle + 90)))) + y1;

    return new Pair<Double, Double>(-x2, y2);
  }

  /**
   * Forward kinematics. When run without arguments, this uses the arms' encoder
   * values.
   * 
   * @return The calculated coordinates of the end of the arm.
   */
  private Pair<Double, Double> forK() {
    return forK(upperArmEncoder.getDistance(), foreArmEncoder.getDistance());
  }

  /** @return The angle that the end of the arm makes with the robot horizontal */
  public double getEndAngle() {
    Pair<Double, Double> endPoint = forK();
    return Math.toDegrees(Math.atan2(endPoint.getSecond(), endPoint.getFirst()));
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
  private Pair<Double, Double> invK(double x, double y) {
    double xSign = Math.signum(x);
    x = Math.abs(x);

    double L1 = cnst.UPPER_ARM_LENGTH;
    double L2 = cnst.FORE_ARM_LENGTH;

    double r = Math.hypot(x, y);

    double theta = Math.atan2(y, x);
    double alpha = invCosRule(L1, r, L2);
    double beta = invCosRule(L2, L1, r);

    alpha = Math.toDegrees(alpha);
    beta = Math.toDegrees(beta);
    theta = Math.toDegrees(theta);

    double foreArmAngle = xSign * (alpha + theta - 90);
    double upperArmAngle = xSign * beta;

    // Return previous results if coordinates are invalid
    if (Double.isNaN(foreArmAngle) || Double.isNaN(upperArmAngle)) {
      return new Pair<Double, Double>(upperArmTargetAngle, foreArmTargetAngle);
    }

    return new Pair<Double, Double>(upperArmAngle, foreArmAngle);
  }

  /**
   * Inverts the arm.
   * @apiNote Disabled until further notice.
   */
  public void invert() {
    // Disable inversion until confirmed to be good
    if (true) {
      return;
    }
    invert = !invert;
    double tmp1 = invert ? cnst.ARM_REACH_EXCLUSION[0][0] : cnst.ARM_REACH_EXCLUSION[0][1];
    double tmp2 = invert ? cnst.ARM_REACH_EXCLUSION[0][1] : cnst.ARM_REACH_EXCLUSION[0][0];
    addCoord(0, tmp1, cnst.ARM_SWING_THROUGH_HEIGHT, false);
    addCoord(1, tmp2, cnst.ARM_SWING_THROUGH_HEIGHT, false);
    Pair<Double, Double> tmp = getCurTarget();
    setDestCoord(-tmp.getFirst(), tmp.getSecond(), true);
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

    upperArmTargetAngle = MathUtil.clamp(upperArm, -85, 85);
    foreArmTargetAngle = MathUtil.clamp(foreArm, -180, 180);

    upperArmController.setSetpoint(upperArmTargetAngle);
    foreArmController.setSetpoint(foreArmTargetAngle);
  }

  /**
   * Sets the target coordinates for the arm.
   * 
   * @param x The target x
   * @param y The target y
   */
  public void setCoord(Pair<Double, Double> coord, double x, double y, boolean invertable) {
    if (!targetIsValid(x, y)) { // if target coord is not allowed stay still
      Logger.warn("ArmSub : setCoords : dest is not allowed");
      Pair<Double, Double> tmp = getCurTarget();
      x = tmp.getFirst();
      y = tmp.getSecond();
    }
    int inv = invert && invertable ? -1 : 1;
    targets.set(targets.indexOf(coord), new Pair<Double, Double>(x * inv, y));
    Pair<Double, Double> tmp = getCurTarget();
    Pair<Double, Double> res = invK(tmp.getFirst(), tmp.getSecond());
    setAngles(res.getFirst(), res.getSecond());
  }

  public void setDestCoord(double x, double y, boolean invertable) {
    setCoord(getFinalTarget(), x, y, invertable);
  }

  public void shiftCoord(Pair<Double, Double> coord, double x, double y, boolean invertable) {
    setCoord(coord, coord.getFirst() + x, coord.getSecond() + y, invertable);
  }

  public void shiftDestCoord(double x, double y, boolean invertable) {
    shiftCoord(getFinalTarget(), x, y, invertable);
  }

  public void addCoord(int idx, double x, double y, boolean invertable) {
    if (idx > targets.size()) {
      Logger.warn("ArmSub : addCoord : idx of new too large check code -1 = end");
      idx = targets.size();
    }
    if (idx < 0)
      idx = targets.size();

    if (!targetIsValid(x, y)) { // if target coord is not allowed stay still
      Logger.warn("ArmSub : setCoords : dest is not allowed");
      Pair<Double, Double> tmp = getCurTarget();
      x = tmp.getFirst();
      y = tmp.getSecond();
    }
    int inv = invert && invertable ? -1 : 1;
    targets.add(idx, new Pair<Double, Double>(x * inv, y));
    Pair<Double, Double> tmp = getCurTarget();
    Pair<Double, Double> res = invK(tmp.getFirst(), tmp.getSecond());
    setAngles(res.getFirst(), res.getSecond());
  }

  /** Returns a {@link Command} that moves the arm up indefinitely. */
  public Command moveUp() {
    return this.run(() -> {
      shiftDestCoord(0, cnst.ARM_VELOCITY, true);
    });
  }

  /** Returns a {@link Command} that moves the arm down indefinitely. */
  public Command moveDown() {
    return this.run(() -> {
      shiftDestCoord(0, -cnst.ARM_VELOCITY, true);
    });
  }

  /** Returns a {@link Command} that moves the arm forward indefinitely. */
  public Command moveForward() {
    return this.run(() -> {
      shiftDestCoord(cnst.ARM_VELOCITY, 0, true);
    });
  }

  /** Returns a {@link Command} that moves the arm backward indefinitely. */
  public Command moveBack() {
    return this.run(() -> {
      shiftDestCoord(-cnst.ARM_VELOCITY, 0, true);
    });
  }

  // TODO: Update the coordinates on all the preconfigured methods
  /** @return A {@link Command} that moves the arm to the low field position */
  public Command lowField() {
    return moveTo(1, 0.0);
  }

  /** @return A {@link Command} that moves the arm to the far field position */
  public Command farField() {
    return moveTo(1.5, 0.0);
  }

  /** @return a {@link Command} that moves the arm to the lower rung position */
  public Command lowerRung() {
    return moveTo(1, 1);
  }

  /** @return a {@link Command} that moves the arm to the upper rung position */
  public Command upperRung() {
    return moveTo(1.4, 1.4);
  }

  public Command moveTo(double x, double y) {
    return new FunctionalCommand(
        () -> this.setDestCoord(x, y, true),
        () -> {
        },
        (interrupted) -> {
        },
        () -> {
          if (RobotBase.isSimulation()) {
            Logger.info("ArmSub::moveTo : In simulation, skipping...");
            return true;
          }

          return foreArmController.atSetpoint() && upperArmController.atSetpoint();
        }, this);
  }

  /**
   * Zeroes the upper and fore arm encoders.
   * Use this method when the upper arm pointing up and the fore arm is pointed
   * down.
   */
  public void calibrate() {
    upperArmEncoder.reset();
    foreArmEncoder.reset();
    calibrated = true;
    setAngles(0, 0);
    //addCoord(0, cnst.ARM_REACH_EXCLUSION[0][0], cnst.ARM_SWING_THROUGH_HEIGHT, false);
    setDestCoord(0.5, 0, false);
    Logger.info("ArmSub : Calibrated!");
  }

  Pair<Double, Double> getFinalTarget() {
    if (targets.isEmpty())
      return new Pair<Double, Double>(0.0, 0.0);
    return targets.get(targets.size() - 1);
  }

  Pair<Double, Double> getCurTarget() {
    if (targets.isEmpty())
      return new Pair<Double, Double>(0.0, 0.0);
    return targets.get(0);
  }

  /** Kills the robot if an illegal arm angle is reached. */
  private void killCheck() {
    double upperArmAngle = upperArmEncoder.getDistance();
    double foreArmAngle = foreArmEncoder.getDistance();

    if (Math.abs(upperArmAngle) > 90 || Math.abs(foreArmAngle) > 185) {
      throw new IllegalStateException(
          String.format("ArmSub::killCheck : Illegal angles reached, killing robot! (foreArm: %f, upperArm: %f)",
              foreArmAngle, upperArmAngle));
    }
  }

  /**
   * Checks if the given coordinates are valid for the arm.
   * 
   * @param x The coordinate x
   * @param y The coordinate y
   * @return True if valid, false if not
   */
  boolean targetIsValid(double x, double y) {
    // check legal reach limits
    if (x < cnst.MAX_ARM_REACH_LEGAL[0][0] || x > cnst.MAX_ARM_REACH_LEGAL[0][1])
      return false;
    if (y < cnst.MAX_ARM_REACH_LEGAL[1][0] || y > cnst.MAX_ARM_REACH_LEGAL[1][1])
      return false;

    // max arm reach is around the axle x is around robot center x is adjusted to be
    // around axle before checking
    if (Math.hypot(x + cnst.UPPER_ARM_X_OFFSET, y) > cnst.MAX_ARM_REACH_PHYSICAL)
      return false;

    // check arm swing-through bounds
    if (x > cnst.ARM_REACH_EXCLUSION[0][0] && x < cnst.ARM_REACH_EXCLUSION[0][1]
        && !(y > cnst.ARM_SWING_THROUGH_HEIGHT * 0.95 && y < cnst.ARM_SWING_THROUGH_HEIGHT * 1.05)) {
      return false;
    }

    // check robot limits
    if (x > cnst.ARM_REACH_ROBOT_EXCLUSION[0][0] && x < cnst.ARM_REACH_ROBOT_EXCLUSION[0][1]
        && y > cnst.ARM_REACH_ROBOT_EXCLUSION[1][0] && y < cnst.ARM_REACH_ROBOT_EXCLUSION[1][1]) {
      return false;
    }

    return true;
  }

  /**
   * Inverse cosine rule.
   * 
   * @param a Length of side a
   * @param b Length of side b
   * @param c Length of side c
   * @return The included angle C
   */
  double invCosRule(double a, double b, double c) {
    double C = Math.acos(
        (Math.pow(a, 2) + Math.pow(b, 2) - Math.pow(c, 2))
            / (2 * a * b));
    return C;
  }

  /**
   * Interpolates the path to the destination point.
   * 
   * @return The next interpolated point to move to.
   */
  Pair<Double, Double> interpolateNext() {
    Pair<Double, Double> curPos = forK();
    double x1 = curPos.getFirst();
    double y1 = curPos.getSecond();

    Pair<Double, Double> destPos = getFinalTarget();
    double x2 = destPos.getFirst();
    double y2 = destPos.getSecond();

    double angle = Math.atan2(y2 - y1, x2 - x1);

    double changeX = cnst.ARM_INTERPOLATION_STEP * Math.cos(angle);
    double changeY = cnst.ARM_INTERPOLATION_STEP * Math.sin(angle);

    if (Math.hypot(changeX, changeY) < cnst.ARM_INTERPOLATION_STEP) {
      return destPos;
    }

    double x = changeX + x1;
    double y = changeY + y1;

    return new Pair<Double, Double>(x, y);
  }

  @Override
  public void periodic() {
    // PRAY TO GOD THAT THIS CODE WORKS.

    if (!calibrated) {
      // Don't do anything if no calibration has happened.
      updateShuffleboard(0, 0);
      return;
    }

    //killCheck();

    Pair<Double, Double> kinematicsCoords = forK(upperArmEncoder.getDistance(), foreArmEncoder.getDistance());
    ShuffleControl.armTab.setKinematicsCoords(kinematicsCoords.getFirst(), kinematicsCoords.getSecond());

    double upperArmOutput = upperArmController.calculate(upperArmEncoder.getDistance() * -1);
    double foreArmOutput = foreArmController.calculate(foreArmEncoder.getDistance());

    upperArmOutput = MathUtil.clamp(upperArmOutput, -0.2, 0.2);
    foreArmOutput = MathUtil.clamp(foreArmOutput, -0.2, 0.2);

    if (!(upperArmController.atSetpoint() && foreArmController.atSetpoint())) {
      Pair<Double, Double> nextPoint = interpolateNext();
      Pair<Double, Double> res = invK(nextPoint.getFirst(), nextPoint.getSecond());
      setAngles(res.getFirst(), res.getSecond());
    }

    if (!upperArmController.atSetpoint()) {
      upperArmMotors.set(upperCurve.fit(MathUtil.applyDeadband(upperArmOutput, 0.01)));
    } else {
      upperArmMotors.stopMotor();

      if (foreArmController.atSetpoint() && prevArmTargetPoints != getFinalTarget()) { // if at both setpoints
        if (targets.size() > 1)
          targets.remove(0);
        Pair<Double, Double> tmp = getCurTarget();
        Pair<Double, Double> res = invK(tmp.getFirst(), tmp.getSecond());
        setAngles(res.getFirst(), res.getSecond());
      }
    }

    if (!foreArmController.atSetpoint()) {
      foreArmMotors.set(lowerCurve.fit(MathUtil.applyDeadband(foreArmOutput, 0.01)));
    } else {
      foreArmMotors.stopMotor();
    }

    updateShuffleboard(upperArmOutput, foreArmOutput);
  }
}
