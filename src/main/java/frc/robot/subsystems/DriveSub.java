package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Subsystems;
import frc.robot.utils.CurveFit;
// import frc.robot.Variables;

/**
 * Drive Subsystem.
 * Handles all drive functionality.
 */
public class DriveSub extends SubsystemBase {
  private final Constants cnst = Constants.getInstance();
  // private final Variables vars = Variables.getInstance();

  private final MotorController leftMaster = cnst.DRIVE_MOTOR_ID_LM.createMotorController();
  private final MotorController leftSlave = cnst.DRIVE_MOTOR_ID_LS.createMotorController();
  private final MotorControllerGroup leftMotors = new MotorControllerGroup(leftMaster, leftSlave);

  private final MotorController rightMaster = cnst.DRIVE_MOTOR_ID_RM.createMotorController();
  private final MotorController rightSlave = cnst.DRIVE_MOTOR_ID_RS.createMotorController();
  private final MotorControllerGroup rightMotors = new MotorControllerGroup(rightMaster, rightSlave);

  public final DifferentialDrive drive = new DifferentialDrive(leftMotors, rightMotors); // pub for shuffleboard

  public final PIDController pidThrot = new PIDController(0, 0, 0); // pub for shuffleboard
  public final PIDController pidSteer = new PIDController(0, 0, 0); // pub for shuffleboard

  private CurveFit pidThrotCurve;
  private CurveFit pidTurnCurve;

  public DriveSub() {
    pidThrot.setPID(cnst.TELEOP_THROTTLE_PID[0], cnst.TELEOP_THROTTLE_PID[1],
        cnst.TELEOP_THROTTLE_PID[2]);
    pidSteer.setPID(cnst.TELEOP_STEERING_PID[0], cnst.TELEOP_STEERING_PID[1],
        cnst.TELEOP_STEERING_PID[2]);
    addChild("Differential Drive", drive);
  }

  /**
   * Activates tank drive. Similar to MoveTank from ev3dev.
   * 
   * @param leftSpeed  The left speed.
   * @param rightSpeed The right speed.
   */
  public void tank(double leftSpeed, double rightSpeed) {
    leftSpeed = MathUtil.clamp(leftSpeed, -1, 1);
    rightSpeed = MathUtil.clamp(rightSpeed, -1, 1);
    drive.tankDrive(leftSpeed, rightSpeed);
  }

  public void pidArcadeSetup(double[][] settings) {
    pidThrotCurve = new CurveFit(settings[0][0], settings[0][1], settings[0][2]);
    pidTurnCurve = new CurveFit(settings[1][0], settings[1][1], settings[1][2]);
  }

  /**
   * runs a pid loop to drive at set speed and turn rate
   * 
   * @param speed    speed to drive at m/s
   * @param turnRate rate to turn at deg/s
   */
  public void pidArcade(double speed, double turnRate) {

    double throttle = pidThrot.calculate(Subsystems.nav.speed, speed);
    throttle = pidThrotCurve.fit(throttle);
    double steering = 0;
    arcade(throttle, steering);
  }

  /**
   * Activates arcade drive. Similar to MoveSteering from ev3dev.
   * 
   * @param throttle The speed
   * @param steering The steering
   */
  public void arcade(double throttle, double steering) {
    throttle = MathUtil.clamp(throttle, -1, 1);
    steering = MathUtil.clamp(steering, -1, 1);
    drive.arcadeDrive(throttle, steering);

  }

  /** Stop all motors. */
  public void off() {
    drive.stopMotor();
  }
}
