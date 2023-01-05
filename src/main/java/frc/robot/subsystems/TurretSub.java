package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
// import frc.robot.Variables;
import frc.robot.utils.logger.Logger;

public class TurretSub extends SubsystemBase {
  private final Constants cnst = Constants.getInstance();
  // private final Variables vars = Variables.getInstance();

  private final MotorController slew = cnst.TURRET_SLEW_MOTOR_ID.createMotorController();
  private final Encoder slewEncoder = cnst.TURRET_SLEW_MOTOR_ID.createEncoder();
  // private final MotorController hood =
  // cnst.TURRET_HOOD_MOTOR_ID.createMotorController();
  // private final Encoder hoodEncoder =
  // cnst.TURRET_HOOD_MOTOR_ID.createEncoder();
  private final DigitalInput slewLimit = new DigitalInput(cnst.TURRET_SLEW_LIMIT);
  // private final DigitalInput hoodLimit = new
  // DigitalInput(cnst.TURRET_HOOD_LIMIT);

  private final PIDController slewController = new PIDController(cnst.TURRET_SLEW_PID[0],
      cnst.TURRET_SLEW_PID[1], cnst.TURRET_SLEW_PID[2]);

  private double angle = 0;
  private boolean initialized = false;

  public final Command initSlewCommand = new FunctionalCommand(() -> {
    Logger.info("Turret : Slew Initialization Started");
    setSpeed(-0.15);
  }, () -> {
  }, interrupted -> {
    slew.stopMotor();
    slewEncoder.reset();
    initialized = true;
    Logger.info("Turret : Slew Initialization Finished");
  }, () -> {
    return !slewLimit.get();
  }, this);

  @Override
  public void periodic() {
    SmartDashboard.putData(slewEncoder);

    angle = SmartDashboard.getNumber("Turret Angle", 0);
    SmartDashboard.putNumber("Turret Angle", angle);
    
    // Slew PID
    if (!initialized)
      return;
    slewController.setSetpoint(angle);
    double output = slewController.calculate(slewEncoder.getDistance());
    SmartDashboard.putNumber("Slew PID Output", output);
    setSpeed(output);
  }

  /**
   * Sets the target angle for the turret slew.
   * 
   * @param angle The desired angle from 0 to 300 (degrees).
   */
  public void setAngle(double angle) {
    if (!initialized) {
      Logger.error("Turret : TurretSub#setAngle() called before initialization");
      return;
    }
    this.angle = MathUtil.clamp(angle, cnst.TURRET_HOOD_RANGE[0], cnst.TURRET_HOOD_RANGE[1]);
  }

  /**
   * Sets the speed of the turret slew.
   * 
   * @param speed Speed, from -1 to 1
   */
  public void setSpeed(double speed) {
    speed = MathUtil.clamp(speed, -1, 1);
    slew.set(speed);
  }
}
