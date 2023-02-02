package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class SubArmRotateSub extends SubsystemBase {
  private final Constants cnst = Constants.getInstance();

  private final Servo servo = new Servo(cnst.SUBARM_PIVOT_SERVO);

  /**
   * Sets the value of the servo.
   * 
   * @param value The value to set the servo to.
   */
  public void set(double value) {
    value = MathUtil.clamp(value, 0, 1);
    servo.set(value);
  }

  /** @return a {@link Command} to rotate the subarm clockwise */
  public Command clockwise() {
    return this.run(() -> set(servo.get() + 0.01));
  }

  /** @return a {@link Command} to rotate the subarm anticlockwise */
  public Command anticlockwise() {
    return this.run(() -> set(servo.get() - 0.01));
  }
}
