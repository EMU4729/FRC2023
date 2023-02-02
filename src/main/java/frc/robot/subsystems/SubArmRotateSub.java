package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class SubArmRotateSub extends SubsystemBase {
  private final Constants cnst = Constants.getInstance();

  private final Servo servo = new Servo(cnst.SUBARM_PIVOT_SERVO);

  public void set(double value) {
    servo.set(value);
  }

  public Command rotateClockwise() {
    return this.run(() -> servo.set(servo.get() + 0.01));
  }

  public Command rotateAnticlockwise() {
    return this.run(() -> servo.set(servo.get() - 0.01));
  }
}
