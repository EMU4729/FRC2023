package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class SubArmRotateSub extends SubsystemBase {
  private final Constants cnst = Constants.getInstance();

  private final Servo servo = new Servo(cnst.SUBARM_PIVOT_SERVO);

  /** Stops the servo. */
  public void stop() {
    servo.set(0.5);
  }

  /** Starts running the servo clockwise. */
  public void clockwise() {
    servo.set(1);
  }

  /** Starts running the servo anticlockwise. */
  public void anticlockwise() {
    servo.set(0);
  }
}
