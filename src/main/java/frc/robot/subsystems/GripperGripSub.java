package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class GripperGripSub extends SubsystemBase {
  private final Constants cnst = Constants.getInstance();
  private final Servo servoOne = new Servo(cnst.GRIPPER_GRIP_SERVO_1);
  private final Servo servoTwo = new Servo(cnst.GRIPPER_GRIP_SERVO_2);

  public GripperGripSub() {
    // https://cdn.andymark.com/media/W1siZiIsIjIwMTkvMDMvMjIvMTAvMjYvNDMvZjQzZTk3NzMtN2MxNi00MDIwLWE5YTgtMTA4MDliMTMxZDExL1VzaW5nIEwxNiBMaW5lYXIgU2Vydm8gMDMtMjAxOS5wZGYiXV0/Using%20L16%20Linear%20Servo%2003-2019.pdf?sha=7b43b981c4f1c13d
    servoOne.setBounds(2.0, 1.8, 1.5, 1.2, 1.0);
    servoTwo.setBounds(2.0, 1.8, 1.5, 1.2, 1.0);
  }

  /** Opens the gripper */
  public void open() {
    servoOne.set(1);
    servoTwo.set(1);
  }

  /** Closes the gripper */
  public void close() {
    servoOne.set(0);
    servoTwo.set(0);
  }
}
