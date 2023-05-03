package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;

/** Subsystem for controlling the gripper */
public class GripperSub extends SubsystemBase {
  private final Servo servos = new Servo(Constants.gripper.SERVOS_ID);

  public GripperSub() {
    // https://cdn.andymark.com/media/W1siZiIsIjIwMTkvMDMvMjIvMTAvMjYvNDMvZjQzZTk3NzMtN2MxNi00MDIwLWE5YTgtMTA4MDliMTMxZDExL1VzaW5nIEwxNiBMaW5lYXIgU2Vydm8gMDMtMjAxOS5wZGYiXV0/Using%20L16%20Linear%20Servo%2003-2019.pdf?sha=7b43b981c4f1c13d
    servos.setBounds(2.0, 1.8, 1.5, 1.2, 1.0);
  }

  /** Opens the gripper */
  public void open() {
    servos.set(1);
  }

  /** Closes the gripper for cones. */
  public void closeCone() {
    servos.set(0);
  }

  /** Closes the gripper for cubes. */
  public void closeCube() {
    // This uses a lower value than closeCone() because the cube could pop if too
    // much force is applied
    servos.set(0.4);
  }
}
