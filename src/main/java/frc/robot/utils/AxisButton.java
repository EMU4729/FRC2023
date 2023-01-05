package frc.robot.utils;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.button.Button;

/**
 * A {@link Button} that gets its state from a {@link GenericHID}'s axis and a
 * provided threshold.
 */
public class AxisButton extends Button {
  private final GenericHID joystick;
  private final int axisNumber;
  private final double threshold;

  /**
   * Creates an axis button for triggering commands.
   * 
   * @param joystick   The GenericHID object that has the axis.
   * @param axisNumber The axis number
   * @param threshold  The threshold value for the axis that triggers the button
   */
  public AxisButton(GenericHID joystick, int axisNumber, double threshold) {
    this.joystick = joystick;
    this.axisNumber = axisNumber;
    this.threshold = threshold;
  }

  /**
   * Gets the value of the axis button
   * 
   * @return The value of the axis button
   */
  @Override
  public boolean get() {
    return joystick.getRawAxis(axisNumber) > threshold;
  }
}