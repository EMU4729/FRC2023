package frc.robot.utils;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.GenericHID;

/**
 * A {@link BooleanSupplier} that gets its state from a {@link GenericHID}'s
 * axis and a
 * provided threshold.
 */
public class AxisButtonSupplier implements BooleanSupplier {
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
  public AxisButtonSupplier(GenericHID joystick, int axisNumber, double threshold) {
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
  public boolean getAsBoolean() {
    return joystick.getRawAxis(axisNumber) > threshold;
  }
}