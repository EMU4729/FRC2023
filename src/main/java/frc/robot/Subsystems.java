package frc.robot;

import frc.robot.subsystems.CrateSub;
import frc.robot.subsystems.DriveSub;
import frc.robot.subsystems.LEDSub;
import frc.robot.subsystems.NavigationSub;

/**
 * Subsystems - Use this class to initialize and access all subsystems globally.
 */
public class Subsystems {
  public static final NavigationSub nav = new NavigationSub();
  public static final DriveSub drive = new DriveSub();
  public static final CrateSub crate = new CrateSub();
  public static final LEDSub led = new LEDSub();
}
