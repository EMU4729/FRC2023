package frc.robot;

import java.util.Optional;

import frc.robot.subsystems.DriveSub;
import frc.robot.subsystems.NavigationSub;
// import frc.robot.subsystems.TurretSub;
import frc.robot.subsystems.TurretSub;

/**
 * Subsystems - Use this class to initialize and access all subsystems globally.
 */
public class Subsystems {
  // private final Constants cnst = Constants.getInstance();

  public static NavigationSub nav = new NavigationSub();
  public static DriveSub drive = new DriveSub();
  public static TurretSub turret = new TurretSub();

  private Subsystems() {}

}
