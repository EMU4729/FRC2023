package frc.robot.constants;

public class CrateConstants {
  protected CrateConstants() {
  }

  public final int PORT_1 = 0;
  public final int PORT_2 = 1;

  public static enum ShootMode {
    HOLD,
    PULSE,
    TOGGLE
  }

  public final ShootMode shootMode = ShootMode.TOGGLE;
}
