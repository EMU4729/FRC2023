package frc.robot.simulation;

import java.util.Optional;

public class SimConstants {
  private static Optional<SimConstants> inst = Optional.empty();

  private SimConstants() {}

  public static SimConstants getInstance() {
    if (inst.isEmpty()) inst = Optional.of(new SimConstants());
    return inst.get();
  }
  

  public final double KV_LINEAR = 1.98;
  public final double KA_LINEAR = 0.2;
  public final double KV_ANGULAR = 1.5;
  public final double KA_ANGULAR = 0.3;
}
