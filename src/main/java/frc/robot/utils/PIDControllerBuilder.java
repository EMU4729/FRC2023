package frc.robot.utils;

import edu.wpi.first.math.controller.PIDController;

public class PIDControllerBuilder {
  public final double kp;
  public final double ki;
  public final double kd;

  public PIDControllerBuilder(double kp, double ki, double kd) {
    this.kp = kp;
    this.ki = ki;
    this.kd = kd;
  }

  public PIDController build() {
    return new PIDController(kp, ki, ki);
  }
}
