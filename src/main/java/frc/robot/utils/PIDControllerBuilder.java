package frc.robot.utils;

import java.util.Optional;

import edu.wpi.first.math.controller.PIDController;

/** Utility class to build a {@link PIDController} */
public class PIDControllerBuilder {
  public final double kp;
  public final double ki;
  public final double kd;
  public final Optional<Double> setpoint;

  public PIDControllerBuilder(double kp, double ki, double kd) {
    this.kp = kp;
    this.ki = ki;
    this.kd = kd;
    this.setpoint = Optional.empty();
  }

  public PIDControllerBuilder(double kp, double ki, double kd, double setpoint) {
    this.kp = kp;
    this.ki = ki;
    this.kd = kd;
    this.setpoint = Optional.of(setpoint);
  }

  public PIDController build() {
    PIDController controller = new PIDController(kp, ki, kd);
    if (setpoint.isPresent())
      controller.setSetpoint(setpoint.get());
    return controller;
  }
}
