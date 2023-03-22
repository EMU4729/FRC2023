package frc.robot.utils;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import frc.robot.utils.logger.Logger;

public class MotorBuilder {
  public final int port;
  public final Type type;

  public boolean invert = false;
  public boolean brake = false;
  public boolean safety = false;

  public enum Type {
    Never,
    TalonSRX,
    VictorSPX
  }

  public MotorBuilder(int motorPort, Type type) {
    this.port = motorPort;
    this.type = type;
  }

  public MotorBuilder withBrake() {
    brake = true;
    return this;
  }

  public MotorBuilder withInvert() {
    invert = true;
    return this;
  }

  public MotorBuilder withSafety() {
    safety = true;
    return this;
  }

  public MotorController build() {
    if (port < 0) {
      Logger.error("MotorInfo : motor port num < 0, check port is defined : " + port);
      return new WPI_TalonSRX(99);
    }

    switch (type) {
      case TalonSRX:
        WPI_TalonSRX talon = new WPI_TalonSRX(port);
        if (!talon.isAlive()) {
          Logger.warn(
              "MotorInfo : new WPI_TalonSRX on port " + port + "not found, may not exist or be of wrong type");
        }
        talon.setInverted(invert);
        if (brake) {
          talon.setNeutralMode(NeutralMode.Brake);
        } else {
          talon.setNeutralMode(NeutralMode.Coast);
        }
        talon.setSafetyEnabled(safety);
        return talon;

      case VictorSPX:
        WPI_VictorSPX victor = new WPI_VictorSPX(port);
        if (!victor.isAlive()) {
          Logger.warn(
              "MotorInfo : new WPI_VictorSPX on port " + port + "not found, may not exist or be of wrong type");
        }
        victor.setInverted(invert);
        if (brake) {
          victor.setNeutralMode(NeutralMode.Brake);
        } else {
          victor.setNeutralMode(NeutralMode.Coast);
        }
        victor.setSafetyEnabled(safety);
        return victor;

      case Never:
        Logger.error("MotorInfo : controller type not set");
        return new WPI_TalonSRX(99);

      default:
        Logger.error("MotorInfo : controller type not found");
        return new WPI_TalonSRX(99);
    }
  }

}
